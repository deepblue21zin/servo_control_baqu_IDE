#include "app_runtime.h"

#include "main.h"
#include "can_runtime.h"
#include "gpio.h"
#include "iwdg.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"

#include "constants.h"
#include "encoder_reader.h"
#include "ethernet_communication.h"
#include "latency_profiler.h"
#include "project_params.h"
#include "position_control.h"
#include "position_control_diag.h"
#include "pulse_control.h"
#include "relay_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern volatile uint8_t interrupt_flag;

static uint32_t g_latency_report_seq = 0U;
static uint32_t g_debug_print_divider = 0U;
#if APP_RUNTIME_CAN_ENABLE
static uint8_t g_can_rx_log_enabled = (uint8_t)APP_RUNTIME_CAN_RX_LOG_ENABLE;
#endif
#if APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD
typedef enum {
    KB_SCENARIO_IDLE = 0,
    KB_SCENARIO_RAMP_UP,
    KB_SCENARIO_PEAK_HOLD,
    KB_SCENARIO_RAMP_DOWN
} AppRuntime_KeyboardScenarioPhase_t;

typedef struct {
    uint8_t active;
    AppRuntime_KeyboardScenarioPhase_t phase;
    uint32_t next_step_ms;
} AppRuntime_KeyboardScenario_t;

static float g_keyboard_target_steer_deg = 0.0f;
static char g_keyboard_line_buf[32] = {0};
static uint8_t g_keyboard_line_len = 0U;
static AppRuntime_KeyboardScenario_t g_keyboard_scenario = {0};
static float AppRuntime_KeyboardClampSteeringDeg(float steering_deg);
static void AppRuntime_KeyboardScenarioStop(const char *reason);
#else
static SteerMode_t g_prev_mode = STEER_MODE_NONE;
static SteerMode_t g_current_mode = STEER_MODE_NONE;
#endif
#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
static uint8_t g_periodic_csv_enabled = 1U;
#endif

#if APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE
typedef struct {
    int64_t accum_count;
    float count_residual;
} AppRuntime_VirtualEncoder_t; /* Putty-only encoder estimator from applied pulse output. */

static AppRuntime_VirtualEncoder_t g_virtual_encoder = {0};

/* Reset the bench-only display encoder derived from pulse output. */
static void AppRuntime_ResetVirtualEncoder(void)
{
    g_virtual_encoder.accum_count = 0;
    g_virtual_encoder.count_residual = 0.0f;
    EncoderReader_SetVirtualFeedbackCount(0);
}

/* Approximate encoder motion from the currently applied pulse frequency. */
static void AppRuntime_UpdateVirtualEncoder(void)
{
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    float delta_pulses = 0.0f;
    float delta_counts = 0.0f;
    float total_counts = 0.0f;
    int32_t whole_counts = 0;

    if ((pulse_status.output_active == 0U) || (pulse_status.applied_frequency_hz == 0U)) {
        EncoderReader_SetVirtualFeedbackCount(g_virtual_encoder.accum_count);
        return;
    }

    delta_pulses = ((float)pulse_status.applied_frequency_hz) * 0.001f;
    if (pulse_status.direction == DIR_CCW) {
        delta_pulses = -delta_pulses;
    }

    delta_counts = delta_pulses * (DEG_PER_PULSE / ENCODER_DEG_PER_COUNT);
    total_counts = g_virtual_encoder.count_residual + delta_counts;
    whole_counts = (int32_t)total_counts;

    g_virtual_encoder.accum_count += (int64_t)whole_counts;
    g_virtual_encoder.count_residual = total_counts - (float)whole_counts;
    EncoderReader_SetVirtualFeedbackCount(g_virtual_encoder.accum_count);
}

/* Return the Putty display count derived from the commanded motion. */
static int32_t AppRuntime_GetDisplayEncoderCount(void)
{
    return (int32_t)g_virtual_encoder.accum_count;
}

/* Return a timer-like raw counter value for Putty display only. */
static uint32_t AppRuntime_GetDisplayEncoderRaw(void)
{
    int32_t raw32 = 32768 + (int32_t)g_virtual_encoder.accum_count;
    return (uint32_t)((uint16_t)raw32);
}
#else
#define AppRuntime_ResetVirtualEncoder() ((void)0)
#define AppRuntime_UpdateVirtualEncoder() ((void)0)
static int32_t AppRuntime_GetDisplayEncoderCount(void)
{
    return EncoderReader_GetCount();
}

static uint32_t AppRuntime_GetDisplayEncoderRaw(void)
{
    return EncoderReader_GetRawCounter();
}
#endif

/* Convert a steering-angle target into the equivalent motor-angle target. */
static float AppRuntime_TargetSteeringDegToMotorDeg(float steering_deg)
{
    return SteeringDegToMotorDeg(steering_deg);
}

/* Convert a motor-angle reading back into steering-angle units. */
static float AppRuntime_TargetMotorDegToSteeringDeg(float motor_deg)
{
    return MotorDegToSteeringDeg(motor_deg);
}

/* Print which command-input build profile is active so bench logs explain why UDP is or isn't serviced. */
static void AppRuntime_PrintInputSourceSummary(void)
{
#if APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD
    printf("[AppRuntime] input=KEYBOARD_TEST. UDP path is intentionally not serviced in this build.\r\n");
    printf("[AppRuntime] switch APP_RUNTIME_INPUT_SOURCE to APP_RUNTIME_INPUT_SOURCE_UDP in project_params.h for real UDP tests.\r\n");
#else
    printf("[AppRuntime] input=UDP. Keyboard bench path is disabled in this build.\r\n");
    printf("[AppRuntime] configure UDP sender/packet settings in project_params.h before real tests.\r\n");
#endif
}

#if APP_RUNTIME_CAN_ENABLE
static int16_t AppRuntime_CanDegToDeciDeg(float deg)
{
    if (deg > 3276.7f) {
        return 32767;
    }
    if (deg < -3276.8f) {
        return -32768;
    }
    return (int16_t)(deg * 10.0f);
}

static void AppRuntime_CanPrintFrame(const char *tag, const CAN_Frame_t *frame)
{
    printf("[CAN][%s] id=0x%03lX dlc=%u ext=%u rtr=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           tag,
           (unsigned long)frame->id,
           (unsigned int)frame->dlc,
           (unsigned int)frame->is_extended,
           (unsigned int)frame->is_remote,
           (unsigned int)frame->data[0],
           (unsigned int)frame->data[1],
           (unsigned int)frame->data[2],
           (unsigned int)frame->data[3],
           (unsigned int)frame->data[4],
           (unsigned int)frame->data[5],
           (unsigned int)frame->data[6],
           (unsigned int)frame->data[7]);
}

static void AppRuntime_CanSendStatusFrame(void)
{
    PositionControl_State_t state = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    int16_t current_x10 = AppRuntime_CanDegToDeciDeg(AppRuntime_TargetMotorDegToSteeringDeg(state.current_angle));
    int16_t target_x10 = AppRuntime_CanDegToDeciDeg(AppRuntime_TargetMotorDegToSteeringDeg(state.target_angle));
    int16_t error_x10 = AppRuntime_CanDegToDeciDeg(AppRuntime_TargetMotorDegToSteeringDeg(state.error));
    uint8_t payload[8];
    int ret = 0;

    payload[0] = (uint8_t)(current_x10 & 0xFF);
    payload[1] = (uint8_t)((current_x10 >> 8) & 0xFF);
    payload[2] = (uint8_t)(target_x10 & 0xFF);
    payload[3] = (uint8_t)((target_x10 >> 8) & 0xFF);
    payload[4] = (uint8_t)(error_x10 & 0xFF);
    payload[5] = (uint8_t)((error_x10 >> 8) & 0xFF);
    payload[6] = (uint8_t)state.mode;
    payload[7] = (uint8_t)cmd.state;

    ret = CAN_Runtime_SendStd((uint16_t)APP_RUNTIME_CAN_STATUS_STDID, payload, 8U);
    printf("[CAN] status tx ret=%d id=0x%03X cur=%.2f tgt=%.2f err=%.2f mode=%d cmd=%d\r\n",
           ret,
           APP_RUNTIME_CAN_STATUS_STDID,
           AppRuntime_TargetMotorDegToSteeringDeg(state.current_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(state.target_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(state.error),
           (int)state.mode,
           (int)cmd.state);
}

static void AppRuntime_HandleCanControlCommand(uint8_t command)
{
    switch (command) {
    case 0U:
        PositionControl_Disable();
        printf("[CAN] control command: disable\r\n");
        break;
    case 1U:
        PositionControl_Enable();
        printf("[CAN] control command: enable\r\n");
        break;
    case 2U:
        PositionControl_EmergencyStop();
        printf("[CAN] control command: estop\r\n");
        break;
    case 3U:
#if APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD
        AppRuntime_KeyboardScenarioStop("CAN center command");
        g_keyboard_target_steer_deg = 0.0f;
#endif
        PositionControl_SetTargetWithSource(AppRuntime_TargetSteeringDegToMotorDeg(0.0f), CMD_SRC_SERVICE);
        printf("[CAN] control command: center\r\n");
        break;
    case 4U:
        AppRuntime_CanSendStatusFrame();
        break;
    default:
        printf("[CAN] unknown control command=%u\r\n", (unsigned int)command);
        break;
    }
}

static void AppRuntime_HandleCanFrame(const CAN_Frame_t *frame)
{
    if (frame->is_remote != 0U) {
        printf("[CAN] remote frame ignored id=0x%03lX\r\n", (unsigned long)frame->id);
        return;
    }

    if (frame->id == APP_RUNTIME_CAN_CMD_STEER_STDID) {
        int16_t steer_x10 = (int16_t)(((uint16_t)frame->data[1] << 8) | frame->data[0]);
        float steering_deg = ((float)steer_x10) * 0.1f;
        int ret = 0;

#if APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD
        AppRuntime_KeyboardScenarioStop("CAN steering command");
        g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(steering_deg);
        steering_deg = g_keyboard_target_steer_deg;
#endif
        ret = PositionControl_SetTargetWithSource(AppRuntime_TargetSteeringDegToMotorDeg(steering_deg),
                                                  CMD_SRC_SERVICE);
        printf("[CAN] steer cmd=%.1f deg ret=%d\r\n", steering_deg, ret);
        return;
    }

    if (frame->id == APP_RUNTIME_CAN_CMD_CONTROL_STDID) {
        AppRuntime_HandleCanControlCommand(frame->data[0]);
        return;
    }

    if (frame->id == APP_RUNTIME_CAN_QUERY_STDID) {
        AppRuntime_CanSendStatusFrame();
        return;
    }
}

static void AppRuntime_ServiceCan(void)
{
    CAN_Frame_t frame = {0};

    CAN_Runtime_Service();

    while (CAN_Runtime_Pop(&frame) != 0U) {
        if (g_can_rx_log_enabled != 0U) {
            AppRuntime_CanPrintFrame("rx", &frame);
        }
        AppRuntime_HandleCanFrame(&frame);
    }
}

static void AppRuntime_SendCanTestFrame(void)
{
    const uint8_t payload[8] = {'C', 'A', 'N', 'T', 'E', 'S', 'T', 0U};
    int ret = CAN_Runtime_SendStd((uint16_t)APP_RUNTIME_CAN_TEST_TX_STDID, payload, 8U);

    printf("[CAN] test tx ret=%d id=0x%03X mode=%s\r\n",
           ret,
           APP_RUNTIME_CAN_TEST_TX_STDID,
           (APP_RUNTIME_CAN_MODE == APP_RUNTIME_CAN_MODE_LOOPBACK) ? "LOOPBACK" : "NORMAL");
}
#else
#define AppRuntime_ServiceCan() ((void)0)
#define AppRuntime_SendCanTestFrame() ((void)0)
#define AppRuntime_CanSendStatusFrame() ((void)0)
#endif

#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
/* Print the CSV schema once so bench logs remain self-describing. */
static void AppRuntime_PrintPeriodicCsvHeader(void)
{
    printf("CSV_HEADER,ms,mode,target_deg,current_deg,error_deg,output,dir,enc_cnt,enc_raw,req_hz,applied_hz,out_active,rev_guard,cmd_id,cmd_state,cmd_result\r\n");
}

/* Emit a throttled CSV telemetry row for offline log analysis. */
static void AppRuntime_ServicePeriodicCsv(void)
{
    static uint32_t last_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    PositionControl_State_t s = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
    int32_t enc_count = AppRuntime_GetDisplayEncoderCount();
    uint32_t enc_raw = AppRuntime_GetDisplayEncoderRaw();

    if (g_periodic_csv_enabled == 0U) {
        return;
    }

    if ((uint32_t)(now_ms - last_ms) < APP_RUNTIME_PERIODIC_CSV_LOG_PERIOD_MS) {
        return;
    }
    last_ms = now_ms;

    printf("CSV,%lu,%d,%.3f,%.3f,%.3f,%.0f,%d,%ld,%lu,%ld,%lu,%u,%u,%lu,%d,%d\r\n",
           (unsigned long)now_ms,
           (int)PositionControl_GetMode(),
           AppRuntime_TargetMotorDegToSteeringDeg(s.target_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(s.current_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(s.error),
           s.output,
           (int)dir_state,
           (long)enc_count,
           (unsigned long)enc_raw,
           (long)pulse_status.requested_frequency_hz,
           (unsigned long)pulse_status.applied_frequency_hz,
           (unsigned int)pulse_status.output_active,
           (unsigned int)pulse_status.reverse_guard_active,
           (unsigned long)cmd.command_id,
           (int)cmd.state,
           (int)cmd.result);
}
#else
#define AppRuntime_PrintPeriodicCsvHeader() ((void)0)
#define AppRuntime_ServicePeriodicCsv() ((void)0)
#endif

/* Emit the latency-profiler batch report once enough samples have been collected. */
static void AppRuntime_TryLatencyAutoReport(void)
{
#if LATENCY_AUTO_REPORT_ENABLE
    static uint32_t check_div = 0U;
    uint32_t i = 0U;
    uint32_t miss_count = 0U;
    LatencyStageStats_t stats = {0};

    if (++check_div < 10U) {
        return;
    }
    check_div = 0U;

    for (i = 0U; i < (uint32_t)LAT_STAGE_COUNT; i++) {
        if (LatencyProfiler_GetStageSampleCount((LatencyStage_t)i) < LATENCY_AUTO_REPORT_SAMPLES) {
            return;
        }
    }

    miss_count = LatencyProfiler_GetDeadlineMissCount();
    printf("LATENCY_BATCH_BEGIN,seq=%lu,samples=%u,core_hz=%lu,deadline_miss=%lu\r\n",
           (unsigned long)g_latency_report_seq,
           (unsigned int)LATENCY_AUTO_REPORT_SAMPLES,
           (unsigned long)SystemCoreClock,
           (unsigned long)miss_count);

    for (i = 0U; i < (uint32_t)LAT_STAGE_COUNT; i++) {
        if (LatencyProfiler_GetStageStats((LatencyStage_t)i, &stats)) {
            printf("LATENCY_STAGE,seq=%lu,name=%s,count=%lu,avg_cycles=%lu,p99_cycles=%lu,max_cycles=%lu,avg_us=%.3f,p99_us=%.3f,max_us=%.3f\r\n",
                   (unsigned long)g_latency_report_seq,
                   LatencyProfiler_StageName((LatencyStage_t)i),
                   (unsigned long)stats.sample_count,
                   (unsigned long)stats.avg_cycles,
                   (unsigned long)stats.p99_cycles,
                   (unsigned long)stats.max_cycles,
                   stats.avg_us,
                   stats.p99_us,
                   stats.max_us);
        }
    }

    printf("LATENCY_BATCH_END,seq=%lu\r\n", (unsigned long)g_latency_report_seq);

    g_latency_report_seq++;
    LatencyProfiler_Reset();
#endif
}

/* Print live TIM2 encoder register details when bench diagnostics are enabled. */
static void AppRuntime_ServiceEncoderRuntimeDiag(void)
{
#if APP_RUNTIME_ENCODER_DIAG_ENABLE
    static uint32_t last_ms = 0U;
    static uint32_t prev_cnt = 32768UL;
    uint32_t now_ms = HAL_GetTick();
    uint32_t cnt = 0U;
    uint32_t delta_u32 = 0U;
    int32_t delta = 0;
    uint32_t cr1 = 0U;
    uint32_t smcr = 0U;
    uint32_t ccmr1 = 0U;
    uint32_t ccer = 0U;
    uint32_t cen = 0U;
    uint32_t sms = 0U;
    uint32_t cc1s = 0U;
    uint32_t cc2s = 0U;
    uint32_t cc1e = 0U;
    uint32_t cc2e = 0U;
    GPIO_PinState enc_a_state = GPIO_PIN_RESET;
    GPIO_PinState enc_b_state = GPIO_PIN_RESET;

    if ((uint32_t)(now_ms - last_ms) < APP_RUNTIME_ENCODER_DIAG_PERIOD_MS) {
        return;
    }
    last_ms = now_ms;

    cnt = __HAL_TIM_GET_COUNTER(&htim2);
    delta_u32 = (uint32_t)(cnt - prev_cnt);
    delta = (int32_t)delta_u32;
    cr1 = htim2.Instance->CR1;
    smcr = htim2.Instance->SMCR;
    ccmr1 = htim2.Instance->CCMR1;
    ccer = htim2.Instance->CCER;
    cen = ((cr1 & TIM_CR1_CEN) != 0U) ? 1U : 0U;
    sms = (smcr & TIM_SMCR_SMS) >> TIM_SMCR_SMS_Pos;
    cc1s = (ccmr1 & TIM_CCMR1_CC1S) >> TIM_CCMR1_CC1S_Pos;
    cc2s = (ccmr1 & TIM_CCMR1_CC2S) >> TIM_CCMR1_CC2S_Pos;
    cc1e = ((ccer & TIM_CCER_CC1E) != 0U) ? 1U : 0U;
    cc2e = ((ccer & TIM_CCER_CC2E) != 0U) ? 1U : 0U;
    enc_a_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    enc_b_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);

    printf("[ENCDBG] ms=%lu cnt=%lu prev=%lu delta=%ld A=%d B=%d CEN=%lu SMS=%lu CC1S=%lu CC2S=%lu CC1E=%lu CC2E=%lu CR1=0x%04lX SMCR=0x%04lX CCMR1=0x%04lX CCER=0x%04lX\r\n",
           (unsigned long)now_ms,
           (unsigned long)cnt,
           (unsigned long)prev_cnt,
           (long)delta,
           (int)enc_a_state,
           (int)enc_b_state,
           (unsigned long)cen,
           (unsigned long)sms,
           (unsigned long)cc1s,
           (unsigned long)cc2s,
           (unsigned long)cc1e,
           (unsigned long)cc2e,
           (unsigned long)cr1,
           (unsigned long)smcr,
           (unsigned long)ccmr1,
           (unsigned long)ccer);

    prev_cnt = cnt;
#endif
}

#if APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD
/* Clamp keyboard-entered steering angles to the allowed steering envelope. */
static float AppRuntime_KeyboardClampSteeringDeg(float steering_deg)
{
    if (steering_deg > MAX_STEERING_ANGLE) {
        return MAX_STEERING_ANGLE;
    }
    if (steering_deg < MIN_STEERING_ANGLE) {
        return MIN_STEERING_ANGLE;
    }
    return steering_deg;
}

/* Clear the buffered keyboard command line. */
static void AppRuntime_KeyboardClearLine(void)
{
    g_keyboard_line_len = 0U;
    g_keyboard_line_buf[0] = '\0';
}

static void AppRuntime_KeyboardApplyTarget(void);

static uint8_t AppRuntime_KeyboardScenarioConfigValid(void)
{
#if APP_RUNTIME_KEYBOARD_SCENARIO_ENABLE
    if (APP_RUNTIME_KEYBOARD_SCENARIO_STEP_DEG <= 0.0f) {
        return 0U;
    }
    if (APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_DEG < APP_RUNTIME_KEYBOARD_SCENARIO_STEP_DEG) {
        return 0U;
    }
    if (APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_DEG > MAX_STEERING_ANGLE) {
        return 0U;
    }
    return 1U;
#else
    return 0U;
#endif
}

static void AppRuntime_KeyboardScenarioStop(const char *reason)
{
    if (g_keyboard_scenario.active == 0U) {
        return;
    }

    g_keyboard_scenario.active = 0U;
    g_keyboard_scenario.phase = KB_SCENARIO_IDLE;
    g_keyboard_scenario.next_step_ms = 0U;

    if (reason != NULL) {
        printf("[KB][scenario] stop: %s\r\n", reason);
    }
}

static void AppRuntime_KeyboardScenarioStart(void)
{
    if (AppRuntime_KeyboardScenarioConfigValid() == 0U) {
        printf("[KB][scenario] invalid config: step=%.1f peak=%.1f\r\n",
               APP_RUNTIME_KEYBOARD_SCENARIO_STEP_DEG,
               APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_DEG);
        return;
    }

    g_keyboard_scenario.active = 1U;
    g_keyboard_scenario.phase = KB_SCENARIO_RAMP_UP;
    g_keyboard_scenario.next_step_ms = HAL_GetTick() + APP_RUNTIME_KEYBOARD_SCENARIO_STEP_HOLD_MS;

    g_keyboard_target_steer_deg = 0.0f;
    printf("[KB][scenario] start right turn: 0 -> %.1f -> ... -> %.1f -> 0 deg, step_hold=%lu ms, peak_hold=%lu ms\r\n",
           APP_RUNTIME_KEYBOARD_SCENARIO_STEP_DEG,
           APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_DEG,
           (unsigned long)APP_RUNTIME_KEYBOARD_SCENARIO_STEP_HOLD_MS,
           (unsigned long)APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_HOLD_MS);
    AppRuntime_KeyboardApplyTarget();
}

static void AppRuntime_KeyboardScenarioService(void)
{
    uint32_t now_ms = 0U;
    float next_target_deg = 0.0f;

    if (g_keyboard_scenario.active == 0U) {
        return;
    }

    if (PositionControl_GetMode() == CTRL_MODE_EMERGENCY) {
        AppRuntime_KeyboardScenarioStop("controller in emergency");
        return;
    }

    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - g_keyboard_scenario.next_step_ms) < 0) {
        return;
    }

    switch (g_keyboard_scenario.phase) {
    case KB_SCENARIO_RAMP_UP:
        next_target_deg = g_keyboard_target_steer_deg + APP_RUNTIME_KEYBOARD_SCENARIO_STEP_DEG;
        if (next_target_deg >= APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_DEG) {
            g_keyboard_target_steer_deg = APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_DEG;
            AppRuntime_KeyboardApplyTarget();
            g_keyboard_scenario.phase = KB_SCENARIO_PEAK_HOLD;
            g_keyboard_scenario.next_step_ms = now_ms + APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_HOLD_MS;
        } else {
            g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(next_target_deg);
            AppRuntime_KeyboardApplyTarget();
            g_keyboard_scenario.next_step_ms = now_ms + APP_RUNTIME_KEYBOARD_SCENARIO_STEP_HOLD_MS;
        }
        break;

    case KB_SCENARIO_PEAK_HOLD:
        g_keyboard_scenario.phase = KB_SCENARIO_RAMP_DOWN;
        g_keyboard_scenario.next_step_ms = now_ms;
        break;

    case KB_SCENARIO_RAMP_DOWN:
        next_target_deg = g_keyboard_target_steer_deg - APP_RUNTIME_KEYBOARD_SCENARIO_STEP_DEG;
        if (next_target_deg <= 0.0f) {
            g_keyboard_target_steer_deg = 0.0f;
            AppRuntime_KeyboardApplyTarget();
            AppRuntime_KeyboardScenarioStop("completed");
        } else {
            g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(next_target_deg);
            AppRuntime_KeyboardApplyTarget();
            g_keyboard_scenario.next_step_ms = now_ms + APP_RUNTIME_KEYBOARD_SCENARIO_STEP_HOLD_MS;
        }
        break;

    case KB_SCENARIO_IDLE:
    default:
        AppRuntime_KeyboardScenarioStop("invalid state");
        break;
    }
}

/* Print a human-readable control snapshot for keyboard bench testing. */
static void AppRuntime_KeyboardPrintControlSnapshot(const char *reason)
{
    PositionControl_State_t s = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
    int32_t enc_count = AppRuntime_GetDisplayEncoderCount();
    uint32_t enc_raw = AppRuntime_GetDisplayEncoderRaw();

    printf("[KB][%s] T=%.2fdeg C=%.2fdeg E=%.2fdeg O=%.0f DIR=%d ENC=%ld RAW=%lu REQ=%ld AP=%lu RUN=%u REV=%u CMD=%lu/%s/%s\r\n",
           reason,
           AppRuntime_TargetMotorDegToSteeringDeg(s.target_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(s.current_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(s.error),
           s.output,
           (int)dir_state,
           (long)enc_count,
           (unsigned long)enc_raw,
           (long)pulse_status.requested_frequency_hz,
           (unsigned long)pulse_status.applied_frequency_hz,
           (unsigned int)pulse_status.output_active,
           (unsigned int)pulse_status.reverse_guard_active,
           (unsigned long)cmd.command_id,
           PositionControlDiag_CommandStateString(cmd.state),
           PositionControlDiag_CommandResultString(cmd.result));
}

/* Apply the current keyboard target to the position controller. */
static void AppRuntime_KeyboardApplyTarget(void)
{
    float motor_target_deg = SteeringDegToMotorDeg(g_keyboard_target_steer_deg);
    int ret = PositionControl_SetTargetWithSource(motor_target_deg, CMD_SRC_KEYBOARD);

    printf("[KB] target steer=%.1f deg motor=%.1f deg ret=%d\r\n",
           g_keyboard_target_steer_deg,
           motor_target_deg,
           ret);
    AppRuntime_KeyboardPrintControlSnapshot("target");
}

/* Print the interactive keyboard bench-test help text. */
static void AppRuntime_KeyboardPrintHelp(void)
{
    printf("[KB] A:left D:right S:center R:right-scenario C:cancel-scenario E:enable Q:disable X:estop P:print L:csv H:help step=%.1f deg\r\n",
           APP_RUNTIME_KEYBOARD_STEP_DEG);
    printf("[KB] numeric target: type steering deg then Enter. ex) 5, -3.5, 0\r\n");
#if APP_RUNTIME_CAN_ENABLE
    printf("[KB] N:can-status M:can-test-tx J:can-rx-log-toggle (steer=0x%03X ctrl=0x%03X query=0x%03X)\r\n",
           APP_RUNTIME_CAN_CMD_STEER_STDID,
           APP_RUNTIME_CAN_CMD_CONTROL_STDID,
           APP_RUNTIME_CAN_QUERY_STDID);
#endif
#if APP_RUNTIME_KEYBOARD_SCENARIO_ENABLE
    printf("[KB] right scenario: 0 -> %.1f -> ... -> %.1f -> 0 deg, hold=%lu ms, peak_hold=%lu ms\r\n",
           APP_RUNTIME_KEYBOARD_SCENARIO_STEP_DEG,
           APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_DEG,
           (unsigned long)APP_RUNTIME_KEYBOARD_SCENARIO_STEP_HOLD_MS,
           (unsigned long)APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_HOLD_MS);
#endif
}

/* Parse the buffered numeric steering target and apply it. */
static void AppRuntime_KeyboardApplyTypedTarget(void)
{
    char *end_ptr = NULL;
    float typed_target_deg = 0.0f;

    g_keyboard_line_buf[g_keyboard_line_len] = '\0';
    typed_target_deg = strtof(g_keyboard_line_buf, &end_ptr);

    if (end_ptr == g_keyboard_line_buf || *end_ptr != '\0') {
        printf("[KB] invalid target \"%s\"\r\n", g_keyboard_line_buf);
        AppRuntime_KeyboardClearLine();
        return;
    }

    g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(typed_target_deg);
    AppRuntime_KeyboardApplyTarget();
    AppRuntime_KeyboardClearLine();
}

/* Service the UART-driven keyboard bench-test interface. */
static void AppRuntime_KeyboardProcessInput(void)
{
    uint8_t ch = 0U;

    if (HAL_UART_Receive(&huart3, &ch, 1, 0U) != HAL_OK) {
        return;
    }

    switch (ch) {
    case '\r':
    case '\n':
        if (g_keyboard_line_len > 0U) {
            AppRuntime_KeyboardScenarioStop("manual numeric target");
            AppRuntime_KeyboardApplyTypedTarget();
        }
        break;

    case '\b':
    case 0x7FU:
        if (g_keyboard_line_len > 0U) {
            g_keyboard_line_len--;
            g_keyboard_line_buf[g_keyboard_line_len] = '\0';
        }
        break;

    case 'a':
    case 'A':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardScenarioStop("manual left step");
        g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(
            g_keyboard_target_steer_deg - APP_RUNTIME_KEYBOARD_STEP_DEG);
        AppRuntime_KeyboardApplyTarget();
        break;

    case 'd':
    case 'D':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardScenarioStop("manual right step");
        g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(
            g_keyboard_target_steer_deg + APP_RUNTIME_KEYBOARD_STEP_DEG);
        AppRuntime_KeyboardApplyTarget();
        break;

    case 's':
    case 'S':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardScenarioStop("manual center");
        g_keyboard_target_steer_deg = 0.0f;
        AppRuntime_KeyboardApplyTarget();
        break;

    case 'r':
    case 'R':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardScenarioStart();
        break;

    case 'c':
    case 'C':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardScenarioStop("operator cancel");
        break;

    case 'e':
    case 'E':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardScenarioStop("control enable");
        PositionControl_Enable();
        printf("[KB] control enabled\r\n");
        break;

    case 'q':
    case 'Q':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardScenarioStop("control disable");
        PositionControl_Disable();
        printf("[KB] control disabled\r\n");
        break;

    case 'x':
    case 'X':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardScenarioStop("emergency stop");
        PositionControl_EmergencyStop();
        printf("[KB] emergency stop\r\n");
        break;

    case 'p':
    case 'P':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardPrintControlSnapshot("snapshot");
        break;

    case 'l':
    case 'L':
        AppRuntime_KeyboardClearLine();
#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
        g_periodic_csv_enabled = (uint8_t)(g_periodic_csv_enabled == 0U ? 1U : 0U);
        printf("[KB] csv log %s\r\n", (g_periodic_csv_enabled != 0U) ? "enabled" : "disabled");
        if (g_periodic_csv_enabled != 0U) {
            AppRuntime_PrintPeriodicCsvHeader();
        }
#else
        printf("[KB] csv log feature disabled at build time\r\n");
#endif
        break;

    case 'j':
    case 'J':
        AppRuntime_KeyboardClearLine();
#if APP_RUNTIME_CAN_ENABLE
        g_can_rx_log_enabled = (uint8_t)(g_can_rx_log_enabled == 0U ? 1U : 0U);
        printf("[CAN] rx log %s\r\n", (g_can_rx_log_enabled != 0U) ? "enabled" : "disabled");
#else
        printf("[CAN] feature disabled at build time\r\n");
#endif
        break;

    case 'h':
    case 'H':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardPrintHelp();
        break;

    case 'm':
    case 'M':
        AppRuntime_KeyboardClearLine();
        AppRuntime_SendCanTestFrame();
        break;

    case 'n':
    case 'N':
        AppRuntime_KeyboardClearLine();
#if APP_RUNTIME_CAN_ENABLE
        CAN_Runtime_PrintStatus();
        AppRuntime_CanSendStatusFrame();
#else
        printf("[CAN] feature disabled at build time\r\n");
#endif
        break;

    default:
        if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.') {
            if (g_keyboard_line_len < (uint8_t)(sizeof(g_keyboard_line_buf) - 1U)) {
                g_keyboard_line_buf[g_keyboard_line_len++] = (char)ch;
                g_keyboard_line_buf[g_keyboard_line_len] = '\0';
            } else {
                printf("[KB] input too long\r\n");
                AppRuntime_KeyboardClearLine();
            }
        } else {
            printf("[KB] unknown key '%c' (0x%02X)\r\n", (char)ch, (unsigned int)ch);
            AppRuntime_KeyboardClearLine();
        }
        break;
    }
}
#endif

/* Configure the direction line-driver GPIO that CubeMX leaves to user code. */
static void AppRuntime_ConfigureDirectionPin(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = GPIO_PIN_10;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio_init);
}

#if APP_RUNTIME_INPUT_SOURCE_IS_UDP
/* Process UDP mode changes and map received packets into controller commands. */
static void AppRuntime_ServiceUdpComms(void)
{
    SteerMode_t mode = STEER_MODE_NONE;
    uint32_t now_ms = 0U;
    uint32_t last_rx_ms = 0U;

    LAT_BEGIN(LAT_STAGE_COMMS);
    MX_LWIP_Process();

    mode = EthComm_GetCurrentMode();
    now_ms = HAL_GetTick();
    last_rx_ms = EthComm_GetLastRxTick();

    if ((mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) &&
        ((now_ms - last_rx_ms) > ETHCOMM_RX_TIMEOUT_MS)) {
        EthComm_ForceMode(STEER_MODE_ESTOP);
        mode = STEER_MODE_ESTOP;
    }

    if (mode == STEER_MODE_ESTOP) {
        if (g_prev_mode != STEER_MODE_ESTOP) {
            PositionControl_EmergencyStop();
        }
    } else if (mode == STEER_MODE_NONE) {
        if (g_prev_mode != STEER_MODE_NONE) {
            PositionControl_Disable();
        }
    } else if ((mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) &&
               (g_prev_mode == STEER_MODE_NONE || g_prev_mode == STEER_MODE_ESTOP)) {
        Relay_EmergencyRelease();
        PositionControl_Enable();
    }

    if (EthComm_ConsumeEmergencyRequest()) {
        PositionControl_EmergencyStop();
        mode = STEER_MODE_ESTOP;
    }

    if (EthComm_HasNewData()) {
        AutoDrive_Packet_t pkt = EthComm_GetLatestData();
        if (mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) {
            PositionControl_SetTargetWithSource(AppRuntime_TargetSteeringDegToMotorDeg(pkt.steering_angle),
                                                CMD_SRC_UDP);
        }
    }

    LAT_END(LAT_STAGE_COMMS);

    g_current_mode = mode;
    g_prev_mode = mode;
}
#endif

/* Emit the slower human-readable diagnostic snapshot used during bring-up. */
static void AppRuntime_PrintPeriodicDiag(void)
{
    PositionControl_State_t s = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
    int32_t enc_count = AppRuntime_GetDisplayEncoderCount();
    uint32_t enc_raw = AppRuntime_GetDisplayEncoderRaw();
    float target_steer_deg = AppRuntime_TargetMotorDegToSteeringDeg(s.target_angle);
    float current_steer_deg = AppRuntime_TargetMotorDegToSteeringDeg(s.current_angle);
    float error_steer_deg = AppRuntime_TargetMotorDegToSteeringDeg(s.error);

#if LATENCY_LOG_ENABLE
    printf("[DIAG] MODE:%d CMD:%lu/%s/%s Tst:%.2f Cst:%.2f Est:%.2f O:%.0f REQ:%ld AP:%lu ARR:%lu CCR:%lu DIR:%d RUN:%u REV:%u ENC:%ld RAW:%lu\r\n",
           (int)PositionControl_GetMode(),
           (unsigned long)cmd.command_id,
           PositionControlDiag_CommandStateString(cmd.state),
           PositionControlDiag_CommandResultString(cmd.result),
           target_steer_deg,
           current_steer_deg,
           error_steer_deg,
           s.output,
           (long)pulse_status.requested_frequency_hz,
           (unsigned long)pulse_status.applied_frequency_hz,
           (unsigned long)pulse_status.autoreload,
           (unsigned long)pulse_status.compare,
           (int)dir_state,
           (unsigned int)pulse_status.output_active,
           (unsigned int)pulse_status.reverse_guard_active,
           (long)enc_count,
           (unsigned long)enc_raw);
#else
    (void)pulse_status;
    (void)enc_count;
    (void)enc_raw;
    (void)dir_state;
    (void)s;
    (void)cmd;
    (void)target_steer_deg;
    (void)current_steer_deg;
    (void)error_steer_deg;
#endif
}

/* Service the 1 ms application path that runs from the timer interrupt flag. */
static void AppRuntime_ServiceFastTick(void)
{
#if APP_RUNTIME_AUTO_FIXED_PULSE_TEST
#if APP_RUNTIME_INPUT_SOURCE_IS_UDP
    if (g_current_mode == STEER_MODE_AUTO) {
        PulseControl_SetFrequency(APP_RUNTIME_AUTO_FIXED_PULSE_HZ);
    } else {
        PulseControl_Stop();
    }
#else
    PulseControl_Stop();
#endif
#else
    PositionControl_Update();
#endif

    AppRuntime_UpdateVirtualEncoder();

    AppRuntime_ServiceEncoderRuntimeDiag();

    if (++g_debug_print_divider < APP_RUNTIME_PERIODIC_DIAG_DIVIDER) {
        return;
    }

    g_debug_print_divider = 0U;
    AppRuntime_PrintPeriodicDiag();
}

/* Initialize the application-specific runtime after CubeMX peripherals are ready. */
void AppRuntime_Init(void)
{
    LatencyProfiler_Init(SystemCoreClock);
    AppRuntime_ConfigureDirectionPin();

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    Relay_Init();
    PulseControl_Init();
    EncoderReader_Init();
#if APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE
    EncoderReader_EnableVirtualFeedback(1U);
#else
    EncoderReader_EnableVirtualFeedback(0U);
#endif
    PositionControl_Init();

#if APP_RUNTIME_CAN_ENABLE
#if APP_RUNTIME_CAN_AUTO_START
    {
        int can_ret = CAN_Runtime_Start();
        printf("[CAN] start ret=%d mode=%s bitrate=500000 rx_log=%s\r\n",
               can_ret,
               (APP_RUNTIME_CAN_MODE == APP_RUNTIME_CAN_MODE_LOOPBACK) ? "LOOPBACK" : "NORMAL",
               (g_can_rx_log_enabled != 0U) ? "ON" : "OFF");
    }
#else
    printf("[CAN] init complete. auto start is OFF.\r\n");
#endif
#endif

    Relay_ServoOn();
    HAL_Delay(500);

    {
        char msg[] = "Servo Start!\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
    }

#if APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE
    printf("[VENC] Putty ENC/RAW uses pulse-integrated virtual encoder display.\r\n");
#endif

#if APP_RUNTIME_RESET_ENCODER_ON_BOOT
    EncoderReader_Reset();
#endif
    AppRuntime_ResetVirtualEncoder();
    PositionControl_SetTargetWithSource(AppRuntime_TargetSteeringDegToMotorDeg(0.0f), CMD_SRC_LOCALTEST);
#if APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD
    g_keyboard_target_steer_deg = 0.0f;
    g_keyboard_scenario.active = 0U;
    g_keyboard_scenario.phase = KB_SCENARIO_IDLE;
    g_keyboard_scenario.next_step_ms = 0U;
#else
    g_prev_mode = STEER_MODE_NONE;
    g_current_mode = STEER_MODE_NONE;
#endif
    g_debug_print_divider = 0U;
#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
    g_periodic_csv_enabled = 1U;
#endif
    AppRuntime_PrintInputSourceSummary();
#if APP_RUNTIME_AUTO_START_CONTROL_ENABLE
    PositionControl_Enable();
#endif

#if APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD
    AppRuntime_KeyboardPrintHelp();
    AppRuntime_PrintPeriodicCsvHeader();
#else
    EthComm_UDP_Init();
#endif
}

/* Run one application super-loop iteration on top of the CubeMX main loop. */
void AppRuntime_RunIteration(void)
{
#if APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD
    LAT_BEGIN(LAT_STAGE_COMMS);
    AppRuntime_KeyboardProcessInput();
    AppRuntime_KeyboardScenarioService();
    AppRuntime_ServiceCan();
    LAT_END(LAT_STAGE_COMMS);
#else
    AppRuntime_ServiceUdpComms();
    AppRuntime_ServiceCan();
#endif

    if (interrupt_flag != 0U) {
        interrupt_flag = 0U;
        AppRuntime_ServiceFastTick();
    }

    AppRuntime_ServicePeriodicCsv();
    AppRuntime_TryLatencyAutoReport();
    HAL_IWDG_Refresh(&hiwdg);
}
