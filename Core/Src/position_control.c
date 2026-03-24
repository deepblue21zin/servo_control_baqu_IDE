#include "position_control.h"
#include "position_control_diag.h"
#include "encoder_reader.h"      // 엔코더 읽기
#include "pulse_control.h"       // 펄스 출력
#include "relay_control.h"       // 릴레이 제어 (EmergencyStop에서 사용)
#include "constants.h"
#include <stdio.h>
#include <math.h>                // fabsf
#include "main.h"
#include "latency_profiler.h"

//의존성

// 내부 변수 지정
static volatile uint8_t fault_flag = 0; // 안전 관련 플래그
static uint32_t command_next_id = 1U;
static CommandSource_t pending_command_source = CMD_SRC_NONE;

#define POSITION_COMMAND_TIMEOUT_MS 3000U
#define POSITION_COMMAND_TIMEOUT_FAILSAFE_ENABLE 0U

static PID_Params_t pid_params = {
    .Kp = 50.0f,           // 초기값 (실험으로 튜닝 필요)
    .Ki = 5.0f,
    .Kd = 20.0f,
    .integral_limit = 1000.0f,
    .output_limit = 10000.0f   // 최대 펄스 주파수 (Hz)
};

static struct {
    float prev_error;
    float integral;
    uint32_t last_time_ms;
} pid_state = {0};

static PositionControl_State_t state = {
    .target_angle = 0.0f,
    .current_angle = 0.0f,
    .error = 0.0f,
    .output = 0.0f,
    .is_stable = false,
    .stable_time_ms = 0
};

static volatile bool control_enabled = false;
static volatile ControlMode_t control_mode = CTRL_MODE_IDLE;
static CommandLifecycle_t command_lifecycle = {
    .command_id = 0U,
    .state = CMD_IDLE,
    .source = CMD_SRC_NONE,
    .result = CMD_RESULT_NONE,
    .timeout_ms = POSITION_COMMAND_TIMEOUT_MS
};

int PositionControl_SetTargetWithSource(float target_deg, CommandSource_t source);

#ifdef DBG_LOOP_Pin
#define DBG_LOOP_SET() HAL_GPIO_WritePin(DBG_LOOP_GPIO_Port, DBG_LOOP_Pin, GPIO_PIN_SET)
#define DBG_LOOP_RESET() HAL_GPIO_WritePin(DBG_LOOP_GPIO_Port, DBG_LOOP_Pin, GPIO_PIN_RESET)
#else
#define DBG_LOOP_SET() ((void)0)
#define DBG_LOOP_RESET() ((void)0)
#endif

/* Mirror the latest controller snapshot into the shared debug variables. */
static void PositionControl_SyncDiagState(void)
{
    PositionControlDiag_UpdateDebugVars(&state, control_enabled, control_mode, fault_flag);
}

/* Check whether the controller is allowed to start a new command lifecycle. */
static bool PositionControl_CommandReadyForStart(void)
{
    if (!control_enabled) {
        return false;
    }
    if (control_mode == CTRL_MODE_EMERGENCY) {
        return false;
    }
    if (EncoderReader_IsInitialized() == 0U) {
        return false;
    }
    return true;
}

/* Start a fresh command lifecycle entry using the current target and position. */
static void PositionControl_CommandStart(CommandSource_t source)
{
    uint32_t now_ms = HAL_GetTick();

    command_lifecycle.command_id = command_next_id++;
    command_lifecycle.state = CMD_ACTIVE;
    command_lifecycle.source = source;
    command_lifecycle.result = CMD_RESULT_NONE;
    command_lifecycle.target_steering_deg = MotorDegToSteeringDeg(state.target_angle);
    command_lifecycle.target_motor_deg = state.target_angle;
    command_lifecycle.start_steering_deg = MotorDegToSteeringDeg(state.current_angle);
    command_lifecycle.final_steering_deg = command_lifecycle.start_steering_deg;
    command_lifecycle.final_error_deg = MotorDegToSteeringDeg(state.error);
    command_lifecycle.start_ms = now_ms;
    command_lifecycle.end_ms = 0U;
    command_lifecycle.timeout_ms = POSITION_COMMAND_TIMEOUT_MS;
    pending_command_source = CMD_SRC_NONE;

    printf("CMD_START,id=%lu,src=%s,target_deg=%.3f,target_motor_deg=%.3f,start_ms=%lu,start_deg=%.3f,start_error_deg=%.3f\r\n",
           (unsigned long)command_lifecycle.command_id,
           PositionControlDiag_CommandSourceString(command_lifecycle.source),
           command_lifecycle.target_steering_deg,
           command_lifecycle.target_motor_deg,
           (unsigned long)command_lifecycle.start_ms,
           command_lifecycle.start_steering_deg,
           command_lifecycle.final_error_deg);
}

/* Close the active command lifecycle entry with the final motion snapshot. */
static void PositionControl_CommandFinish(CommandState_t end_state, CommandResult_t result, uint32_t now_ms)
{
    if (command_lifecycle.state != CMD_ACTIVE) {
        return;
    }

    pid_state.integral = 0.0f;
    command_lifecycle.state = end_state;
    command_lifecycle.result = result;
    command_lifecycle.end_ms = now_ms;
    command_lifecycle.final_steering_deg = MotorDegToSteeringDeg(state.current_angle);
    command_lifecycle.final_error_deg = MotorDegToSteeringDeg(state.error);

    switch (end_state) {
    case CMD_REACHED:
        printf("CMD_REACHED,id=%lu,end_ms=%lu,settling_ms=%lu,final_deg=%.3f,final_error_deg=%.3f\r\n",
               (unsigned long)command_lifecycle.command_id,
               (unsigned long)command_lifecycle.end_ms,
               (unsigned long)(command_lifecycle.end_ms - command_lifecycle.start_ms),
               command_lifecycle.final_steering_deg,
               command_lifecycle.final_error_deg);
        break;

    case CMD_TIMEOUT:
        printf("CMD_TIMEOUT,id=%lu,end_ms=%lu,elapsed_ms=%lu,error_deg=%.3f\r\n",
               (unsigned long)command_lifecycle.command_id,
               (unsigned long)command_lifecycle.end_ms,
               (unsigned long)(command_lifecycle.end_ms - command_lifecycle.start_ms),
               command_lifecycle.final_error_deg);
        break;

    case CMD_ABORTED:
        printf("CMD_ABORT,id=%lu,reason=%s,end_ms=%lu,error_deg=%.3f\r\n",
               (unsigned long)command_lifecycle.command_id,
               PositionControlDiag_CommandResultString(result),
               (unsigned long)command_lifecycle.end_ms,
               command_lifecycle.final_error_deg);
        break;

    case CMD_FAULTED:
        printf("CMD_FAULT,id=%lu,reason=%s,end_ms=%lu,error_deg=%.3f\r\n",
               (unsigned long)command_lifecycle.command_id,
               PositionControlDiag_CommandResultString(result),
               (unsigned long)command_lifecycle.end_ms,
               command_lifecycle.final_error_deg);
        break;

    case CMD_IDLE:
    case CMD_ACTIVE:
    default:
        break;
    }
}

// ========== 초기화 ==========
// Init()함수를 호출 안하는 프로그램의 경우 선언시 초기화만 해도 된다.
//런타임에 재초기화가 필요한 경우

/* Reset controller state, lifecycle state, and diagnostic mirrors. */
int PositionControl_Init(void) {
    pid_state.prev_error = 0.0f;  //D항 계산: (현재오차 - 이전오차)/dt, 초기화 안하면 폭주 가능
    pid_state.integral = 0.0f;   //적분 누적값 초기화
    pid_state.last_time_ms = HAL_GetTick();  //HAL_GetTick():STM32HAL함수, 시스템 시작후 경과 시간 반환(dt계산을 위한 기준점 설정)
    
    state.target_angle = 0.0f;
    state.current_angle = 0.0f;
    state.error = 0.0f;
    state.output = 0.0f;
    state.is_stable = false;
    state.stable_time_ms = 0U;
    control_enabled = false; //제어 비활성화 상태로 시작, 안전장치, Enable() 함수를 명시적으로 호출해야 제어 시작
    control_mode = CTRL_MODE_IDLE;
    state.mode = CTRL_MODE_IDLE;
    state.last_error = POS_CTRL_OK;
    fault_flag = 0U;
    command_next_id = 1U;
    pending_command_source = CMD_SRC_NONE;
    command_lifecycle.command_id = 0U;
    command_lifecycle.state = CMD_IDLE;
    command_lifecycle.source = CMD_SRC_NONE;
    command_lifecycle.result = CMD_RESULT_NONE;
    command_lifecycle.target_steering_deg = 0.0f;
    command_lifecycle.target_motor_deg = 0.0f;
    command_lifecycle.start_steering_deg = 0.0f;
    command_lifecycle.final_steering_deg = 0.0f;
    command_lifecycle.final_error_deg = 0.0f;
    command_lifecycle.start_ms = 0U;
    command_lifecycle.end_ms = 0U;
    command_lifecycle.timeout_ms = POSITION_COMMAND_TIMEOUT_MS;

    PositionControl_SyncDiagState();

#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] Initialized\n");
#endif
    return POS_CTRL_OK;  // 성공
}

// ========== PID 계산 ==========
/* Convert the current tracking error into a signed pulse frequency command. */
static float PID_Calculate(float error, float dt) {
    // P항
    float p_term = pid_params.Kp * error;
    
    // I항 (적분 와인드업 방지)
    pid_state.integral += error * dt;
    
    // 적분 제한(최대치 제한)
    if (pid_state.integral > pid_params.integral_limit) {
        pid_state.integral = pid_params.integral_limit;
    } else if (pid_state.integral < -pid_params.integral_limit) {
        pid_state.integral = -pid_params.integral_limit;
    }
    
    float i_term = pid_params.Ki * pid_state.integral;
    
    // D항 (미분)
    float derivative = (error - pid_state.prev_error) / dt;
    float d_term = pid_params.Kd * derivative;
    
    pid_state.prev_error = error;
    
    // 출력 계산
    float output = p_term + i_term + d_term;
    
    // 출력 제한
    if (output > pid_params.output_limit) {
        output = pid_params.output_limit;
    } else if (output < -pid_params.output_limit) {
        output = -pid_params.output_limit;
    }
    
    return output;
}

// ========== 메인 제어 루프 (1ms마다 호출!) ==========
/* Run one closed-loop control iteration using the latest encoder feedback. */
void PositionControl_Update(void) {
    DBG_LOOP_SET();

    if (!control_enabled) {
        state.current_angle = EncoderReader_GetAngleDeg();
        state.error = state.target_angle - state.current_angle;
        state.output = 0.0f;
        PulseControl_Stop();
        PositionControl_SyncDiagState();
        DBG_LOOP_RESET();
        return;
    }

    LAT_BEGIN(LAT_STAGE_SENSE);
    // 1. 현재 각도 읽기
    state.current_angle = EncoderReader_GetAngleDeg();

    // 2. 오차 계산 (안전 체크보다 먼저!)
    // [BUG FIX] 기존 코드: 안전 체크가 오차 계산보다 앞에 있어서
    // CheckSafety()가 이전 루프의 state.error를 참조하는 1-step 지연 버그 존재.
    // 수정: 오차를 먼저 계산하고 안전 체크 수행.
    state.error = state.target_angle - state.current_angle;
    LAT_END(LAT_STAGE_SENSE); // 센서 읽기 시간 측정 종료

    // 3. 안전 체크 (현재 오차 기준으로 판단)
    LAT_BEGIN(LAT_STAGE_CONTROL);
    if (!PositionControl_CheckSafety()) {
        CommandResult_t fault_result = CMD_RESULT_NONE;
        // 어떤 에러(장애)가 발생했는지 확인합니다.
        if (fault_flag == 1U) {
            fault_result = CMD_RESULT_FAULT_LIMIT; // 예: 각도 범위 초과
        } else if (fault_flag == 2U) {
            fault_result = CMD_RESULT_FAULT_TRACKING; // 예: 추적 실패 (오차 지속)
        }

        state.output = 0.0f; // 즉시 모터 출력을 0으로 만듭니다.
        if (command_lifecycle.state == CMD_ACTIVE) {
            // 현재 명령을 에러 상태(CMD_FAULTED)로 강제 종료시킵니다.
            PositionControl_CommandFinish(CMD_FAULTED, fault_result, HAL_GetTick());
        }
        PositionControl_SyncDiagState();
        LAT_END(LAT_STAGE_CONTROL); // 안전 체크 시간 측정 종료
        PositionControl_EmergencyStop(); // 안전 정지 수행 (릴레이 OFF 등)
        DBG_LOOP_RESET(); // 제어 루프 종료 (안전 위반 시 더 이상의 동작 방지)
        return; // 제어 루프 종료 (안전 위반 시 더 이상의 동작 방지)
    }
    
    // 4. 시간 계산
    uint32_t current_time = HAL_GetTick(); // 현재 시간(ms)
    float dt = (current_time - pid_state.last_time_ms) / 1000.0f;  // ms → s
    //dt가 0 이되면 0으로 나누는게 되버림(D항 계산에서 폭주 가능), dt가 너무 크면 제어 성능 저하, 0.001s(1ms)보다 너무 크면 제어 성능 저하, 0.001s보다 너무 작으면 D항 계산에서 노이즈 영향 커짐
    if (dt <= 0.0f) {
        dt = 0.001f;  // 최소 dt 보장 (1ms)
        
    }else if (dt > 0.1f) {
        dt = 0.1f;    // 최대 dt 제한 (100ms)
    }

    pid_state.last_time_ms = current_time;
    // [BUG FIX] 기존 코드: PositionControl_CommandFinish()가 안전 체크보다 뒤에 있어서
    // 명령 타임아웃이 발생해도 안전 체크가 이전 오차로 수행되는 1-step 지연 버그 존재.
    // 수정: 타임아웃 체크를 안전 체크보다 먼저 수행하도록 순서 변경.
    if (POSITION_COMMAND_TIMEOUT_FAILSAFE_ENABLE != 0U &&
        command_lifecycle.state == CMD_ACTIVE) {// 명령이 활성 상태인 경우에만 타임아웃 체크 수행
        uint32_t elapsed_ms = current_time - command_lifecycle.start_ms; // 명령 시작 후 경과 시간 계산
        if (elapsed_ms > command_lifecycle.timeout_ms) { // 타임아웃 발생
            fault_flag = 3U;
            state.last_error = POS_CTRL_ERR_TIMEOUT;
            state.output = 0.0f;
            PulseControl_Stop();
            control_enabled = false;
            control_mode = CTRL_MODE_EMERGENCY;
            state.mode = CTRL_MODE_EMERGENCY;
            pid_state.integral = 0.0f;
            Relay_Emergency();
            PositionControl_CommandFinish(CMD_TIMEOUT, CMD_RESULT_TIMEOUT, current_time);
            PositionControl_SyncDiagState();
            LAT_END(LAT_STAGE_CONTROL);
            DBG_LOOP_RESET();
            return;
        }
    }

    if (command_lifecycle.state == CMD_REACHED) {// 명령이 이미 완료된 경우, 추가 제어 없이 안정 유지
        state.output = 0.0f;
        PulseControl_Stop();
        PositionControl_SyncDiagState();
        LAT_END(LAT_STAGE_CONTROL);
        DBG_LOOP_RESET();
        return;
    }
    
    // 5. PID 계산
    state.output = PID_Calculate(state.error, dt);
    LAT_END(LAT_STAGE_CONTROL);
    
    // 6. 펄스 출력
    LAT_BEGIN(LAT_STAGE_ACTUATE);
    PulseControl_SetFrequency((int32_t)state.output);
    LAT_END(LAT_STAGE_ACTUATE);
    
    // 7. 안정화 판단
    if (fabsf(state.error) < POSITION_TOLERANCE) {
        state.stable_time_ms += (uint32_t)(dt * 1000.0f); // 안정 유지 시간 누적
        if (state.stable_time_ms > 100) {  // 100ms 이상 안정
            state.is_stable = true;
            if (command_lifecycle.state == CMD_ACTIVE) {
                PositionControl_CommandFinish(CMD_REACHED, CMD_RESULT_REACHED, current_time);
                state.output = 0.0f;
                PulseControl_Stop();
            }
        }
    } else {
        state.stable_time_ms = 0;
        state.is_stable = false;
    }

    PositionControl_SyncDiagState();
    DBG_LOOP_RESET();
}

// ========== 목표 설정 ==========
/* Preserve the legacy API by treating a plain target set as an anonymous source. */
int PositionControl_SetTarget(float target_deg) {
    return PositionControl_SetTargetWithSource(target_deg, CMD_SRC_NONE);
}

// CommandSource_t을 명시적으로 받아서 명령 출처를 기록하는 새로운 API. UDP, 키보드, 서비스 등 다양한 출처 구분 가능. 내부적으로는 기존 SetTarget과 동일한 동작을 하며, 명령 라이프사이클 관리와 진단에 활용됨.
int PositionControl_SetTargetWithSource(float target_deg, CommandSource_t source) {
    // 범위 체크
    if (target_deg > MAX_ANGLE_DEG || target_deg < MIN_ANGLE_DEG) {
        return POS_CTRL_ERR_OVER_LIMIT; // 범위 초과 에러 반환
    }

    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_REPLACED, HAL_GetTick()); // 기존 명령이 활성 상태면 교체로 종료 처리
    }
    // 명령 라이프사이클 시작 준비: 인터럽트 보호 구역에서 상태 업데이트
    __disable_irq(); // 인터럽트 보호 시작 (명령 라이프사이클과 상태 업데이트의 원자성 보장)
    state.target_angle = target_deg;
    state.is_stable = false;
    state.stable_time_ms = 0;
    pending_command_source = source;
    __enable_irq(); // 인터럽트 보호 종료

    state.current_angle = EncoderReader_GetAngleDeg();
    state.error = state.target_angle - state.current_angle;

    if (PositionControl_CommandReadyForStart()) {
        PositionControl_CommandStart(source);
    }

    PositionControl_SyncDiagState();

    return POS_CTRL_OK;
}

// ========== 상태 읽기 ==========
/* Return the latest target angle snapshot. */
float PositionControl_GetTarget(void) {
    return state.target_angle;
}
/* Return the latest closed-loop tracking error. */
float PositionControl_GetError(void) {
    return state.error;
}
/* Copy the active PID gains for tuning or diagnostics. */
void PositionControl_GetPID(PID_Params_t* params) {
    if (params != NULL) {
        *params = pid_params;
    }
}
/* Return the current controller state snapshot. */
PositionControl_State_t PositionControl_GetState(void) {
    return state;
}

/* Return the latest command lifecycle snapshot. */
CommandLifecycle_t PositionControl_GetCommandLifecycle(void) {
    return command_lifecycle;
}

/* Return the current motor angle estimate in motor degrees. */
float PositionControl_GetCurrentAngle(void) {
    return state.current_angle;
}

/* Report whether the current tracking error has stayed within tolerance. */
bool PositionControl_IsStable(void) {
    return state.is_stable;
}

// ========== PID 게인 설정 ==========
/* Update the runtime PID gains and clear the stored integral term. */
void PositionControl_SetPID(float Kp, float Ki, float Kd) {
    pid_params.Kp = Kp;
    pid_params.Ki = Ki;
    pid_params.Kd = Kd;
    
    // 적분 리셋
    pid_state.integral = 0.0f;
    
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] PID updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
           Kp, Ki, Kd);
#endif
}

// ========== 제어 모드 ==========
/* Override the externally visible control mode. */
void PositionControl_SetMode(ControlMode_t mode) {
    control_mode = mode;
    state.mode = mode;
}
/* Return the currently latched control mode. */
ControlMode_t PositionControl_GetMode(void) {
    return control_mode;
}

/* Arm closed-loop position control and resume relay output if safe. */
int PositionControl_Enable(void) {
    control_enabled = true;
    fault_flag = 0;              // EmergencyStop 후 재활성화 시 fault 초기화
    control_mode = CTRL_MODE_POSITION;
    state.mode = CTRL_MODE_POSITION;
    state.last_error = POS_CTRL_OK;
    Relay_EmergencyRelease();

    // [BUG FIX] D항 킥 방지 (Derivative Kick Prevention)
    // 기존: prev_error=0으로 초기화 → 첫 호출 시 D=(error-0)/0.001=10000 → d_term=200000 → MAX cap
    // 수정: Enable 시점의 현재 오차로 prev_error 초기화 → 첫 D항 = (error-error)/dt = 0
    state.current_angle = EncoderReader_GetAngleDeg();
    pid_state.prev_error = state.target_angle - state.current_angle;
    pid_state.integral = 0.0f;
    pid_state.last_time_ms = HAL_GetTick();
    state.error = state.target_angle - state.current_angle;

    if (command_lifecycle.state != CMD_ACTIVE && fabsf(state.error) > POSITION_TOLERANCE) {
        PositionControl_CommandStart(
            (pending_command_source != CMD_SRC_NONE) ? pending_command_source : CMD_SRC_LOCALTEST
        );
    }
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] Enabled (FLT cleared, angle=%.2f)\r\n", state.current_angle);
#endif
    PositionControl_SyncDiagState();
    return POS_CTRL_OK;
}

/* Stop pulse output and leave the controller in idle mode. */
void PositionControl_Disable(void) {
    if (!control_enabled) {
        return;
    }
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_DISABLED, HAL_GetTick());
    }
    control_enabled = false;
    control_mode = CTRL_MODE_IDLE;
    state.mode = CTRL_MODE_IDLE;
    state.output = 0.0f;
    PulseControl_Stop();
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] Disabled\n");
#endif
    PositionControl_SyncDiagState();
}

/* Clear the target and PID accumulator without reinitializing the whole module. */
void PositionControl_Reset(void) {
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_DISABLED, HAL_GetTick());
    }
    pid_state.integral = 0.0f;
    pid_state.prev_error = 0.0f;
    state.target_angle = 0.0f;
    state.output = 0.0f;
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] Reset\n");
#endif
    PositionControl_SyncDiagState();
}

// ========== 안전 기능 ==========

/* Keep the safety-limit API stubbed until external limit tuning is introduced. */
void PositionControl_SetSafetyLimits(SafetyLimits_t* limits) {
    // 현재는 미구현
    (void)limits;
}

/* Reject motion when the measured position or tracking error exceeds safe bounds. */
bool PositionControl_CheckSafety(void) {
    // 각도 범위 체크 (하드웨어 물리 한계)
    if (state.current_angle > MAX_ANGLE_DEG + 5.0f ||
        state.current_angle < MIN_ANGLE_DEG - 5.0f) {
        fault_flag = 1;
        state.last_error = POS_CTRL_ERR_OVER_LIMIT;
        return false;
    }

    // 다회전 운용(기어비 적용) 기준 추종 오차 한계
    // 참고: 이 체크는 state.error 계산 후에 호출해야 의미 있음
    if (fabsf(state.error) > MAX_TRACKING_ERROR_DEG) {
        fault_flag = 2;
        state.last_error = POS_CTRL_ERR_SAFETY;
        return false;
    }

    state.last_error = POS_CTRL_OK;
    return true;
}
/* Reuse the same safety gate for quick status polling. */
bool PositionControl_IsSafe(void) {
    return PositionControl_CheckSafety();
}

/* Force the controller into emergency mode and drop pulse output immediately. */
void PositionControl_EmergencyStop(void) {
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_ESTOP, HAL_GetTick());
    }
    control_enabled = false;
    control_mode = CTRL_MODE_EMERGENCY;
    state.mode = CTRL_MODE_EMERGENCY;
    state.output = 0.0f;
    PulseControl_Stop();
    pid_state.integral = 0.0f;
    Relay_Emergency();
    // [수정] fault 상세 원인 출력 (fault_flag: 1=각도범위초과, 2=오차초과)
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] EMERGENCY STOP! FLT=%d Ang:%.1f Err:%.1f\r\n",
           (int)fault_flag, state.current_angle, state.error);
#endif
    PositionControl_SyncDiagState();
    // 현재는 소프트 정지 + EMG 릴레이 정지를 함께 수행.
    // 재활성화 시에는 상위 모드 전이에서 Relay_EmergencyRelease() 이후
    // PositionControl_Enable()이 호출되어 제어를 재개한다.
}

/* Abort the active command lifecycle without changing the current control mode. */
void PositionControl_AbortCommand(CommandResult_t reason)
{
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, reason, HAL_GetTick());
    }
}
/* Print a compact bench-friendly controller snapshot. */
void PositionControl_PrintStatus(void) {
    PositionControlDiag_PrintStateSummary(&state,
                                          control_enabled,
                                          fault_flag,
                                          &command_lifecycle);
}
