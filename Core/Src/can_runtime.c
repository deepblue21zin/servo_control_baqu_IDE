/**
  ******************************************************************************
  * @file    can_runtime.c
  * @brief   Runtime CAN helper layered on top of CubeMX-generated CAN1 init.
  *
  * The generated `Core/Src/can.c` owns the peripheral handle and MSP hooks.
  * This file owns runtime tuning, filter setup, polling-based RX, and
  * application-facing send/status helpers so that future code generation does
  * not overwrite the CAN command path.
  ******************************************************************************
  */

#include "can_runtime.h"

#include "can.h"
#include "project_params.h"

#include <stdio.h>
#include <string.h>

#define CAN_RUNTIME_RX_QUEUE_LEN 16U

typedef struct {
    volatile uint8_t head;
    volatile uint8_t tail;
    CAN_Frame_t frames[CAN_RUNTIME_RX_QUEUE_LEN];
} CAN_RuntimeRxQueue_t;

static CAN_RuntimeStatus_t g_can_status = {0};
static CAN_RuntimeRxQueue_t g_can_rx_queue = {0};

static uint32_t CAN_Runtime_MapSjw(uint32_t tq)
{
    switch (tq) {
    case 1U:
        return CAN_SJW_1TQ;
    case 2U:
        return CAN_SJW_2TQ;
    case 3U:
        return CAN_SJW_3TQ;
    case 4U:
        return CAN_SJW_4TQ;
    default:
        return CAN_SJW_1TQ;
    }
}

static uint32_t CAN_Runtime_MapBs1(uint32_t tq)
{
    switch (tq) {
    case 1U:
        return CAN_BS1_1TQ;
    case 2U:
        return CAN_BS1_2TQ;
    case 3U:
        return CAN_BS1_3TQ;
    case 4U:
        return CAN_BS1_4TQ;
    case 5U:
        return CAN_BS1_5TQ;
    case 6U:
        return CAN_BS1_6TQ;
    case 7U:
        return CAN_BS1_7TQ;
    case 8U:
        return CAN_BS1_8TQ;
    case 9U:
        return CAN_BS1_9TQ;
    case 10U:
        return CAN_BS1_10TQ;
    case 11U:
        return CAN_BS1_11TQ;
    case 12U:
        return CAN_BS1_12TQ;
    case 13U:
        return CAN_BS1_13TQ;
    case 14U:
        return CAN_BS1_14TQ;
    case 15U:
        return CAN_BS1_15TQ;
    case 16U:
        return CAN_BS1_16TQ;
    default:
        return CAN_BS1_15TQ;
    }
}

static uint32_t CAN_Runtime_MapBs2(uint32_t tq)
{
    switch (tq) {
    case 1U:
        return CAN_BS2_1TQ;
    case 2U:
        return CAN_BS2_2TQ;
    case 3U:
        return CAN_BS2_3TQ;
    case 4U:
        return CAN_BS2_4TQ;
    case 5U:
        return CAN_BS2_5TQ;
    case 6U:
        return CAN_BS2_6TQ;
    case 7U:
        return CAN_BS2_7TQ;
    case 8U:
        return CAN_BS2_8TQ;
    default:
        return CAN_BS2_2TQ;
    }
}

static void CAN_Runtime_ApplyInitConfig(void)
{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = APP_RUNTIME_CAN_PRESCALER;
    hcan1.Init.Mode = (APP_RUNTIME_CAN_MODE == APP_RUNTIME_CAN_MODE_LOOPBACK) ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_Runtime_MapSjw(APP_RUNTIME_CAN_SJW_TQ);
    hcan1.Init.TimeSeg1 = CAN_Runtime_MapBs1(APP_RUNTIME_CAN_BS1_TQ);
    hcan1.Init.TimeSeg2 = CAN_Runtime_MapBs2(APP_RUNTIME_CAN_BS2_TQ);
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = APP_RUNTIME_CAN_AUTO_BUS_OFF_ENABLE ? ENABLE : DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = APP_RUNTIME_CAN_NO_AUTO_RETRANSMISSION ? DISABLE : ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
}

static int CAN_Runtime_ConfigFilter(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

#if APP_RUNTIME_CAN_ACCEPT_ALL_FILTER
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
#else
    filter.FilterIdHigh = (uint16_t)(APP_RUNTIME_CAN_CMD_STEER_STDID << 5);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0xFFFFU;
    filter.FilterMaskIdLow = 0xFFF8U;
#endif

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        g_can_status.last_error_code = HAL_CAN_GetError(&hcan1);
        return -1;
    }

    return 0;
}

static void CAN_Runtime_PushRx(const CAN_Frame_t *frame)
{
    uint8_t next_head = (uint8_t)((g_can_rx_queue.head + 1U) % CAN_RUNTIME_RX_QUEUE_LEN);

    if (next_head == g_can_rx_queue.tail) {
        g_can_status.rx_overrun_count++;
        g_can_rx_queue.tail = (uint8_t)((g_can_rx_queue.tail + 1U) % CAN_RUNTIME_RX_QUEUE_LEN);
    }

    g_can_rx_queue.frames[g_can_rx_queue.head] = *frame;
    g_can_rx_queue.head = next_head;

    g_can_status.rx_count++;
    g_can_status.last_rx_id = frame->id;
}

static void CAN_Runtime_DrainRxFifo0(void)
{
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
        CAN_RxHeaderTypeDef rx_header = {0};
        CAN_Frame_t frame = {0};

        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, frame.data) != HAL_OK) {
            g_can_status.last_error_code = HAL_CAN_GetError(&hcan1);
            g_can_status.error_poll_count++;
            break;
        }

        frame.timestamp_ms = HAL_GetTick();
        frame.dlc = (uint8_t)rx_header.DLC;
        frame.is_extended = (uint8_t)((rx_header.IDE == CAN_ID_EXT) ? 1U : 0U);
        frame.is_remote = (uint8_t)((rx_header.RTR == CAN_RTR_REMOTE) ? 1U : 0U);
        frame.filter_match_index = (uint8_t)rx_header.FilterMatchIndex;
        frame.id = (rx_header.IDE == CAN_ID_EXT) ? rx_header.ExtId : rx_header.StdId;

        CAN_Runtime_PushRx(&frame);
    }
}

void CAN_Runtime_Init(void)
{
    memset(&g_can_status, 0, sizeof(g_can_status));
    memset((void *)&g_can_rx_queue, 0, sizeof(g_can_rx_queue));

    CAN_Runtime_ApplyInitConfig();

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        g_can_status.last_error_code = HAL_CAN_GetError(&hcan1);
        return;
    }

    if (CAN_Runtime_ConfigFilter() != 0) {
        return;
    }

    g_can_status.initialized = 1U;
    g_can_status.loopback_mode = (uint8_t)((APP_RUNTIME_CAN_MODE == APP_RUNTIME_CAN_MODE_LOOPBACK) ? 1U : 0U);
}

int CAN_Runtime_Start(void)
{
    if (g_can_status.initialized == 0U) {
        CAN_Runtime_Init();
    }

    if (g_can_status.initialized == 0U) {
        return -1;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        g_can_status.last_error_code = HAL_CAN_GetError(&hcan1);
        return -2;
    }

    g_can_status.started = 1U;
    return 0;
}

void CAN_Runtime_Stop(void)
{
    if (g_can_status.started == 0U) {
        return;
    }

    (void)HAL_CAN_Stop(&hcan1);
    g_can_status.started = 0U;
}

void CAN_Runtime_Service(void)
{
    uint32_t error_code = 0U;

    if (g_can_status.started == 0U) {
        return;
    }

    CAN_Runtime_DrainRxFifo0();

    error_code = HAL_CAN_GetError(&hcan1);
    if (error_code != HAL_CAN_ERROR_NONE) {
        g_can_status.last_error_code = error_code;
        g_can_status.error_poll_count++;
        if ((error_code & HAL_CAN_ERROR_BOF) != 0U) {
            g_can_status.bus_off_count++;
        }
    }
}

int CAN_Runtime_SendStd(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t payload[8] = {0};
    uint32_t mailbox = 0U;
    uint32_t start_ms = 0U;

    if ((g_can_status.started == 0U) || (dlc > 8U)) {
        return -1;
    }

    if ((dlc > 0U) && (data == NULL)) {
        return -2;
    }

    if (dlc > 0U) {
        memcpy(payload, data, dlc);
    }

    tx_header.StdId = std_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, payload, &mailbox) != HAL_OK) {
        g_can_status.last_error_code = HAL_CAN_GetError(&hcan1);
        g_can_status.tx_fail_count++;
        return -3;
    }

    start_ms = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox) != 0U) {
        if ((uint32_t)(HAL_GetTick() - start_ms) >= APP_RUNTIME_CAN_TX_TIMEOUT_MS) {
            g_can_status.last_error_code = HAL_CAN_GetError(&hcan1);
            g_can_status.tx_timeout_count++;
            g_can_status.tx_fail_count++;
            return -4;
        }
    }

    g_can_status.tx_count++;
    g_can_status.last_tx_id = std_id;
    return 0;
}

uint8_t CAN_Runtime_Pop(CAN_Frame_t *out_frame)
{
    if ((out_frame == NULL) || (g_can_rx_queue.head == g_can_rx_queue.tail)) {
        return 0U;
    }

    *out_frame = g_can_rx_queue.frames[g_can_rx_queue.tail];
    g_can_rx_queue.tail = (uint8_t)((g_can_rx_queue.tail + 1U) % CAN_RUNTIME_RX_QUEUE_LEN);
    return 1U;
}

CAN_RuntimeStatus_t CAN_Runtime_GetStatus(void)
{
    return g_can_status;
}

void CAN_Runtime_PrintStatus(void)
{
    printf("[CAN] init=%u started=%u mode=%s rx=%lu tx=%lu tx_fail=%lu tx_to=%lu rx_ovr=%lu err=%lu bus_off=%lu last_err=0x%08lX last_rx=0x%03lX last_tx=0x%03lX\r\n",
           (unsigned int)g_can_status.initialized,
           (unsigned int)g_can_status.started,
           (g_can_status.loopback_mode != 0U) ? "LOOPBACK" : "NORMAL",
           (unsigned long)g_can_status.rx_count,
           (unsigned long)g_can_status.tx_count,
           (unsigned long)g_can_status.tx_fail_count,
           (unsigned long)g_can_status.tx_timeout_count,
           (unsigned long)g_can_status.rx_overrun_count,
           (unsigned long)g_can_status.error_poll_count,
           (unsigned long)g_can_status.bus_off_count,
           (unsigned long)g_can_status.last_error_code,
           (unsigned long)g_can_status.last_rx_id,
           (unsigned long)g_can_status.last_tx_id);
}
