#ifndef __CAN_RUNTIME_H
#define __CAN_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

typedef struct {
    uint32_t id;
    uint32_t timestamp_ms;
    uint8_t dlc;
    uint8_t is_extended;
    uint8_t is_remote;
    uint8_t filter_match_index;
    uint8_t data[8];
} CAN_Frame_t;

typedef struct {
    uint8_t initialized;
    uint8_t started;
    uint8_t loopback_mode;
    uint8_t reserved;
    uint32_t rx_count;
    uint32_t rx_overrun_count;
    uint32_t tx_count;
    uint32_t tx_fail_count;
    uint32_t tx_timeout_count;
    uint32_t error_poll_count;
    uint32_t bus_off_count;
    uint32_t last_error_code;
    uint32_t last_rx_id;
    uint32_t last_tx_id;
} CAN_RuntimeStatus_t;

void CAN_Runtime_Init(void);
int CAN_Runtime_Start(void);
void CAN_Runtime_Stop(void);
void CAN_Runtime_Service(void);

int CAN_Runtime_SendStd(uint16_t std_id, const uint8_t *data, uint8_t dlc);
uint8_t CAN_Runtime_Pop(CAN_Frame_t *out_frame);

CAN_RuntimeStatus_t CAN_Runtime_GetStatus(void);
void CAN_Runtime_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_RUNTIME_H */
