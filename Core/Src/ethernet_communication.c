/*
 * ethernet_communication.c
 *
 * 구조만 잡아둔 템플릿:
 * - 초기화
 * - 주기 업데이트
 * - 라인 파싱
 * - 명령 디스패치
 *
 * TODO 표시된 부분을 직접 구현해서 채우면 됩니다.
 */

#include "ethernet_communication.h"

#include "main.h"
#include "position_control.h"
#include "pulse_control.h"
#include "relay_control.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ========== Private Types ========== */

typedef struct {
    bool initialized;
    EthComm_Config_t cfg;
    uint16_t rx_len;
} EthComm_Context_t;

/* ========== Private Variables ========== */

static EthComm_Context_t g_eth = {0};

/* ========== Private Function Prototypes ========== */

static int EthComm_ReadRxByte(uint8_t *out_byte);
static int EthComm_ProcessRxByte(uint8_t byte);
static int EthComm_ParseCommand(const char *line, EthComm_Command_t *out_cmd);
static int EthComm_DispatchCommand(const EthComm_Command_t *cmd);
static int EthComm_SendStatus(void);

/* ========== Public Functions ========== */

int EthComm_Init(const EthComm_Config_t *config)
{
    if (config == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }
    if (config->rx_buffer == NULL || config->rx_buffer_size == 0) {
        return ETH_COMM_ERR_BAD_PARAM;
    }
    if (config->tx_buffer == NULL || config->tx_buffer_size == 0) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    memset(&g_eth, 0, sizeof(g_eth));
    g_eth.cfg = *config;
    g_eth.initialized = true;

    /* TODO: Ethernet PHY/Socket/UART 브리지 초기화 코드를 여기에 추가 */

    return ETH_COMM_OK;
}

void EthComm_Update(void)
{
    uint8_t byte = 0;

    if (!g_eth.initialized) {
        return;
    }

    /*
     * TODO:
     * - non-blocking 방식으로 수신 바이트를 반복 읽기
     * - 데이터가 있으면 EthComm_ProcessRxByte 호출
     * - 아래 while 조건부는 실제 드라이버에 맞게 교체
     */
    while (EthComm_ReadRxByte(&byte) == ETH_COMM_OK) {
        (void)EthComm_ProcessRxByte(byte);
    }
}

bool EthComm_IsInitialized(void)
{
    return g_eth.initialized;
}

int EthComm_SendString(const char *msg)
{
    size_t len = 0;

    if (!g_eth.initialized || msg == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    len = strlen(msg);
    if (len >= g_eth.cfg.tx_buffer_size) {
        len = g_eth.cfg.tx_buffer_size - 1;
    }

    memcpy(g_eth.cfg.tx_buffer, msg, len);
    g_eth.cfg.tx_buffer[len] = '\0';

    /* TODO: 실제 전송 함수로 교체 (LAN8742/LwIP, Wiznet, UART 등) */
    /* 예: HAL_UART_Transmit(...), netconn_write(...), send(...) */

    return ETH_COMM_OK;
}

int EthComm_HandleLine(const char *line)
{
    EthComm_Command_t cmd = {0};
    int ret = 0;

    if (!g_eth.initialized || line == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    ret = EthComm_ParseCommand(line, &cmd);
    if (ret != ETH_COMM_OK) {
        (void)EthComm_SendString("ERR PARSE\r\n");
        return ret;
    }

    ret = EthComm_DispatchCommand(&cmd);
    if (ret != ETH_COMM_OK) {
        (void)EthComm_SendString("ERR EXEC\r\n");
        return ret;
    }

    return ETH_COMM_OK;
}

/* ========== Private Functions ========== */

static int EthComm_ReadRxByte(uint8_t *out_byte)
{
    if (out_byte == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    /*
     * TODO:
     * - 수신 데이터가 있으면 *out_byte에 1바이트 저장 후 ETH_COMM_OK 반환
     * - 데이터가 없으면 ETH_COMM_ERR_RX_FAIL 또는 별도 코드 반환
     * - 블로킹 금지(주기 루프 성능 유지)
     */
    (void)out_byte;
    return ETH_COMM_ERR_RX_FAIL;
}

static int EthComm_ProcessRxByte(uint8_t byte)
{
    char *line = (char *)g_eth.cfg.rx_buffer;

    if (!g_eth.initialized) {
        return ETH_COMM_ERR_NOT_INIT;
    }

    /* 줄바꿈 기반 프로토콜: "CMD ARG\r\n" */
    if (byte == '\r') {
        return ETH_COMM_OK;
    }

    if (byte == '\n') {
        line[g_eth.rx_len] = '\0';
        g_eth.rx_len = 0;
        return EthComm_HandleLine(line);
    }

    if (g_eth.rx_len < (uint16_t)(g_eth.cfg.rx_buffer_size - 1)) {
        line[g_eth.rx_len++] = (char)byte;
    } else {
        /* 버퍼 오버플로우 방지: 라인 폐기 */
        g_eth.rx_len = 0;
        return ETH_COMM_ERR_RX_FAIL;
    }

    return ETH_COMM_OK;
}

static int EthComm_ParseCommand(const char *line, EthComm_Command_t *out_cmd)
{
    char cmd[24] = {0};
    char arg[32] = {0};
    int count = 0;

    if (line == NULL || out_cmd == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    memset(out_cmd, 0, sizeof(*out_cmd));

    /*
     * 프로토콜 예시:
     * - "SERVO ON"
     * - "SERVO OFF"
     * - "SET TARGET 12.5"
     * - "STOP"
     * - "GET STATUS"
     */
    count = sscanf(line, "%23s %31s", cmd, arg);
    if (count <= 0) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    if (strcmp(cmd, "STOP") == 0) {
        out_cmd->type = ETH_CMD_STOP;
        return ETH_COMM_OK;
    }

    if (strcmp(cmd, "GET") == 0 && strcmp(arg, "STATUS") == 0) {
        out_cmd->type = ETH_CMD_GET_STATUS;
        return ETH_COMM_OK;
    }

    if (strcmp(cmd, "SERVO") == 0) {
        if (strcmp(arg, "ON") == 0) {
            out_cmd->type = ETH_CMD_SERVO_ON;
            return ETH_COMM_OK;
        }
        if (strcmp(arg, "OFF") == 0) {
            out_cmd->type = ETH_CMD_SERVO_OFF;
            return ETH_COMM_OK;
        }
    }

    if (strncmp(line, "SET TARGET ", 11) == 0) {
        out_cmd->type = ETH_CMD_SET_TARGET;
        out_cmd->f32_value = strtof(&line[11], NULL);
        return ETH_COMM_OK;
    }

    return ETH_COMM_ERR_BAD_PARAM;
}

static int EthComm_DispatchCommand(const EthComm_Command_t *cmd)
{
    if (cmd == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    switch (cmd->type) {
    case ETH_CMD_SERVO_ON:
        Relay_ServoOn();
        return EthComm_SendString("OK SERVO ON\r\n");

    case ETH_CMD_SERVO_OFF:
        Relay_ServoOff();
        return EthComm_SendString("OK SERVO OFF\r\n");

    case ETH_CMD_SET_TARGET:
        if (PositionControl_SetTarget(cmd->f32_value) != 0) {
            return EthComm_SendString("ERR TARGET\r\n");
        }
        return EthComm_SendString("OK TARGET\r\n");

    case ETH_CMD_STOP:
        PositionControl_Disable();
        PulseControl_Stop();
        return EthComm_SendString("OK STOP\r\n");

    case ETH_CMD_GET_STATUS:
        return EthComm_SendStatus();

    default:
        return ETH_COMM_ERR_BAD_PARAM;
    }
}

static int EthComm_SendStatus(void)
{
    float cur = PositionControl_GetCurrentAngle();
    float tgt = PositionControl_GetTarget();
    float err = PositionControl_GetError();
    int n = 0;

    n = snprintf((char *)g_eth.cfg.tx_buffer,
                 g_eth.cfg.tx_buffer_size,
                 "OK STATUS CUR=%.2f TGT=%.2f ERR=%.2f\r\n",
                 cur, tgt, err);
    if (n <= 0) {
        return ETH_COMM_ERR_TX_FAIL;
    }

    /* TODO: 실제 네트워크 전송 함수로 교체 */
    return EthComm_SendString((char *)g_eth.cfg.tx_buffer);
}

/* ============================================================
 * UDP 자율주행 수신 구현
 * ============================================================ */

static struct udp_pcb      *g_udp_pcb   = NULL;
static AutoDrive_Packet_t   g_latest_pkt;
static volatile bool        g_new_data  = false;

/**
 * @brief LwIP가 UDP 패킷 수신 시 자동 호출하는 콜백
 *
 * 흐름:
 *   인지PC 브로드캐스트 → 이더넷 허브 → STM32 NIC
 *   → LwIP 스택 → ethernetif_input() → udp_input()
 *   → 이 콜백 호출
 *
 * [필터링 로직]
 *   header == 0xAA   : 우리 시스템 패킷인지 확인
 *   msg_type == 0x01 : 제어 메시지인지 확인 (속도·조향·ASMS)
 *   → 두 조건 모두 만족할 때만 g_latest_pkt에 저장
 */
static void udp_recv_cb(void *arg,
                        struct udp_pcb *pcb,
                        struct pbuf *p,
                        const ip_addr_t *addr,
                        u16_t port)
{
    (void)arg; (void)pcb; (void)addr; (void)port;

    if (p == NULL) {
        return;
    }

    /* 패킷 크기 확인 (잘린 패킷 방어) */
    if (p->tot_len < (u16_t)sizeof(AutoDrive_Packet_t)) {
        printf("[EthComm] RX too short: %u bytes\r\n", p->tot_len);
        pbuf_free(p);
        return;
    }

    /* pbuf → 로컬 구조체로 복사 (pbuf가 체인일 수 있어서 copy_partial 사용) */
    AutoDrive_Packet_t pkt;
    pbuf_copy_partial(p, &pkt, sizeof(pkt), 0);
    pbuf_free(p);   /* LwIP 메모리 즉시 해제 (필수!) */

    /* ── 필터 1: 헤더 확인 (다른 시스템 패킷 걸러냄) ── */
    if (pkt.header != AUTODRIVE_PKT_HEADER) {
        return;
    }

    /* ── 필터 2: 타입 확인 (제어 메시지만 처리) ── */
    if (pkt.msg_type != AUTODRIVE_MSG_CTRL) {
        return;
    }

    /* ── 안전 검증: 조향각 범위 ── */
    if (pkt.steering_angle < -45.0f || pkt.steering_angle > 45.0f) {
        printf("[EthComm] Invalid angle: %.1f\r\n", pkt.steering_angle);
        return;
    }

    /* ── 비상정지 플래그 확인 (bit0 = 1이면 비상정지) ── */
    if (pkt.flags & 0x01) {
        PositionControl_Disable();
        PulseControl_Stop();
        printf("[EthComm] EMG STOP from perception!\r\n");
        return;
    }

    /* ── 데이터 저장 ── */
    g_latest_pkt = pkt;
    g_new_data   = true;
}

/**
 * @brief UDP 수신 소켓 초기화 (MX_LWIP_Init() 이후 1회 호출)
 */
void EthComm_UDP_Init(void)
{
    g_udp_pcb = udp_new();
    if (g_udp_pcb == NULL) {
        printf("[EthComm] ERROR: udp_new() failed\r\n");
        return;
    }

    err_t err = udp_bind(g_udp_pcb, IP_ADDR_ANY, AUTODRIVE_UDP_PORT);
    if (err != ERR_OK) {
        printf("[EthComm] ERROR: udp_bind() failed (%d)\r\n", (int)err);
        udp_remove(g_udp_pcb);
        g_udp_pcb = NULL;
        return;
    }

    udp_recv(g_udp_pcb, udp_recv_cb, NULL);
    printf("[EthComm] UDP ready  PORT:%d\r\n", AUTODRIVE_UDP_PORT);
}

/**
 * @brief 새 패킷 수신 여부 확인
 * @return true : 새 데이터 있음 → EthComm_GetLatestData() 로 읽을 것
 */
bool EthComm_HasNewData(void)
{
    return g_new_data;
}

/**
 * @brief 가장 최근 수신 패킷 반환 (플래그 초기화)
 */
AutoDrive_Packet_t EthComm_GetLatestData(void)
{
    g_new_data = false;
    return g_latest_pkt;
}
