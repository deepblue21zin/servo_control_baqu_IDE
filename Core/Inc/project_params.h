#ifndef PROJECT_PARAMS_H
#define PROJECT_PARAMS_H

/*
 * 사용자가 자주 바꾸는 튜닝값과 시험용 스위치를 한곳에 모아둔 파일.
 * 기구/센서 변환 상수는 constants.h에 두고,
 * 각 모듈 내부에서만 쓰는 세부 상수는 해당 .c/.h 파일에 둔다.
 * 각 섹션에는 이 설정을 주로 소비하는 코드 경로를 함께 적어둔다.
 */

/* ========== App Runtime / 시험/벤치 스위치 ==========
 * 주요 사용처: Core/Src/app_runtime.c
 */
/* 입력 경로 선택:
 * - KEYBOARD: 현재 로컬 벤치 시험용 UART 키보드 입력 경로
 * - UDP: 실제 연동 시험 시 사용할 Ethernet/UDP 입력 경로
 * 시험 모드를 바꿀 때는 아래 APP_RUNTIME_INPUT_SOURCE만 변경하면 된다.
 */
#define APP_RUNTIME_INPUT_SOURCE_KEYBOARD        1U
#define APP_RUNTIME_INPUT_SOURCE_UDP             2U
#define APP_RUNTIME_INPUT_SOURCE                 APP_RUNTIME_INPUT_SOURCE_KEYBOARD /* 현재 기본값은 keyboard 시험 모드, 실제 UDP 시험 시 여기만 UDP로 변경 */
#if ((APP_RUNTIME_INPUT_SOURCE != APP_RUNTIME_INPUT_SOURCE_KEYBOARD) && \
     (APP_RUNTIME_INPUT_SOURCE != APP_RUNTIME_INPUT_SOURCE_UDP))
#error "APP_RUNTIME_INPUT_SOURCE must be APP_RUNTIME_INPUT_SOURCE_KEYBOARD or APP_RUNTIME_INPUT_SOURCE_UDP"
#endif
#define APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD     (APP_RUNTIME_INPUT_SOURCE == APP_RUNTIME_INPUT_SOURCE_KEYBOARD)
#define APP_RUNTIME_INPUT_SOURCE_IS_UDP          (APP_RUNTIME_INPUT_SOURCE == APP_RUNTIME_INPUT_SOURCE_UDP)
#define APP_RUNTIME_KEYBOARD_TEST_MODE           APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD /* 기존 코드/문서 호환용 별칭 */

#define APP_RUNTIME_AUTO_FIXED_PULSE_TEST        0        /* 1ms 루프에서 고정 펄스만 내보내는 벤치 전용 모드 사용 여부 */
#define APP_RUNTIME_AUTO_FIXED_PULSE_HZ          500000   /* 위 고정 펄스 모드 활성화 시 사용할 목표 펄스 주파수(Hz) */
#define APP_RUNTIME_KEYBOARD_STEP_DEG            1.0f     /* keyboard 입력에서 A/D 키 한 번당 바뀌는 steering 각도(deg) */
#define APP_RUNTIME_KEYBOARD_SCENARIO_ENABLE     1        /* keyboard bench에서 우회전 시나리오 자동 재생 기능 사용 여부 */
#define APP_RUNTIME_KEYBOARD_SCENARIO_STEP_DEG   10.0f    /* 우회전 시나리오에서 한 단계마다 증가/감소할 steering 각도(deg) */
#define APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_DEG   30.0f    /* 우회전 시나리오가 올라갈 최대 steering 각도(deg) */
#define APP_RUNTIME_KEYBOARD_SCENARIO_STEP_HOLD_MS 700U   /* 각 단계(10,20,30...)를 유지할 시간(ms) */
#define APP_RUNTIME_KEYBOARD_SCENARIO_PEAK_HOLD_MS 1200U  /* 최고 steering 각도에서 유지할 시간(ms) */
#define APP_RUNTIME_ENCODER_DIAG_ENABLE          1        /* ENCDBG 형식의 엔코더 진단 로그 출력 여부 */
#define APP_RUNTIME_ENCODER_DIAG_PERIOD_MS       100U     /* 엔코더 진단 로그 출력 주기(ms) */
#define APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE   0        /* 실제 TIM2 대신 펄스 적분값을 표시용 엔코더로 쓸지 여부 */
#define APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE      1        /* CSV 형식 주기 로그 출력 여부 */
#define APP_RUNTIME_PERIODIC_CSV_LOG_PERIOD_MS   100U     /* CSV 로그 출력 주기(ms) */
#define APP_RUNTIME_PERIODIC_DIAG_DIVIDER        100U     /* 1ms 빠른 루프 중 사람이 읽는 느린 DIAG를 몇 주기마다 찍을지 */

/* ========== App Runtime / 부팅 시퀀스 ==========
 * 주요 사용처: Core/Src/app_runtime.c::AppRuntime_Init()
 */
#define APP_RUNTIME_AUTO_START_CONTROL_ENABLE    1        /* 초기화 직후 position control을 자동 enable 할지 여부 */
#define APP_RUNTIME_RESET_ENCODER_ON_BOOT        1        /* 부팅 시 논리 엔코더 원점을 0으로 다시 맞출지 여부 */

/* ========== Position Control / 위치 제어 ==========
 * 주요 사용처: Core/Inc/position_control.h, Core/Src/position_control.c
 */
#define MAX_ANGLE_DEG                    4320.0f          /* 소프트웨어가 허용하는 motor축 최대 각도(deg) */
#define MIN_ANGLE_DEG                   -4320.0f          /* 소프트웨어가 허용하는 motor축 최소 각도(deg) */
#define MAX_TRACKING_ERROR_DEG           4500.0f          /* 추종 오차가 이 값을 넘으면 fault 후보로 보는 기준(deg) */
#define DEFAULT_KP                         80.0f          /* 기본 비례 이득: 오차가 클수록 더 큰 펄스 주파수를 요청 */
#define DEFAULT_KI                          5.0f          /* 기본 적분 이득: 작은 오차가 오래 남을 때 밀어주는 항 */
#define DEFAULT_KD                         20.0f          /* 기본 미분 이득: 오차 변화 속도에 반응하는 항 */
#define DEFAULT_INTEGRAL_LIMIT          1000.0f           /* 적분항이 과도하게 쌓이지 않도록 제한하는 상한 */
#define DEFAULT_OUTPUT_LIMIT           50000.0f           /* PID 출력 상한(실질적 속도 상한). 기존 10k에서 50k로 높여 저속 병목을 완화 */
#define STABLE_ERROR_THRESHOLD             0.5f           /* 목표 도달로 보기 시작하는 오차 범위(deg) */
#define STABLE_TIME_MS                     100U           /* 위 오차 범위 안에 이 시간 이상 머물면 CMD_REACHED 처리(ms) */
#define POSITION_COMMAND_TIMEOUT_MS          0U           /* legacy 기본 timeout 값. 실제 active timeout은 아래 fail-safe profile에서 결정 */

/* ========== Position Safety / 위치 제어 안전 ==========
 * 주요 사용처: Core/Src/position_control_safety.c
 * 참고:
 * - 소프트 angle limit(MAX/MIN + margin)은 profile과 무관하게 항상 유지한다.
 * - 아래 스위치는 tracking / velocity / timeout 같은 "추가 fail-safe"를 어떻게 쓸지 정한다.
 * - 파라미터 튜닝 중에는 PARAM_TEST, 실제 차체/연동 시험에서는 VEHICLE_TEST를 권장한다.
 */
#define POSITION_SAFETY_ANGLE_MARGIN_DEG    5.0f          /* 소프트 한계(MAX/MIN_ANGLE_DEG) 바깥으로 추가 허용하는 안전 여유(deg) */
#define POSITION_FAILSAFE_EXTRA_ENABLE       1            /* 0이면 추가 fail-safe(tracking/velocity/timeout)를 끄고 소프트 angle limit만 유지 */
#define POSITION_FAILSAFE_PROFILE_PARAM_TEST 1U           /* 파라미터 튜닝용: nuisance trip을 줄이는 프로파일 */
#define POSITION_FAILSAFE_PROFILE_VEHICLE_TEST 2U         /* 실제 차체/연동 시험용: tracking/velocity/timeout을 켜는 프로파일 */
#define POSITION_FAILSAFE_PROFILE            POSITION_FAILSAFE_PROFILE_PARAM_TEST /* 현재 기본은 튜닝 방해를 줄이는 PARAM_TEST */
#if ((POSITION_FAILSAFE_PROFILE != POSITION_FAILSAFE_PROFILE_PARAM_TEST) && \
     (POSITION_FAILSAFE_PROFILE != POSITION_FAILSAFE_PROFILE_VEHICLE_TEST))
#error "POSITION_FAILSAFE_PROFILE must be PARAM_TEST or VEHICLE_TEST"
#endif
#define POSITION_FAILSAFE_PARAM_TEST_MAX_ERROR_DEG        MAX_TRACKING_ERROR_DEG  /* 튜닝 중에는 큰 tracking error로 즉시 trip하지 않도록 넉넉하게 둠 */
#define POSITION_FAILSAFE_PARAM_TEST_MAX_VELOCITY_DEG_PER_S 0.0f                  /* 0이면 velocity fail-safe 비활성 */
#define POSITION_FAILSAFE_PARAM_TEST_TIMEOUT_MS           0U                      /* 0이면 timeout fail-safe 비활성 */
#define POSITION_FAILSAFE_VEHICLE_TEST_MAX_ERROR_DEG      800.0f                  /* 실제 시험 시작점용 motor축 tracking error 한계 */
#define POSITION_FAILSAFE_VEHICLE_TEST_MAX_VELOCITY_DEG_PER_S 450.0f              /* 실제 시험 시작점용 motor축 속도 한계 */
#define POSITION_FAILSAFE_VEHICLE_TEST_TIMEOUT_MS         5000U                   /* 실제 시험 시작점용 명령 timeout(ms), 현재 40deg급 스텝도 false trip이 덜 나도록 여유를 둠 */

/* ========== Pulse Output / 펄스 출력 ==========
 * 주요 사용처: Core/Inc/pulse_control.h, Core/Src/pulse_control.c
 */
#define DIR_ACTIVE_HIGH_FOR_CW               1            /* DIR 핀이 HIGH일 때 CW로 해석할지 여부 */
#define PULSECONTROL_MIN_FREQ_HZ            10U           /* 0이 아닌 출력에서 너무 느린 펄스를 막기 위한 최소 주파수(Hz) */
#define PULSECONTROL_MAX_FREQ_HZ        100000U           /* TIM1 펄스 출력의 펌웨어 상한 주파수(Hz) */
#define PULSECONTROL_DIRECTION_GUARD_MS      1U           /* 방향 전환 전후 stop/settle 대기 시간(ms) */

/* ========== Ethernet / UDP 연동 ==========
 * 주요 사용처: Core/Inc/ethernet_communication.h, Core/Src/ethernet_communication.c, Core/Src/app_runtime.c
 */
#define ETHCOMM_ASMS_IP_LAST_OCTET            5U           /* ASMS 5바이트 패킷을 허용할 송신자 IP 마지막 옥텟 */
#define ETHCOMM_PC_IP_LAST_OCTET              1U           /* PC 9바이트 패킷을 허용할 송신자 IP 마지막 옥텟 */
#define ETHCOMM_ASMS_PACKET_SIZE              5U           /* ASMS 패킷 기대 길이(byte) */
#define ETHCOMM_PC_PACKET_SIZE                9U           /* PC 패킷 기대 길이(byte) */
#define AUTODRIVE_UDP_PORT                5000U           /* 상위 제어기 명령을 받을 UDP 포트 번호 */
#define ETHCOMM_RX_TIMEOUT_MS              300U           /* 이 시간 동안 수신이 없으면 timeout으로 보는 기준(ms) */
#define ETHCOMM_LOG_ENABLE                   0            /* Ethernet/UDP 파서 상세 로그 출력 여부 */

/* ========== CAN / CAN1 연동 ==========
 * 주요 사용처: Core/Src/can.c, Core/Src/can_runtime.c, Core/Src/app_runtime.c
 * 참고:
 * - `can.c/h`는 CubeMX가 생성하는 CAN1 핸들/MSP 초기화 파일이다.
 * - `can_runtime.c/h`는 앱 명령 처리용 런타임 계층으로, filter/queue/status/send helper를 맡는다.
 * - bench에서 외부 노드 없이 먼저 확인하려면 LOOPBACK 모드로 바꾸면 내부 self-test가 가능하다.
 */
#define APP_RUNTIME_CAN_ENABLE                  1U        /* CAN1 런타임 초기화/서비스 사용 여부 */
#define APP_RUNTIME_CAN_RX_LOG_ENABLE           1U        /* 수신 프레임을 UART 로그로 바로 출력할지 여부 */
#define APP_RUNTIME_CAN_AUTO_START              1U        /* 부팅 시 CAN1을 자동 start할지 여부 */
#define APP_RUNTIME_CAN_ACCEPT_ALL_FILTER       1U        /* 1이면 filter 0에서 모든 표준/확장 프레임을 수신한다 */
#define APP_RUNTIME_CAN_MODE_NORMAL             0U
#define APP_RUNTIME_CAN_MODE_LOOPBACK           1U
#define APP_RUNTIME_CAN_MODE                    APP_RUNTIME_CAN_MODE_NORMAL /* 외부 CAN 버스 없이 내부 self-test를 할 때는 LOOPBACK으로 변경 */
#if ((APP_RUNTIME_CAN_MODE != APP_RUNTIME_CAN_MODE_NORMAL) && \
     (APP_RUNTIME_CAN_MODE != APP_RUNTIME_CAN_MODE_LOOPBACK))
#error "APP_RUNTIME_CAN_MODE must be NORMAL or LOOPBACK"
#endif
#define APP_RUNTIME_CAN_PRESCALER               5U        /* IOC 기준 500 kbps timing */
#define APP_RUNTIME_CAN_BS1_TQ                 15U        /* IOC 기준 Bit Segment 1 */
#define APP_RUNTIME_CAN_BS2_TQ                  2U        /* IOC 기준 Bit Segment 2 */
#define APP_RUNTIME_CAN_SJW_TQ                  1U        /* Sync Jump Width */
#define APP_RUNTIME_CAN_AUTO_BUS_OFF_ENABLE     1U        /* bus-off 발생 시 하드웨어 자동 복귀 사용 여부 */
#define APP_RUNTIME_CAN_NO_AUTO_RETRANSMISSION  1U        /* ACK 실패 시 자동 재전송 비활성화 여부 (IOC NART=ENABLE와 동일) */
#define APP_RUNTIME_CAN_INIT_TIMEOUT_MS        50U        /* init/start 진입/복귀 대기 timeout(ms) */
#define APP_RUNTIME_CAN_TX_TIMEOUT_MS          20U        /* 한 프레임 전송 완료 대기 timeout(ms) */
#define APP_RUNTIME_CAN_CMD_STEER_STDID     0x201U       /* signed int16 0.1deg steering target command ID */
#define APP_RUNTIME_CAN_CMD_CONTROL_STDID   0x202U       /* 1-byte control command ID: 0 disable, 1 enable, 2 estop, 3 center */
#define APP_RUNTIME_CAN_QUERY_STDID         0x203U       /* 상태 조회 요청 ID */
#define APP_RUNTIME_CAN_STATUS_STDID        0x301U       /* 상태 응답 ID */
#define APP_RUNTIME_CAN_TEST_TX_STDID       0x321U       /* keyboard에서 보내는 시험용 TX frame ID */

/* ========== Latency Profiler / 지연시간 계측 ==========
 * 주요 사용처: Core/Inc/latency_profiler.h, Core/Src/latency_profiler.c, Core/Src/app_runtime.c
 */
#define LATENCY_PROFILER_ENABLE              1            /* latency 계측 기능 자체를 컴파일에 포함할지 여부 */
#define LATENCY_LOG_ENABLE                   0            /* 매 주기 latency 로그를 바로 찍을지 여부 */
#define LATENCY_MAX_SAMPLES               2048U           /* 단계별로 최대 몇 개 샘플까지 저장할지 */
#define LATENCY_AUTO_REPORT_ENABLE           1            /* 샘플이 쌓였을 때 자동 배치 보고를 낼지 여부 */
#define LATENCY_AUTO_REPORT_SAMPLES       2000U           /* 자동 보고 1회를 만들기 위해 필요한 샘플 개수 */

#endif /* PROJECT_PARAMS_H */
