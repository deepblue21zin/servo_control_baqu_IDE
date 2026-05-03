# STM32 조향 서보 제어 시스템 초심자 설명서

작성일: 2026-04-27  
대상: 이 프로젝트를 처음 보는 팀원, 교수님, 리뷰어, 현업 엔지니어

## 1. 이 시스템의 목표

이 프로젝트의 목표는 STM32F429ZI 보드를 이용해 **자율주행 조향 액추에이터 하위 제어기**를 만드는 것이다.

상위 제어기, 예를 들어 PC, Jetson, Raspberry Pi, 자율주행 판단 모듈은 주행 상황을 보고 "조향을 몇 도로 돌릴지"를 결정한다. 이 프로젝트의 STM32는 그 목표 조향각을 받아서 실제 서보드라이버와 서보모터가 이해할 수 있는 pulse/direction 신호로 바꾸고, 엔코더 피드백을 이용해 목표 위치에 도달하도록 제어한다.

한 줄로 정리하면 아래와 같다.

```text
상위 제어기의 목표 조향각(steering_deg)
  -> STM32 내부 위치 제어
  -> pulse/direction 출력
  -> 서보드라이버/서보모터 구동
  -> 엔코더 피드백으로 현재 위치 확인
```

이 프로젝트가 단순 모터 데모와 다른 점은 다음과 같다.

- 모터를 그냥 회전시키는 것이 아니라 목표 각도를 기준으로 위치 제어한다.
- 1 ms 주기 제어 루프에서 현재 위치와 목표 위치의 오차를 계속 계산한다.
- PID 출력값을 pulse 주파수로 바꾸어 서보드라이버에 전달한다.
- 엔코더 카운트를 각도로 변환해 피드백으로 사용한다.
- CSV, `[ENCDBG]`, `CMD_*`, latency 로그를 남겨 실험 결과를 추적할 수 있다.
- 안전 제한, ESTOP, timeout, lifecycle 같은 실제 제어기 운영 요소를 고려한다.

## 2. 전체 구성

### 2.1 하드웨어 구성

| 항목 | 현재 기준 |
|---|---|
| MCU | STM32F429ZI, 180 MHz |
| Servo Driver | LS ELECTRIC XDL-L7SA004BAA |
| Servo Motor / Encoder | XML-FBL04AMK1, 12000 PPR |
| Encoder Count | 12000 PPR x4 = 48000 count/rev |
| Control Loop | SysTick 기반 1 ms |
| Pulse Output | `PE9 = TIM1_CH1` |
| Direction Output | `PE10 = GPIO` |
| Encoder Input | `PA0 = TIM2_CH1`, `PB3 = TIM2_CH2` |
| Debug UART | USART3, `printf()` 출력 |
| Communication | Keyboard bench 기본, UDP/CAN 경로 존재 |
| Watchdog | IWDG 사용 |

### 2.2 소프트웨어 구성

전체 소프트웨어는 크게 아래 계층으로 나눌 수 있다.

```text
CubeMX 초기화 계층
  main.c, gpio.c, tim.c, usart.c, adc.c, can.c, lwip.c

Application Runtime 계층
  app_runtime.c

제어 계층
  position_control.c
  position_control_safety.c
  position_control_diag.c

입출력 계층
  encoder_reader.c
  pulse_control.c
  relay_control.c
  ethernet_communication.c
  can_runtime.c

계측/문서화 계층
  latency_profiler.c
  debug_vars.c
  Doc/steering_portal
  Doc/putty
```

초심자는 `main.c`를 전체 로직의 중심으로 오해하기 쉽지만, 현재 구조에서는 `main.c`가 매우 얇다. 실제 동작은 대부분 `app_runtime.c`와 제어 모듈들이 담당한다.

## 3. 전체 데이터 흐름

### 3.1 가장 중요한 흐름

```text
1. 목표 입력
   keyboard / UDP / CAN에서 목표 조향각을 받음

2. 단위 변환
   steering_deg -> motor_deg

3. 현재 위치 측정
   TIM2 encoder raw count -> 누적 count -> motor_deg

4. 위치 제어
   error = target_motor_deg - current_motor_deg
   PID(error) -> pulse_hz

5. 출력 생성
   pulse_hz 부호 -> 방향
   pulse_hz 크기 -> TIM1 PWM 주파수

6. 실제 구동
   PE9 pulse, PE10 direction -> line driver -> servo drive

7. 로그/검증
   CSV, ENCDBG, lifecycle, latency 출력
```

### 3.2 단위 체계

이 프로젝트에서 단위가 매우 중요하다. 상위 제어기는 조향축 기준 각도인 `steering_deg`를 다루고, STM32 내부 제어기는 모터축 기준 각도인 `motor_deg`를 다룬다.

| 단위 | 의미 | 사용 위치 |
|---|---|---|
| `steering_deg` | 실제 조향축 각도 | 외부 입력, 로그 표시 |
| `motor_deg` | 서보모터 축 각도 | PID 내부 제어 |
| `enc_count` | 엔코더 누적 카운트 | 센서 피드백 |
| `pulse_hz` | 서보드라이버 입력 펄스 주파수 | actuator 출력 |

현재 변환 기준은 `Core/Inc/constants.h`에 있다.

```c
#define STEERING_GEAR_RATIO             12.5f
#define MOTOR_DEG_PER_STEERING_DEG      STEERING_GEAR_RATIO
#define STEERING_DEG_PER_MOTOR_DEG      (1.0f / MOTOR_DEG_PER_STEERING_DEG)

static inline float SteeringDegToMotorDeg(float steering_deg)
{
    return steering_deg * MOTOR_DEG_PER_STEERING_DEG;
}

static inline float MotorDegToSteeringDeg(float motor_deg)
{
    return motor_deg * STEERING_DEG_PER_MOTOR_DEG;
}
```

예를 들어 상위 제어기가 `20 deg` 조향 명령을 주면 내부 목표는 다음과 같다.

```text
20 steering_deg x 12.5 = 250 motor_deg
```

엔코더는 1회전 48000 count이고 360 deg를 한 바퀴로 본다.

```text
ENCODER_DEG_PER_COUNT = 360 / 48000 = 0.0075 motor_deg/count
```

서보드라이버 펄스 기준은 아래와 같다.

```text
DEG_PER_PULSE = 0.003 motor_deg/pulse
PULSE_PER_DEG = 333.33 pulse/motor_deg
```

## 4. 부팅부터 제어까지 실행 순서

### 4.1 `main.c`의 역할

`Core/Src/main.c`는 CubeMX가 생성한 주변장치 초기화를 수행하고, 이후 애플리케이션 런타임에 제어권을 넘긴다.

흐름은 아래와 같다.

```text
main()
  -> HAL_Init()
  -> SystemClock_Config()
  -> MX_GPIO_Init()
  -> MX_USART3_UART_Init()
  -> MX_USART1_UART_Init()
  -> MX_ADC1_Init()
  -> MX_TIM1_Init()
  -> MX_IWDG_Init()
  -> MX_LWIP_Init()
  -> MX_TIM2_Init()
  -> MX_CAN1_Init()
  -> AppRuntime_Init()
  -> while(1)
       -> AppRuntime_RunIteration()
```

`main.c`는 시스템을 켜고 런타임을 계속 호출하는 역할만 한다. 실제 입력 처리, 제어 실행, 로그 출력은 `AppRuntime` 이후의 모듈들이 담당한다.

### 4.2 `AppRuntime_Init()`에서 하는 일

`Core/Src/app_runtime.c`의 `AppRuntime_Init()`는 CubeMX 초기화 이후 프로젝트 전용 모듈을 켠다.

주요 동작은 다음과 같다.

```text
LatencyProfiler_Init()
AppRuntime_ConfigureDirectionPin()
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL)
Relay_Init()
PulseControl_Init()
EncoderReader_Init()
EncoderReader_EnableVirtualFeedback(...)
PositionControl_Init()
CAN_Runtime_Start()
Relay_ServoOn()
EncoderReader_Reset()
PositionControl_SetTargetWithSource(0 deg)
PositionControl_Enable()
Keyboard help / CSV header 출력 또는 UDP 초기화
```

현재 기본값은 keyboard bench 중심이다.

```c
#define APP_RUNTIME_INPUT_SOURCE APP_RUNTIME_INPUT_SOURCE_KEYBOARD
#define APP_RUNTIME_ENCODER_DIAG_ENABLE 1
#define APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE 1
#define APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE 0
```

즉 현재 기본 빌드는 실제 TIM2 엔코더 진단을 켜고, keyboard 입력으로 목표각을 넣는 벤치 시험용 설정이다.

### 4.3 `AppRuntime_RunIteration()`에서 하는 일

`AppRuntime_RunIteration()`은 `while(1)`에서 계속 호출된다.

현재 keyboard 빌드 기준 흐름은 아래와 같다.

```text
1. Keyboard 입력 처리
2. Keyboard scenario 처리
3. CAN 수신/명령 처리
4. 1 ms interrupt_flag가 있으면 PositionControl_Update()
5. CSV 로그 출력
6. latency batch 출력 여부 확인
7. watchdog refresh
```

UDP 빌드로 바꾸면 keyboard 대신 `MX_LWIP_Process()`와 UDP packet 처리 경로가 활성화된다.

## 5. 코드 구조와 파일별 역할

### 5.1 핵심 파일 요약

| 파일 | 역할 | 핵심 입력 | 핵심 출력 |
|---|---|---|---|
| `Core/Src/main.c` | CubeMX 초기화와 runtime 호출 | 전원/리셋 | `AppRuntime_Init`, `AppRuntime_RunIteration` |
| `Core/Src/app_runtime.c` | 전체 실행 흐름, 입력, 로그, watchdog 관리 | keyboard/UDP/CAN, 1 ms flag | target command, CSV, ENCDBG |
| `Core/Src/position_control.c` | PID 위치 제어와 command lifecycle | target/current angle | pulse frequency command |
| `Core/Src/position_control_safety.c` | 각도/오차/속도/timeout 안전 평가 | current/error/velocity | safe/fault result |
| `Core/Src/position_control_diag.c` | 상태 문자열, debug vars, 요약 출력 | controller state | readable log/debug vars |
| `Core/Src/encoder_reader.c` | TIM2 encoder count를 각도로 변환 | TIM2 raw count | current motor_deg, count |
| `Core/Src/pulse_control.c` | PID 출력을 pulse/direction으로 변환 | signed pulse_hz | TIM1 PWM, DIR GPIO |
| `Core/Src/relay_control.c` | servo on/off, emergency GPIO 제어 | enable/estop 요청 | SVON/EMG GPIO |
| `Core/Src/ethernet_communication.c` | UDP 패킷 파싱, mode/target 저장 | ASMS 5B, PC 9B UDP | `AutoDrive_Packet_t`, mode |
| `Core/Src/can_runtime.c` | CAN1 filter, RX queue, TX helper | CAN frame | command/status frame |
| `Core/Src/latency_profiler.c` | Sense/Control/Actuate/Comms 시간 계측 | stage begin/end | avg/p99/max latency |
| `Core/Inc/project_params.h` | 운영/시험 파라미터 통합 | build-time 설정 | 모듈별 설정값 |
| `Core/Inc/constants.h` | 기구/센서/펄스 변환 상수 | 고정 상수 | 단위 변환 함수 |

## 6. 모듈별 상세 설명

### 6.1 `app_runtime.c`

`app_runtime.c`는 이 프로젝트의 운영 관리자다. 주변 모듈들을 직접 구현하지는 않지만, 언제 어떤 모듈을 호출할지 결정한다.

#### 처리하는 데이터

| 데이터 | 출처 | 처리 내용 | 다음 목적지 |
|---|---|---|---|
| keyboard 문자 | USART3 | A/D/S/E/Q/X/P/L 등 명령 해석 | `PositionControl_*` |
| numeric target | USART3 | 문자열을 float 목표각으로 변환 | `PositionControl_SetTargetWithSource()` |
| UDP packet | LwIP | mode, steering angle, ESTOP 파싱 | `PositionControl_*` |
| CAN frame | CAN1 | steering/control/query frame 해석 | `PositionControl_*`, status TX |
| 1 ms flag | SysTick | fast tick 실행 여부 판단 | `PositionControl_Update()` |
| controller state | `PositionControl_GetState()` | CSV/DIAG 로그로 변환 | UART 출력 |
| encoder raw | TIM2 | `[ENCDBG]` 진단 출력 | UART 출력 |

#### 주요 함수

| 함수 | 역할 | 활용 시점 |
|---|---|---|
| `AppRuntime_Init()` | 프로젝트 전용 초기화 수행 | `main()` 초기화 끝난 직후 |
| `AppRuntime_RunIteration()` | super-loop 1회 실행 | `while(1)`마다 호출 |
| `AppRuntime_ServiceFastTick()` | 1 ms 제어 경로 실행 | `interrupt_flag`가 켜졌을 때 |
| `AppRuntime_KeyboardProcessInput()` | UART 키 입력 처리 | keyboard bench 빌드 |
| `AppRuntime_KeyboardApplyTarget()` | keyboard 목표각을 motor_deg로 변환해 controller에 전달 | A/D/S 또는 숫자 입력 후 |
| `AppRuntime_ServiceUdpComms()` | UDP mode/target/timeout/ESTOP 처리 | UDP 빌드 |
| `AppRuntime_ServiceCan()` | CAN RX queue를 비우고 command 처리 | CAN 활성화 시 |
| `AppRuntime_ServicePeriodicCsv()` | 주기 CSV telemetry 출력 | bench log 수집 |
| `AppRuntime_ServiceEncoderRuntimeDiag()` | TIM2 raw register와 A/B 핀 상태 출력 | encoder truth 확인 |
| `AppRuntime_TryLatencyAutoReport()` | latency batch 출력 | 샘플 수가 충분할 때 |

#### Keyboard bench 입력

| 키 | 의미 |
|---|---|
| `A` | 목표 조향각을 왼쪽으로 1 deg 감소 |
| `D` | 목표 조향각을 오른쪽으로 1 deg 증가 |
| `S` | 목표 조향각을 0 deg로 이동 |
| `R` | 오른쪽 조향 시나리오 실행 |
| `C` | 시나리오 취소 |
| `E` | 위치 제어 enable |
| `Q` | 위치 제어 disable |
| `X` | emergency stop |
| `P` | 현재 snapshot 출력 |
| `L` | CSV log on/off |
| `H` | 도움말 출력 |

### 6.2 `position_control.c`

`position_control.c`는 실제 위치 제어의 중심이다. 목표 위치와 현재 위치의 차이를 계산하고, PID 제어 결과를 pulse 주파수로 만든다.

#### 처리하는 데이터

| 데이터 | 의미 |
|---|---|
| `state.target_angle` | 내부 목표 motor_deg |
| `state.current_angle` | 엔코더에서 읽은 현재 motor_deg |
| `state.error` | `target_angle - current_angle` |
| `state.output` | PID 출력, `pulse_hz`로 사용 |
| `control_enabled` | 제어 루프가 실제 출력을 낼 수 있는지 |
| `control_mode` | IDLE, POSITION, MANUAL, EMERGENCY |
| `command_lifecycle` | 명령 시작/도달/중단/고장 추적 |
| `measured_velocity_deg_per_s` | 현재 motor_deg/s 추정 속도 |

#### 제어 루프 처리 순서

`PositionControl_Update()`는 1 ms마다 호출되는 핵심 함수다.

```text
1. control_enabled 확인
   disabled이면 현재 각도만 읽고 PulseControl_Stop()

2. Sense 단계
   EncoderReader_GetAngleDeg()
   dt 계산
   velocity 계산
   error 계산
   stats 업데이트

3. Control 단계
   command timeout 확인
   PositionControlSafety_Evaluate()
   PID_Calculate()

4. Actuate 단계
   PulseControl_SetFrequency((int32_t)state.output)

5. Stable 판단
   abs(error) < 0.5 deg 상태가 100 ms 이상이면 CMD_REACHED
   CMD_REACHED 시 pulse stop

6. 진단 동기화
   PositionControlDiag_UpdateDebugVars()
```

#### 주요 함수

| 함수 | 역할 | 활용 |
|---|---|---|
| `PositionControl_Init()` | PID 상태, lifecycle, safety limit 초기화 | 부팅 초기화 |
| `PositionControl_Update()` | 1 ms 위치 제어 실행 | fast tick |
| `PositionControl_SetTarget()` | 목표 motor_deg 설정 | source 구분이 필요 없을 때 |
| `PositionControl_SetTargetWithSource()` | 목표 motor_deg와 command source 설정 | keyboard/UDP/CAN/service |
| `PositionControl_Enable()` | 제어 enable, emergency release | 운전 시작 |
| `PositionControl_Disable()` | 제어 disable, pulse stop | 운전 중지 |
| `PositionControl_EmergencyStop()` | pulse stop, relay emergency, EMERGENCY mode 진입 | ESTOP/fault |
| `PositionControl_SetPID()` | Kp/Ki/Kd 변경 | 튜닝 |
| `PositionControl_GetState()` | 현재 target/current/error/output snapshot 반환 | 로그/상태 표시 |
| `PositionControl_GetCommandLifecycle()` | command 상태 반환 | CSV/DIAG |
| `PositionControl_GetStats()` | max/avg error, overshoot 등 통계 반환 | 분석 |
| `PositionControl_SetSafetyLimits()` | safety limit 변경 | profile/tuning |
| `PositionControl_CheckSafety()` | 현재 상태 safety 평가 | 수동 검사 |

#### Command lifecycle

명령 lifecycle은 "이 목표 명령이 어떻게 끝났는가"를 추적하기 위한 구조다.

| 상태 | 의미 |
|---|---|
| `CMD_IDLE` | 명령 없음 |
| `CMD_ACTIVE` | 목표 위치로 이동 중 |
| `CMD_REACHED` | 목표 도달 |
| `CMD_TIMEOUT` | timeout으로 실패 |
| `CMD_ABORTED` | disable, estop, 새 명령으로 중단 |
| `CMD_FAULTED` | safety fault로 종료 |

| 결과 | 의미 |
|---|---|
| `CMD_RESULT_REACHED` | 정상 도달 |
| `CMD_RESULT_TIMEOUT` | 시간 초과 |
| `CMD_RESULT_ESTOP` | emergency stop |
| `CMD_RESULT_DISABLED` | 제어 비활성화 |
| `CMD_RESULT_REPLACED` | 새 명령으로 기존 명령 대체 |
| `CMD_RESULT_FAULT_LIMIT` | 위치 한계 fault |
| `CMD_RESULT_FAULT_TRACKING` | tracking error fault |
| `CMD_RESULT_FAULT_VELOCITY` | 속도 fault |

### 6.3 `position_control_safety.c`

이 파일은 안전 조건을 평가한다. 중요한 점은 이 모듈이 직접 모터를 멈추지 않는다는 것이다. 대신 안전 결과만 반환하고, 실제 정지 동작은 `position_control.c`에서 수행한다.

#### 평가 조건

| 조건 | 동작 |
|---|---|
| `current_angle`이 소프트 한계 초과 | `POS_CTRL_ERR_OVER_LIMIT` |
| tracking error가 허용값 초과 | `POS_CTRL_ERR_SAFETY` |
| measured velocity가 허용값 초과 | `POS_CTRL_ERR_VELOCITY` |
| 모두 정상 | `is_safe = true` |

#### 주요 함수

| 함수 | 역할 |
|---|---|
| `PositionControlSafety_Init()` | safety limit 초기화 |
| `PositionControlSafety_SetLimits()` | safety limit 갱신 |
| `PositionControlSafety_GetLimits()` | 현재 safety limit snapshot 반환 |
| `PositionControlSafety_Evaluate()` | 현재 angle/error/velocity를 평가해 fault 여부 반환 |

### 6.4 `encoder_reader.c`

`encoder_reader.c`는 TIM2 하드웨어 카운터를 읽어 누적 count와 각도로 바꾼다.

#### 처리 흐름

```text
TIM2 raw counter
  -> prev_raw_count와 차이 계산
  -> int16_t delta로 16-bit wrap 보정
  -> accum_count에 누적
  -> offset_count 보정
  -> motor_deg = count x ENCODER_DEG_PER_COUNT
```

#### 왜 누적 count가 필요한가

TIM2 카운터는 하드웨어 카운터이므로 일정 범위를 넘어가면 wrap된다. 따라서 단순 raw counter만 보면 장시간 회전에서 실제 총 이동량을 알기 어렵다. 이 코드는 매번 raw 차이를 `int16_t` delta로 해석해 누적 count를 만든다.

#### 주요 함수

| 함수 | 역할 | 반환/효과 |
|---|---|---|
| `EncoderReader_Init()` | TIM2 카운터를 center로 맞추고 상태 초기화 | initialized = 1 |
| `EncoderReader_GetAngleDeg()` | 현재 motor_deg 반환 | `GetMotorDeg()`와 동일 |
| `EncoderReader_GetMotorDeg()` | 누적 count를 motor_deg로 변환 | PID current angle |
| `EncoderReader_GetCount()` | offset 보정된 누적 count 반환 | CSV/진단 |
| `EncoderReader_GetDeltaCount()` | 최근 delta count 반환 | 센서 움직임 진단 |
| `EncoderReader_GetSample()` | raw/delta/accum/motor_deg/age를 한 번에 반환 | health check 확장용 |
| `EncoderReader_GetRawCounter()` | TIM2 raw counter 반환 | `[ENCDBG]`, CSV |
| `EncoderReader_Reset()` | 현재 위치를 논리 0으로 재설정 | 부팅/bench reset |
| `EncoderReader_SetOffset()` | 외부 기준에 맞춰 offset 설정 | homing 연동 |
| `EncoderReader_EnableVirtualFeedback()` | 실제 엔코더 대신 virtual feedback 사용 여부 | 벤치 fallback |
| `EncoderReader_SetVirtualFeedbackCount()` | virtual count 수동 설정 | pulse 적분 표시 |
| `EncoderReader_IsInitialized()` | 초기화 여부 반환 | command start gate |

### 6.5 `pulse_control.c`

`pulse_control.c`는 PID 출력값을 실제 서보드라이버 입력 신호로 바꾼다.

#### 입력과 출력

| 입력 | 의미 |
|---|---|
| 양수 `freq_hz` | CW 방향으로 해당 주파수 펄스 출력 |
| 음수 `freq_hz` | CCW 방향으로 해당 주파수 펄스 출력 |
| `0` | 펄스 정지 |

| 출력 | 핀 |
|---|---|
| Pulse PWM | `PE9 = TIM1_CH1` |
| Direction GPIO | `PE10` |
| Line driver enable | `LINE_DRIVER_DE`, `LINE_DRIVER_REN` |

#### 처리 흐름

```text
PulseControl_SetFrequency(freq_hz)
  -> line driver enable
  -> freq_hz == 0이면 PWM stop
  -> 부호로 direction 결정
  -> 주파수 min/max clamp
  -> 방향이 바뀌면 reverse guard 실행
  -> TIM1 ARR/CCR 계산
  -> HAL_TIM_PWM_Start()
```

방향이 바뀔 때는 바로 반대 방향 펄스를 내지 않고, `PULSECONTROL_DIRECTION_GUARD_MS`만큼 정지/settle 시간을 둔다. 이는 방향 전환 중 드라이버 입력이 불안정해지는 것을 줄이기 위한 보호 장치다.

#### 주요 함수

| 함수 | 역할 |
|---|---|
| `PulseControl_Init()` | TIM1 handle, 상태 변수, line driver, 초기 방향 설정 |
| `PulseControl_SetFrequency()` | signed frequency를 pulse/direction 출력으로 변환 |
| `PulseControl_Stop()` | PWM 정지, 상태 clear |
| `PulseControl_SendSteps()` | 정해진 step 수만 출력하는 step mode |
| `pulse_forward()` | `SendSteps(..., DIR_CW)` wrapper |
| `pulse_reverse()` | `SendSteps(..., DIR_CCW)` wrapper |
| `PulseControl_IsBusy()` | step mode 동작 중인지 반환 |
| `PulseControl_GetStatus()` | requested/applied Hz, ARR/CCR, direction, active 상태 반환 |
| `HAL_TIM_PWM_PulseFinishedCallback()` | step mode에서 펄스 수 감소, 완료 시 stop |

### 6.6 `relay_control.c`

`relay_control.c`는 서보 ON/OFF와 emergency GPIO를 담당한다.

| 함수 | 역할 |
|---|---|
| `Relay_Init()` | relay GPIO 초기 상태 설정 |
| `Relay_ServoOn()` | servo on 신호 출력 |
| `Relay_ServoOff()` | servo off 신호 출력 |
| `Relay_Emergency()` | emergency stop 신호 출력 |
| `Relay_EmergencyRelease()` | emergency 상태 해제 |

주의할 점은 현재 문서 기준으로 relay GPIO 제어가 실제 드라이브 안전 동작까지 완전히 검증된 것은 아니라는 점이다. 실제 현업 수준에서는 relay truth table, 드라이브 파라미터, 물리 배선, scope/drive monitor 증거가 함께 필요하다.

### 6.7 `ethernet_communication.c`

이 파일은 UDP 기반 상위 제어기 연동을 담당한다. 현재 기본 빌드는 keyboard지만, UDP 경로는 코드에 존재한다.

#### 처리하는 패킷

| 패킷 | 크기 | 송신자 | 의미 |
|---|---:|---|---|
| ASMS | 5 byte | IP 마지막 옥텟 `.5` | mode + joystick |
| PC | 9 byte | IP 마지막 옥텟 `.1` | steering/speed/misc |

#### UDP 처리 흐름

```text
MX_LWIP_Process()
  -> LwIP가 UDP 수신
  -> udp_recv_cb()
  -> sender/length 필터
  -> mode, steering_angle, emergency flag 저장
  -> AppRuntime_ServiceUdpComms()
  -> PositionControl_SetTargetWithSource()
```

#### 주요 함수

| 함수 | 역할 |
|---|---|
| `EthComm_UDP_Init()` | UDP PCB 생성, port 5000 bind, callback 등록 |
| `udp_recv_cb()` | LwIP UDP 수신 callback, packet parsing |
| `EthComm_HasNewData()` | 새 steering packet 여부 반환 |
| `EthComm_GetLatestData()` | 최신 packet 반환 후 new flag clear |
| `EthComm_GetCurrentMode()` | NONE/AUTO/MANUAL/ESTOP 반환 |
| `EthComm_ConsumeEmergencyRequest()` | emergency 요청 소비 |
| `EthComm_GetLastRxTick()` | 마지막 수신 시각 반환 |
| `EthComm_ForceMode()` | timeout 등으로 mode 강제 변경 |

#### Legacy text API

`EthComm_Init()`, `EthComm_Update()`, `EthComm_HandleLine()` 등은 legacy text command API다. 현재 주 런타임 경로에서는 사용하지 않고, 벤치/디버깅 호환 목적으로 남아 있다.

### 6.8 `can_runtime.c`

CAN 런타임은 CubeMX가 만든 `can.c/h` 위에 앱 계층 helper로 얹혀 있다. CubeMX 재생성으로 command path가 덮이지 않게 runtime helper를 분리한 구조다.

#### 주요 함수

| 함수 | 역할 |
|---|---|
| `CAN_Runtime_Init()` | CAN 설정, filter 설정, 상태 초기화 |
| `CAN_Runtime_Start()` | CAN peripheral start |
| `CAN_Runtime_Stop()` | CAN stop |
| `CAN_Runtime_Service()` | RX FIFO drain, error polling |
| `CAN_Runtime_SendStd()` | standard ID frame 송신 |
| `CAN_Runtime_Pop()` | RX queue에서 frame 하나 꺼냄 |
| `CAN_Runtime_GetStatus()` | CAN runtime status 반환 |
| `CAN_Runtime_PrintStatus()` | CAN 상태 UART 출력 |

`app_runtime.c`는 CAN frame ID에 따라 아래 명령을 처리한다.

| CAN ID | 의미 |
|---|---|
| `APP_RUNTIME_CAN_CMD_STEER_STDID` | signed int16, 0.1 deg 단위 steering target |
| `APP_RUNTIME_CAN_CMD_CONTROL_STDID` | disable/enable/estop/center/status |
| `APP_RUNTIME_CAN_QUERY_STDID` | status 요청 |
| `APP_RUNTIME_CAN_STATUS_STDID` | current/target/error/mode/cmd 상태 응답 |

### 6.9 `position_control_diag.c`

진단 모듈은 사람이 읽기 좋은 문자열과 외부 디버그 변수를 만든다.

#### 주요 함수

| 함수 | 역할 |
|---|---|
| `PositionControlDiag_CommandSourceString()` | command source enum을 문자열로 변환 |
| `PositionControlDiag_CommandResultString()` | command result enum을 문자열로 변환 |
| `PositionControlDiag_CommandStateString()` | command state enum을 문자열로 변환 |
| `PositionControlDiag_UpdateDebugVars()` | `dbg_*` 전역 디버그 변수 갱신 |
| `PositionControlDiag_PrintStateSummary()` | `[PosCtrl]` 상태 요약 출력 |
| `PositionControlDiag_ErrorString()` | error code 문자열 변환 |
| `PositionControl_GetErrorString()` | legacy API 유지용 wrapper |

### 6.10 `latency_profiler.c`

latency profiler는 제어 루프를 단계별로 나눠 실행 시간을 기록한다.

| Stage | 의미 |
|---|---|
| `LAT_STAGE_SENSE` | 센서 읽기, dt/error 계산 |
| `LAT_STAGE_CONTROL` | timeout, safety, PID 계산 |
| `LAT_STAGE_ACTUATE` | pulse output 적용 |
| `LAT_STAGE_COMMS` | keyboard/UDP/CAN 처리 |

주요 API는 다음과 같다.

| 함수 | 역할 |
|---|---|
| `LatencyProfiler_Init()` | core clock 기준으로 profiler 초기화 |
| `LatencyProfiler_Reset()` | sample reset |
| `LatencyProfiler_Begin()` | stage 시작 timestamp 저장 |
| `LatencyProfiler_End()` | stage 종료, cycle 저장 |
| `LatencyProfiler_OnDeadlineTick()` | deadline miss 카운트 |
| `LatencyProfiler_GetStageStats()` | avg/p99/max 계산 |
| `LatencyProfiler_StageName()` | stage 이름 문자열 |

## 7. 로그와 output은 어떻게 나타나는가

### 7.1 CSV 로그

CSV 로그는 주기적으로 controller 상태를 한 줄에 남긴다.

```text
CSV_HEADER,ms,mode,target_deg,current_deg,error_deg,output,dir,enc_cnt,enc_raw,req_hz,applied_hz,out_active,rev_guard,cmd_id,cmd_state,cmd_result
CSV,371276,1,20.000,19.987,0.013,0,0,33311,543,0,0,0,0,99,2,1
```

각 필드의 의미는 아래와 같다.

| 필드 | 의미 |
|---|---|
| `ms` | 부팅 후 시간 |
| `mode` | position control mode |
| `target_deg` | 목표 조향각, steering_deg |
| `current_deg` | 현재 조향각, steering_deg |
| `error_deg` | 목표와 현재의 차이 |
| `output` | PID 출력, pulse_hz 요청값 |
| `dir` | direction GPIO 상태 |
| `enc_cnt` | 엔코더 누적 count |
| `enc_raw` | TIM2 raw counter |
| `req_hz` | 요청된 pulse frequency |
| `applied_hz` | 실제 적용된 pulse frequency |
| `out_active` | PWM 출력 활성 여부 |
| `rev_guard` | 방향 전환 guard 동작 여부 |
| `cmd_id` | command lifecycle ID |
| `cmd_state` | command state enum |
| `cmd_result` | command result enum |

### 7.2 Encoder debug 로그

`[ENCDBG]`는 실제 TIM2 하드웨어 상태를 확인하기 위한 로그다.

```text
[ENCDBG] ms=... cnt=... prev=... delta=... A=... B=... CEN=... SMS=... CC1S=... CC2S=... CC1E=... CC2E=...
```

이 로그를 보면 다음을 확인할 수 있다.

- TIM2 counter가 실제로 변하는지
- A/B 입력 핀이 변하는지
- TIM2 encoder mode 설정이 살아 있는지
- counter delta 방향과 pulse direction이 맞는지

### 7.3 Command lifecycle 로그

명령 단위 추적 로그는 아래처럼 나온다.

```text
CMD_START,id=97,src=KEYBOARD,target_deg=40.000,target_motor_deg=500.000,start_ms=308365,start_deg=5.915,start_error_deg=34.085
CMD_REACHED,id=97,end_ms=324453,settling_ms=16088,final_deg=39.989,final_error_deg=0.011
```

이 예시는 40 deg 목표 명령이 약 16.088 s 후 목표에 도달했고, 최종 오차가 0.011 deg였다는 뜻이다.

### 7.4 Latency 로그

latency batch는 제어 루프 단계별 실행 시간 통계를 보여준다.

```text
LATENCY_BATCH_BEGIN,seq=32,samples=2000,core_hz=180000000,deadline_miss=...
LATENCY_STAGE,seq=32,name=Sense,count=2000,avg_us=...,p99_us=...,max_us=...
LATENCY_STAGE,seq=32,name=Control,count=2000,avg_us=...,p99_us=...,max_us=...
LATENCY_STAGE,seq=32,name=Actuate,count=2000,avg_us=...,p99_us=...,max_us=...
LATENCY_BATCH_END,seq=32
```

주의할 점은 UART `printf()`가 많으면 strict real-time timing에 영향을 줄 수 있다는 것이다. 따라서 로그가 켜진 bring-up 모드와 최종 timing 검증 모드를 분리해서 봐야 한다.

## 8. 현재 상태

현재 프로젝트 상태는 다음처럼 정리할 수 있다.

### 8.1 구현된 것

- STM32F429ZI 기반 프로젝트 구조 구축
- `main.c`를 얇은 부트 엔트리로 정리
- `AppRuntime` 기반 super-loop 구조 구성
- keyboard bench 입력 경로 구현
- UDP packet parsing 경로 구현
- CAN runtime helper와 command/status frame 경로 구현
- 1 ms position control update 구조 구현
- PID 기반 위치 제어 구현
- command lifecycle 구현
- TIM2 real encoder path 구현
- optional virtual feedback path 구현
- TIM1 pulse output + direction GPIO 구현
- reverse direction guard 구현
- relay servo on/off, emergency path 구현
- safety evaluator 분리
- CSV, `[ENCDBG]`, lifecycle, latency log 구현
- PuTTY live viewer와 steering portal 문서화 구조 보유

### 8.2 실험상 확인된 것

최근 bench 기준으로 다음을 확인했다.

- keyboard bench에서 목표각 입력 시 target/current/error/output이 갱신된다.
- `requested_hz`, `applied_hz`, direction, reverse guard 상태를 로그로 볼 수 있다.
- `-20 / 20 / 30 / 40 deg` 목표각에서 real TIM2 기준 목표 근처 정착이 확인됐다.
- 정착 후 steady-state error는 약 `0.02 deg` 이내 수준으로 보인다.
- 20/30/40 deg에서 encoder count가 선형적으로 증가해 스케일 일관성이 좋아졌다.
- command lifecycle 로그로 `CMD_START`, `CMD_REACHED`, `CMD_ABORT`, `CMD_TIMEOUT` 흐름을 추적할 수 있다.

### 8.3 현재 기본 설정

`Core/Inc/project_params.h` 기준 현재 기본은 아래와 같다.

| 설정 | 현재값 | 의미 |
|---|---|---|
| `APP_RUNTIME_INPUT_SOURCE` | `KEYBOARD` | UDP가 아니라 keyboard bench |
| `APP_RUNTIME_ENCODER_DIAG_ENABLE` | `1` | `[ENCDBG]` 출력 |
| `APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE` | `1` | CSV 주기 출력 |
| `APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE` | `0` | 실제 TIM2 기준 |
| `APP_RUNTIME_AUTO_START_CONTROL_ENABLE` | `1` | 부팅 후 자동 enable |
| `POSITION_FAILSAFE_PROFILE` | `PARAM_TEST` | 튜닝 중 nuisance trip 완화 |
| `DEFAULT_OUTPUT_LIMIT` | `50000` | PID 출력 상한 |
| `PULSECONTROL_MAX_FREQ_HZ` | `100000` | 실제 pulse output clamp |

## 9. 미흡한 점과 보완 방향

### 9.1 Startup safety가 아직 약하다

현재는 `APP_RUNTIME_AUTO_START_CONTROL_ENABLE = 1`이므로 부팅 후 자동으로 control enable이 수행된다. 실제 조향 시스템에서는 전원 투입 직후 바로 enable되는 구조가 위험할 수 있다.

보완 방향:

- `INIT -> SELF_TEST -> HOMING -> READY -> ARMED -> RUN -> ESTOP_LATCH` 상태기계 도입
- homing 또는 sensor validation 전에는 `Relay_ServoOn()`과 `PositionControl_Enable()` 금지
- operator arm command 또는 상위 제어기 arm command가 있어야 RUN 진입
- ESTOP 이후에는 명확한 clear 절차 없이는 재시작 금지

### 9.2 Real encoder truth가 더 닫혀야 한다

최근 로그에서 TIM2 엔코더 정착과 count scaling은 좋아졌지만, 현업 기준으로는 아직 더 강한 증거가 필요하다.

보완 방향:

- TIM2 `[ENCDBG]`와 oscilloscope/logic analyzer waveform을 같은 run ID로 묶기
- commanded direction과 encoder delta 부호가 맞는지 자동 검사
- 일정 시간 pulse가 나가는데 encoder delta가 없으면 stale sensor fault 발생
- encoder와 ADC potentiometer cross-check 추가
- long-run wrap 테스트로 누적 count 안정성 검증

### 9.3 응답 속도와 motion profile이 부족하다

현재 큰 각도 변화에서 settling time이 약 14-16 s 수준으로 길다. 최종 위치 정확도는 좋아졌지만, 실제 조향 응답으로는 느릴 수 있다.

보완 방향:

- PID gain 재튜닝
- output limit과 pulse clamp의 관계 정리
- trapezoidal 또는 S-curve motion profile 도입
- command replacement가 너무 자주 일어나지 않도록 command pacing 정책 정의
- 목표각 step test를 표준화해 settling time, overshoot, steady-state error를 자동 계산

### 9.4 Safety fault taxonomy가 더 필요하다

현재 fault는 angle/tracking/velocity/timeout 중심이다. 실제 시스템에서는 fault reason을 더 세분화해야 한다.

보완 방향:

- `FAULT_SENSOR_STALE`
- `FAULT_WRONG_DIRECTION`
- `FAULT_ENCODER_IMPLAUSIBLE`
- `FAULT_DRIVE_NOT_READY`
- `FAULT_COMM_TIMEOUT`
- `FAULT_STARTUP_INHIBIT`
- `FAULT_ESTOP_LATCHED`

각 fault에는 아래 정보를 남기는 것이 좋다.

- 발생 시각
- fault source
- 관련 command id
- target/current/error
- pulse requested/applied
- encoder delta
- clear 가능 여부
- 재시작 조건

### 9.5 Logging이 real-time loop를 방해할 수 있다

현재는 UART `printf()` 기반 로그가 많다. bring-up에는 매우 유용하지만, 엄격한 1 ms 제어 검증에는 영향을 줄 수 있다.

보완 방향:

- DMA UART + ring buffer 도입
- high-rate raw log와 low-rate human log 분리
- compile-time logging profile 분리
- strict timing test에서는 blocking printf 제거
- run ID 기반으로 raw log, plot, commit hash, 설정값을 자동 묶기

### 9.6 Parameter ownership 정리가 더 필요하다

`project_params.h`가 생겼지만, 모든 운영 파라미터가 완전히 하나의 체계로 정리된 것은 아니다.

보완 방향:

- 변환 상수는 `constants.h`
- 운영/시험 스위치는 `project_params.h`
- 모듈 내부 private 상수는 각 `.c`
- 차량 시험용 profile과 bench tuning profile 분리
- 변경 시 `Doc/change_code/YYYY-MM-DD.md`에 이유 기록

## 10. 현업 관점에서 추가하면 좋은 것

### 10.1 명확한 요구사항 문서

현업에서는 "잘 움직인다"보다 "어떤 요구사항을 만족한다"가 중요하다.

추가하면 좋은 요구사항 예시는 다음과 같다.

| 요구사항 | 예시 |
|---|---|
| 제어 주기 | position loop는 1 ms 주기로 실행되어야 한다 |
| 목표 범위 | steering target은 -45 deg에서 +45 deg 사이여야 한다 |
| 정착 오차 | steady-state error는 특정 조건에서 0.5 deg 이하여야 한다 |
| 통신 timeout | UDP/CAN command가 300 ms 이상 없으면 safe state로 전환해야 한다 |
| ESTOP | ESTOP 발생 시 pulse output은 즉시 stop 되어야 한다 |
| startup | homing 완료 전에는 actuator enable이 금지되어야 한다 |

### 10.2 상태기계 문서와 코드 일치

추천 상태기계:

```text
POWER_ON
  -> INIT
  -> SELF_TEST
  -> HOMING
  -> READY
  -> ARMED
  -> RUN
  -> ESTOP_LATCH
  -> FAULT_LATCH
```

각 상태마다 허용되는 입력과 금지되는 출력이 명확해야 한다.

### 10.3 테스트 매트릭스

현업에서는 시험 케이스를 표로 관리한다.

| Test ID | 조건 | 기대 결과 | 증거 |
|---|---|---|---|
| T-001 | target 0 -> 20 deg | 목표 도달, error < 0.5 deg | CSV + CMD_REACHED |
| T-002 | target 20 -> -20 deg | 방향 전환 guard 후 도달 | CSV + scope |
| T-003 | encoder unplug | stale fault | `[ENCDBG]` + fault log |
| T-004 | UDP timeout | ESTOP 또는 disable | mode log |
| T-005 | ESTOP command | pulse stop | GPIO/scope |

### 10.4 Evidence pack 자동화

실험 하나를 아래 파일 묶음으로 남기면 좋다.

```text
run_2026-04-27_001/
  config.md
  git_sha.txt
  raw_putty.log
  telemetry.csv
  latency.csv
  command_lifecycle.csv
  encoder_dbg.log
  scope_capture.png
  summary.md
```

### 10.5 Fault injection

실제 제어기는 정상 입력보다 비정상 입력에 강해야 한다.

추천 fault injection:

- encoder delta 고정
- encoder direction 반전
- pulse output 강제 0
- UDP packet drop
- CAN bus-off
- target out-of-range
- watchdog refresh 중단
- ESTOP 반복 입력

### 10.6 Calibration / homing 절차

현재 homing skeleton은 있지만 운영 계약으로 닫히지 않았다. 현업 수준으로는 아래가 필요하다.

- ADC potentiometer raw min/max calibration
- encoder zero와 absolute sensor 기준 동기화
- homing 실패 시 actuator enable 금지
- homing 결과와 offset 기록
- 재부팅 후 offset 재검증

### 10.7 Safety case

조향 시스템은 안전 중요도가 높다. 최소한 아래 safety case를 문서화하는 것이 좋다.

- 어떤 고장을 감지할 수 있는가
- 어떤 고장은 감지하지 못하는가
- fault 발생 시 출력은 어떻게 차단되는가
- fault clear는 누가, 어떤 조건에서 할 수 있는가
- 상위 제어기와 하위 제어기 중 어느 쪽이 최종 authority를 갖는가

## 11. 처음 보는 사람을 위한 읽는 순서

처음 보는 사람은 아래 순서로 보면 가장 빠르다.

1. `README.md`
   - 현재 프로젝트 요약과 상태 파악

2. `Doc/system_beginner_guide.md`
   - 목표, 구조, 데이터 흐름, 함수 역할 전체 이해

3. `Core/Inc/constants.h`
   - 단위 변환과 gear ratio 이해

4. `Core/Inc/project_params.h`
   - 현재 bench/UDP/CAN/safety/log 설정 확인

5. `Core/Src/main.c`
   - 부팅과 runtime 호출 구조 확인

6. `Core/Src/app_runtime.c`
   - 입력 처리, fast tick, 로그 출력 흐름 확인

7. `Core/Src/position_control.c`
   - PID, lifecycle, safety 적용 흐름 확인

8. `Core/Src/encoder_reader.c`
   - TIM2 raw count가 각도로 바뀌는 과정 확인

9. `Core/Src/pulse_control.c`
   - pulse/direction 출력 생성 확인

10. `Doc/steering_portal/index.html`
    - 시각화된 구조와 현재 평가 확인

11. `Doc/putty/index.html`
    - 실제 로그 분석 방식 확인

## 12. 발표나 리뷰에서 말하면 좋은 요약

이 프로젝트는 STM32F429ZI 기반의 조향 서브컨트롤러다. 상위 제어기에서 받은 `steering_deg` 명령을 내부 `motor_deg` 목표로 변환하고, TIM2 엔코더 피드백으로 현재 위치를 읽어 1 ms PID 위치 제어를 수행한다. PID 출력은 signed `pulse_hz`로 변환되어 TIM1 pulse와 GPIO direction 신호로 서보드라이버에 전달된다.

현재는 keyboard bench와 real TIM2 encoder diagnostic을 기본으로 사용하며, CSV, `[ENCDBG]`, command lifecycle, latency batch 로그를 통해 실험 상태를 추적할 수 있다. 최근 로그 기준으로 여러 목표각에서 최종 정착각은 꽤 정확하지만, 큰 각도 변화의 settling time, startup safety, homing, fault latch, sensor stale detection은 아직 보완해야 한다.

현업 관점에서 이 프로젝트의 강점은 단순 구동 데모가 아니라, 입력 처리, 단위 변환, 위치 제어, actuator 출력, 센서 피드백, safety, evidence logging을 모듈별로 분리했다는 점이다. 다음 단계는 startup state machine과 fault policy를 닫고, real encoder/actuator evidence pack을 scope와 drive monitor까지 포함해 완성하는 것이다.
