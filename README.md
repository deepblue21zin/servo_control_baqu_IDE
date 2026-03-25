# Autonomous Steering Servo Control System

STM32F429ZI 기반 조향 서브컨트롤러 프로젝트다. 상위 명령을 `steering_deg` 기준으로 받아 내부에서 `motor_deg`로 변환하고, 1ms 폐루프 제어로 pulse/direction 출력과 엔코더 피드백을 묶는다.

## 1. Current Snapshot

| 항목 | 현재 상태 |
|---|---|
| MCU | STM32F429ZI, 180 MHz |
| Servo Driver | LS ELECTRIC XDL-L7SA004BAA |
| Motor / Encoder | XML-FBL04AMK1, 12000 PPR, quadrature x4 = 48000 count/rev |
| Loop | SysTick 기반 1 ms 제어 루프 |
| External Unit | `steering_deg` |
| Internal Unit | `motor_deg`, `enc_count`, `pulse_hz` |
| Pulse Output | `PE9 = TIM1_CH1`, `PE10 = direction GPIO` |
| Encoder Input | `PD12 = TIM4_CH1`, `PD13 = TIM4_CH2` |
| Network | LwIP UDP 구현 존재, 현재 bench 기본값은 `KEYBOARD_TEST_MODE = 1` |
| Watchdog | IWDG 사용, 현재 설정 기준 약 0.5 s |
| Runtime Trace | CSV, command lifecycle event, latency batch, keyboard snapshot |

## 2. What Is Implemented Now

### 2.1 Control Runtime

- `main.c`에서 peripheral init, 1 ms scheduler, keyboard bench, CSV logging, watchdog refresh를 수행한다.
- `position_control.c`는 PID, safety check, emergency path, command lifecycle을 담당한다.
- `pulse_control.c`는 signed `pulse_hz`를 받아 `PE9` 펄스와 `PE10` 방향으로 변환한다.
- `encoder_reader.c`는 TIM4 raw counter를 읽고 누적 `unwrap` count로 각도를 계산한다.

### 2.2 Command Lifecycle

현재 runtime은 homing 없이도 명령 단위를 추적한다.

- source: `UDP`, `KEYBOARD`, `SERVICE`, `LOCALTEST`
- state: `CMD_IDLE`, `CMD_ACTIVE`, `CMD_REACHED`, `CMD_TIMEOUT`, `CMD_ABORTED`, `CMD_FAULTED`
- result: `REACHED`, `TIMEOUT`, `ESTOP`, `DISABLED`, `REPLACED`, `FAULT_LIMIT`, `FAULT_TRACKING`
- event log: `CMD_START`, `CMD_REACHED`, `CMD_TIMEOUT`, `CMD_ABORT`, `CMD_FAULT`

현재 정책:

- target 수락 시 `command_id` 발급
- `abs(error_steering_deg) <= 0.5 deg`가 100 ms 유지되면 `CMD_REACHED`
- `CMD_REACHED` 직후 `PulseControl_Stop()`으로 출력을 정지
- 진행 중 새 명령이 들어오면 기존 명령은 `CMD_ABORT(reason=REPLACED)`로 종료

### 2.3 Pulse Contract

현재 `pulse_control.c` 기준 계약은 아래와 같다.

- 입력: signed `freq_hz`
- 출력 범위 clamp: `10 Hz ~ 100000 Hz`
- direction polarity: `DIR_ACTIVE_HIGH_FOR_CW`
- reverse guard: 방향 반전 시 `stop -> wait 1 ms -> dir change -> wait 1 ms -> restart`
- status API: `requested_frequency_hz`, `applied_frequency_hz`, `ARR`, `CCR`, `direction`, `output_active`, `reverse_guard_active`

주의:

- `constants.h`에는 `MAX_PULSE_FREQ = 1000000`이 남아 있지만, 현재 실제 펌웨어 clamp는 `100000 Hz`다.
- 즉 문서와 코드 기준으로는 "설계 상수 1 MHz"보다 "현재 런타임 계약 100 kHz"를 먼저 봐야 한다.

### 2.4 Encoder Contract

현재 `encoder_reader.c`는 centered raw를 직접 각도로 쓰지 않고, raw delta를 누적하는 방식으로 바뀌었다.

- TIM4 counter 중심값: `32768`
- raw read: `__HAL_TIM_GET_COUNTER(&htim4)`
- update rule: `delta = (int16_t)(raw - last_raw)`
- exported count: 누적 `unwrap` count - `offset`
- exported angle: `count * (360 / 48000)`

즉 현재 `GetCount()`는 raw center 기준값이 아니라 누적 count를 반환한다.

## 3. Current Bench State

2026-03-24 기준 최근 bring-up에서 확인한 상태는 아래와 같다.

| 항목 | 현재 관찰 |
|---|---|
| `PE9` | command 시 펄스 출력 확인 |
| `PE10` | direction GPIO로 사용, 펄스가 아니라 High/Low 상태 변화가 정상 |
| TIM4 encoder | 손으로 축을 돌릴 때 `CNT` 변화 확인 |
| Drive 상태 | `P-RUN` 확인 |
| Drive parameter | `P4-00 = 2` 확인 |
| Drive monitor | `St-06 = 0`, `St-04` 정지 상태 확인 |

현재 해석:

- MCU 내부 pulse output과 encoder feedback path는 각각 일부 동작 근거가 있다.
- 하지만 드라이브가 현재 command pulse를 내부 명령으로 카운트하지 못하는 상태가 가장 큰 현안이다.
- 따라서 "closed-loop steering validated"라고 부르기엔 아직 bench closure가 부족하다.

## 4. Current Runtime Behavior

### 4.1 Default Bring-up Path

현재 `main.c` 기본 경로는 bench 편의 중심이다.

- `KEYBOARD_TEST_MODE = 1`
- `PERIODIC_CSV_LOG_ENABLE = 1`
- `ENCODER_RUNTIME_DIAG_ENABLE = 0`
- startup 시 `Relay_ServoOn()` 이후 `EncoderReader_Reset()`
- 이어서 `PositionControl_SetTargetWithSource(..., CMD_SRC_LOCALTEST)`와 `PositionControl_Enable()` 수행

즉 현재는 startup state machine 없이 enable까지 자동 진행된다.

### 4.2 CSV / Snapshot Format

현재 주기 CSV 헤더는 다음과 같다.

```text
CSV_HEADER,ms,mode,target_deg,current_deg,error_deg,output,dir,enc_cnt,enc_raw,req_hz,applied_hz,out_active,rev_guard,cmd_id,cmd_state,cmd_result
```

keyboard snapshot과 periodic DIAG에도 같은 축의 정보를 반영한다.

### 4.3 Latency Evidence

`latency_profiler.c` 기반 DWT 측정 경로는 유지되고 있다.

- `Sense`, `Control`, `Actuate`, `Comms`
- `avg`, `p99`, `max`
- `LATENCY_BATCH_BEGIN`, `LATENCY_STAGE`, `LATENCY_BATCH_END`

다만 엄격한 timing 검증 시에는 UART 로그 부하를 최소화해야 한다.

## 5. Current Risks And Gaps

### 5.1 Bench Closure

- 드라이브 `St-06 = 0` 상태라 command pulse 인식이 아직 닫히지 않았다.
- `P-RUN`, `P4-00 = 2`, `PE9` 파형만으로는 실제 motion proof가 되지 않는다.

### 5.2 Startup Safety

- boot-time auto enable이 남아 있다.
- `homing.c`와 `relay_control.c`는 존재하지만 실제 readiness gate로 완전히 통합되지 않았다.

### 5.3 Fault Policy

- lifecycle은 들어갔지만 latched fault / clear policy는 아직 없다.
- stale sensor, implausible motion, wrong-direction 같은 richer fault taxonomy가 더 필요하다.

### 5.4 Logging Path

- 현재 UART 로그는 blocking path라 제어 주기와 관찰값에 영향을 줄 수 있다.
- async UART DMA + ring buffer 정리가 남아 있다.

## 6. Important Source Files

| 파일 | 역할 |
|---|---|
| `Core/Src/main.c` | init, 1 ms scheduler, keyboard bench, CSV, watchdog |
| `Core/Src/position_control.c` | PID, safety, lifecycle, ESTOP |
| `Core/Src/pulse_control.c` | pulse/direction output, reverse guard, applied Hz status |
| `Core/Src/encoder_reader.c` | TIM4 raw -> unwrap count -> angle |
| `Core/Src/ethernet_communication.c` | UDP packet / mode handling |
| `Core/Src/homing.c` | ADC 기반 zero reference skeleton |
| `Core/Src/relay_control.c` | SVON / EMG relay control |

## 7. Documentation Index

| 문서 | 설명 |
|---|---|
| `Doc/README.md` | 문서 인덱스와 현재 runtime 요약 |
| `Doc/command_lifecycle_no_homing_spec.md` | no-homing lifecycle 명세와 현재 구현 상태 |
| `Doc/steering_portal/index.html` | 현재 구현, REQ, evidence를 시각화한 로컬 포털 |
| `Doc/change_code/2026-03-24.md` | 오늘 코드/문서 변경 이력 |
| `Doc/hardware_pinmap.md` | 실제 핀 매핑 참고 |
| `Doc/latency_*.md` 문서군 | latency 측정 계약과 evidence 관리 |

## 8. One-Line Summary

현재 프로젝트는 "1 ms steering sub-controller의 구조, command lifecycle, pulse status, encoder unwrap, traceability"까지는 잘 정리된 상태이고, 다음 핵심 과제는 "드라이브가 command pulse를 실제 motion으로 받아들이는 bench closure"다.

Last updated: 2026-03-24
