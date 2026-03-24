# Autonomous Steering Sub-Controller (STM32F429ZI)

STM32F429ZI 기반 조향 서브 컨트롤러 프로젝트.
상위 인지/제어 시스템의 UDP 조향 명령을 받아 1ms 주기의 폐루프 제어(PID)로 모터를 구동한다.

## 1. Executive Summary

### 1.1 What This Project Solves

- 상위 시스템과 하위 액추에이터 사이를 연결하는 실시간 조향 제어기 구현
- 통신/제어/안전 기능을 단일 펌웨어에서 통합
- "동작" 수준이 아니라 p99/worst-case를 수치로 검증 가능한 구조 확보

### 1.2 Key Specs (Current Runtime)

- MCU: STM32F429ZI (180MHz)
- Control Period: 1ms (1kHz)
- Control: Encoder feedback + PID + Pulse/Direction
- Command lifecycle: `CMD_START / REACHED / TIMEOUT / ABORT / FAULT`
- Network: Ethernet UDP (LwIP)
- Safety: ESTOP, RX timeout fail-safe, safety limit check, IWDG watchdog
- Latency Metrics: avg/p99/max + deadline miss

### 1.3 Why This Is Portfolio-Ready

- 실시간 제어 + 네트워크 + 안전 경로를 함께 다룸
- DWT 기반 성능 수집 자동화(2000샘플 배치)
- 측정 데이터/메타데이터/변경 이력을 문서 체계로 관리

---

## 2. System Context

```text
Upper Controller (PC/Jetson)
        |
        | UDP packet (target steer / mode / misc)
        v
STM32 Steering Sub-Controller
  - Comms (LwIP polling + packet parse)
  - Mode handling (AUTO/MANUAL/ESTOP)
  - 1ms control loop scheduler
  - PID position control
  - Safety checks + emergency path
        |
        +--> Pulse/Direction output --> Servo Driver --> Motor + Gear (12:1)
        |
        +--> Relay control (SVON/EMG)
        |
        +--> Encoder feedback (TIM4)
```

---

## 3. Runtime Architecture (Detailed)

### 3.1 Loop Model

- Main loop: `while(1)`
- Scheduler trigger: `SysTick` sets `interrupt_flag`
- Control execution: `PositionControl_Update()` when `interrupt_flag == 1`
- Communication execution: `MX_LWIP_Process()` every loop

### 3.2 Mode State

- `STEER_MODE_NONE`: idle (control disable)
- `STEER_MODE_AUTO`: target from PC packet
- `STEER_MODE_MANUAL`: target from joystick packet
- `STEER_MODE_ESTOP`: emergency stop path

### 3.3 Data Path

1. UDP packet arrives via LwIP callback
2. packet parsed into normalized `AutoDrive_Packet_t`
3. mode/emergency flags updated
4. main loop applies mode transition + target update
5. 1ms control loop computes output and actuates motor

---

## 4. Control Design

### 4.1 Control Pipeline (1ms)

- Sense: read encoder angle, compute error
- Control: safety check + PID calculate
- Actuate: set pulse frequency/direction
- Comms: network polling and command application

### 4.2 Core Parameters (Current)

- `CONTROL_PERIOD_MS = 1`
- PID default: `Kp=50`, `Ki=5`, `Kd=20`
- Output limit: `10000 Hz`
- Position tolerance: `0.5 deg`

### 4.3 Multi-turn Operation (12:1 Gear Requirement)

- Motor-angle control range: `-4320 ~ +4320 deg`
- Tracking error safety threshold: `MAX_TRACKING_ERROR_DEG = 4500`
- 목적: 다회전 명령 운용에서 불필요한 즉시 ESTOP 방지

Note:
- 최종 안전값은 실차/기구 통합 시험으로 재튜닝해야 함.

### 4.4 Command Lifecycle (No Homing Baseline)

- accepted command unit: `steering_deg` input -> `motor_deg` internal target
- source tracking: `UDP`, `KEYBOARD`, `SERVICE`, `LOCALTEST`
- state tracking: `CMD_IDLE`, `CMD_ACTIVE`, `CMD_REACHED`, `CMD_TIMEOUT`, `CMD_ABORTED`, `CMD_FAULTED`
- event logs:
  - `CMD_START`
  - `CMD_REACHED`
  - `CMD_TIMEOUT`
  - `CMD_ABORT`
  - `CMD_FAULT`

현재 정책:
- target 수락 시 `command_id` 발급
- `abs(error) <= 0.5 deg`가 `100 ms` 유지되면 `CMD_REACHED`
- `CMD_REACHED` 직후 `PulseControl_Stop()`으로 출력을 정지
- timeout / ESTOP / disable / replace / safety fault는 result code와 함께 종료 로그로 남김

아직 남은 gap:
- boot-time auto-enable 제거
- latched fault / clear policy
- encoder unwrap / accum_count
- encoder-ADC cross-check

---

## 5. Safety Architecture

### 5.1 Active Safety Paths

1. Communication timeout fail-safe
- 조건: AUTO/MANUAL에서 RX timeout(`ETHCOMM_RX_TIMEOUT_MS`) 초과
- 동작: mode forced to ESTOP

2. One-shot emergency request
- 조건: incoming packet emergency bit set
- 동작: immediate `PositionControl_EmergencyStop()`

3. Software safety check
- 조건: angle limit out / tracking error over limit
- 동작: emergency stop path

4. Hardware watchdog
- IWDG enabled
- timeout tuned to ~0.5s

### 5.2 EmergencyStop Behavior

- `PulseControl_Stop()`
- PID integral reset
- `Relay_Emergency()` (EMG low)
- mode/state switched to emergency mode

---

## 6. Hardware Interface

### 6.1 Pin Map (Summary)

| Function | MCU Pin | Peripheral/IO | Note |
|---|---|---|---|
| Pulse Output | PE9 | TIM1_CH1 | Servo pulse |
| Direction Output | PE10 | GPIO Output | CW/CCW direction |
| Encoder A | PD12 | TIM4_CH1 | Quadrature input |
| Encoder B | PD13 | TIM4_CH2 | Quadrature input |
| SVON Relay | PD14 | GPIO Output | Active LOW |
| EMG Relay | PD15 | GPIO Output | Active LOW |
| Debug UART TX | PD8 | USART3_TX | 115200 |
| Debug UART RX | PD9 | USART3_RX | 115200 |
| Ethernet | RMII pins | ETH + LAN8742 | UDP comm |

상세 매핑: [hardware_pinmap.md](hardware_pinmap.md)

---

## 7. Latency Measurement & Evidence

### 7.1 Measurement Scope

- Stages: `Sense`, `Control`, `Actuate`, `Comms`
- KPIs: `avg`, `p99`, `max` (cycles/us), `deadline_miss_count`

### 7.2 Instrumentation

- DWT CYCCNT based instrumentation
- Macros: `LAT_BEGIN(stage)` / `LAT_END(stage)`
- deadline miss count in `SysTick`

### 7.3 Automatic Batch Output

- `LATENCY_AUTO_REPORT_SAMPLES = 2000`
- 2000 samples reached => auto output:
  - `LATENCY_BATCH_BEGIN`
  - `LATENCY_STAGE`
  - `LATENCY_BATCH_END`

### 7.4 Evidence Storage Workflow (Windows)

- logger script: `measurements/start_latency_log.ps1`
- output files:
  - `latency_YYYY-MM-DD_HHMMSS.csv`
  - `latency_YYYY-MM-DD_HHMMSS_meta.md`

관련 문서:
- [latency_measurement_spec.md](latency_measurement_spec.md)
- [latency_contract.md](latency_contract.md)
- [latency_code_application.md](latency_code_application.md)
- [latency_data_evidence.md](latency_data_evidence.md)
- [command_lifecycle_no_homing_spec.md](command_lifecycle_no_homing_spec.md)

---

## 8. Build/Run Notes

- Build option: `-O2` (measurement runs)
- Keep clock/PLL fixed for comparable runs
- Minimize runtime logs during strict latency measurement
- Keep input sequence fixed for reproducibility

---

## 9. Engineering Process & Traceability

- Change/fix logs: `change_code/YYYY-MM-DD.md` + `change_code/README.md`
- Legacy integrated log: `change_code/change_code.md` (+ `Old/change_code.md`)
- Historical problems: `Old/problem.md`
- Cleanup candidates: `docs_cleanup_candidates.md`

---

## 10. Planning Docs

- [career_roadmap_2026.md](career_roadmap_2026.md): 취업 연계 관점의 2026-03-12 ~ 2026-06-15 로드맵
- [part_requirements_2026.md](part_requirements_2026.md): 4개 파트별 REQ, 인수기준, 월별 목표
- [command_lifecycle_no_homing_spec.md](command_lifecycle_no_homing_spec.md): homing 제외 현재 단계의 명령 시작/완료/중단/timeout 명세

---

## 11. Steering Portal

- [steering_portal/index.html](steering_portal/index.html): 프로젝트 목표, 아키텍처, 인터페이스 규격, command lifecycle, 로그 증거, 현재 문제, 향후 REQ, 팀 분배, 코드 탐색을 한 번에 보여주는 로컬 설명 포털
- [doxygen/html/index.html](doxygen/html/index.html): 함수/파일/그래프 중심 Doxygen 문서

---

## 12. Remaining Engineering Work

- UART logging path async (DMA + ring buffer)
- Safety parameter tuning with real vehicle/mechanical tests
- Batch CSV post-processing automation
- Fault latch / clear policy standardization and diagnostics framing

---

## 13. Interview Short Pitch

"Built a 1ms STM32 steering sub-controller with UDP command handling, integrated fail-safe paths (timeout/ESTOP/watchdog), and a DWT-based latency evidence pipeline that automatically reports p99/worst-case and deadline misses in reproducible batches."
