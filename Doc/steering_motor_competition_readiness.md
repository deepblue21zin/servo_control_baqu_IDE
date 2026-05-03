# 조향 모터 시스템 구성 및 대회 준비 부족점 정리

작성일: 2026-05-01  
대상: 대학생 자작자율주행 대회 조향 모터 시스템을 준비하는 팀원 / 리뷰어 / 지도교수

## 1. 이 시스템의 목표

이 프로젝트는 상위 자율주행 제어기에서 받은 조향 목표각을 STM32F429ZI가 받아서, LS ELECTRIC 서보드라이브에 pulse/direction 명령으로 전달하는 조향 서브컨트롤러이다.

최종 목표는 단순히 모터를 돌리는 것이 아니라, 차량 기준 조향각을 안전하게 제어하는 것이다.

```text
상위 제어기 목표 조향각
-> STM32 1 ms 제어 루프
-> pulse/direction 출력
-> 서보드라이브/모터
-> 조향 기구
-> 피드백 센서로 현재 조향각 검증
```

현재 프로젝트는 벤치 bring-up 단계이며, 차량 장착 전 테스트를 위해 여러 안전 기능을 임시로 완화한 상태이다. 따라서 지금 빌드는 대회 실차용 완성 상태가 아니라, RS422 피드백과 TIM2 인크리멘탈 피드백을 비교하면서 제어 구조를 안정화하는 중간 상태로 봐야 한다.

## 2. 현재 전체 구성

### 2.1 하드웨어 구성

| 항목 | 현재 구성 | 상태 |
|---|---|---|
| MCU | STM32F429ZI | 사용 중 |
| Servo Driver | LS ELECTRIC XDL-L7SA004BAA 계열 | pulse/direction 제어 대상 |
| Motor / Encoder | 12000 PPR, quadrature x4 가정 | TIM2 인크리멘탈 기준 |
| Pulse 출력 | `PE9 / TIM1_CH1` | 서보드라이브 pulse 입력 |
| Direction 출력 | `PE10 / GPIO` | 서보드라이브 direction 입력 |
| 인크리멘탈 엔코더 | `PA0 / TIM2_CH1`, `PB3 / TIM2_CH2` | 현재 제어 기준 피드백 |
| RS422 수신 | `USART2_RX = PA3` | 수신/로그 구현됨 |
| ADC 포텐셔미터 | ADC1 path 존재 | 현재 벤치에서는 비활성화 |
| CAN | CAN runtime/bridge 코드 존재 | 현재 RS422 벤치에서는 비활성화 |

### 2.2 소프트웨어 구성

| 파일 | 역할 |
|---|---|
| `Core/Src/main.c` | CubeMX 주변장치 초기화 후 `AppRuntime_Init()`, `AppRuntime_RunIteration()` 호출 |
| `Core/Src/app_runtime.c` | 부팅 시퀀스, 키보드 벤치 명령, UDP path, CSV/진단 로그, watchdog refresh |
| `Core/Src/position_control.c` | 목표각, 현재각, PID, command lifecycle, enable/disable, emergency 처리 |
| `Core/Src/pulse_control.c` | PID output을 signed pulse frequency로 받아 TIM1 PWM과 direction GPIO로 변환 |
| `Core/Src/encoder_reader.c` | TIM2 인크리멘탈 카운터를 누적 count, motor angle, steering angle로 변환 |
| `Core/Src/rs422_encoder_uart.c` | USART2 RS422 4-byte count 수신, zero 보정, 각도 환산, 로그 출력 |
| `Core/Src/position_control_safety.c` | tracking error, velocity, timeout 같은 제어 안전 평가 |
| `Core/Src/adc_potentiometer.c` | 포텐셔미터 기반 절대각 보조 센서 경로, 현재 벤치에서는 꺼짐 |
| `Core/Inc/project_params.h` | 현재 벤치/제어/센서/통신 설정값 집중 관리 |

## 3. 현재 데이터 흐름

### 3.1 제어 명령 흐름

현재 키보드 벤치 모드에서는 Putty에서 숫자 목표각을 넣거나 `A/D/S/E/Q/X/Z/P/L/H` 키를 사용한다.

```text
Putty keyboard input
-> AppRuntime_KeyboardProcessInput()
-> steering_deg target
-> SteeringDegToMotorDeg()
-> PositionControl_SetTargetWithSource()
-> PositionControl_Update()
-> PID_Calculate()
-> PulseControl_SetFrequency()
-> TIM1 pulse + direction GPIO
```

UDP 경로도 코드에 존재하지만, 현재 기본 운용은 keyboard bench 중심이다.

### 3.2 현재 피드백 흐름

현재 실제 제어 기준은 아직 RS422가 아니라 TIM2 인크리멘탈 엔코더이다.

```text
TIM2 raw count
-> EncoderReader_UpdateCount()
-> accum_count
-> motor_deg
-> steering_deg
-> PositionControl current_angle
```

RS422는 현재 별도 수신/로그/zero 보정까지 구현되어 있으며, 아직 PID의 `current_angle`로 들어가지는 않는다.

```text
USART2 RX PA3
-> Rs422Encoder_Service()
-> 4-byte little-endian signed count
-> raw_count
-> raw_count - zero_count
-> motor_deg / steering_deg
-> [RS422] log
```

즉 현재 구조는 다음과 같다.

```text
제어 기준: TIM2 인크리멘탈 엔코더
검증 후보: RS422 서보드라이브 위치값
```

## 4. 좌표계와 단위

현재 코드의 핵심 변환 상수는 다음과 같다.

```text
ENCODER_PPR = 12000
ENCODER_QUADRATURE = 4
ENCODER_COUNT_PER_REV = 48000
ENCODER_DEG_PER_COUNT = 360 / 48000 = 0.0075 motor deg/count
STEERING_GEAR_RATIO = 12.5
1 steering deg = 12.5 motor deg
1 steering deg = 약 1666.67 count
```

조향각 기준으로 보면 다음 범위가 된다.

```text
+45 steering deg = +562.5 motor deg = 약 +75000 count
-45 steering deg = -562.5 motor deg = 약 -75000 count
```

RS422 raw count는 `int32_t`로 해석하므로 자료형 범위는 다음과 같다.

```text
-2147483648 ~ +2147483647
```

하지만 차량 조향 시스템에서 중요한 값은 raw count 자체가 아니라 중앙 기준 상대 count이다.

```text
rs422_relative_count = rs422_raw_count - rs422_center_count
rs422_steering_deg = rs422_relative_count * 0.0006
```

## 5. 인크리멘탈 제어와 RS422 절대값 제어 비교

| 항목 | TIM2 인크리멘탈 기준 | RS422 서보드라이브 값 기준 |
|---|---|---|
| 현재 제어 사용 여부 | 사용 중 | 아직 로그/검증용 |
| 장점 | 구현 단순, 현재 PID와 연결 완료, 빠른 bring-up 가능 | 서보드라이브 내부 위치와 직접 연결, 장기적으로 절대/준절대 기준에 유리 |
| 단점 | 전원 재시작 후 절대 위치를 모름, 카운트 누락/노이즈에 취약 | 프로토콜/스케일/zero 검증 필요, 아직 제어 루프에 연결 전 |
| 차량 중앙 0도 | 부팅 reset 또는 `Z`로 임시 정의 | `zero_count` 또는 실제 중앙 `center_count` 필요 |
| 대회 최종 권장 역할 | 보조 검증 또는 백업 | 안정화 후 primary feedback 후보 |

현재 바로 RS422를 제어 기준으로 바꾸는 것은 위험하다. 더 좋은 방향은 다음 순서이다.

```text
1. TIM2로 계속 제어한다.
2. RS422 각도를 동시에 계산한다.
3. Z 명령으로 두 기준을 같은 0도로 맞춘다.
4. 여러 목표각에서 TIM2_deg와 RS422_deg 차이를 기록한다.
5. 충분히 일관되면 RS422를 primary feedback으로 승격한다.
6. TIM2는 backup 또는 cross-check로 유지한다.
```

## 6. 현재 구현된 기능

### 6.1 키보드 벤치 명령

| 명령 | 의미 |
|---|---|
| `A` / `D` | 목표 조향각을 step만큼 좌/우 이동 |
| 숫자 + Enter | 해당 조향각으로 목표 지정 |
| `S` | 목표각을 0도로 보냄 |
| `Z` | 현재 위치를 테스트 0도로 재정의 |
| `E` | 제어 enable |
| `Q` | 제어 disable |
| `X` | emergency stop 호출 |
| `P` | 현재 상태 snapshot 출력 |
| `L` | CSV 로그 on/off |
| `H` | 도움말 출력 |

`S`와 `Z`는 다르다.

```text
S = 목표값을 0도로 보냄
Z = 현재 위치 자체를 0도로 재정의함
```

### 6.2 RS422 수신 상태

현재 구현된 RS422 기능은 다음과 같다.

- USART2 PA3 RX 사용
- 38400 baud, 8N1
- 4바이트 little-endian signed count로 해석
- raw count 출력
- `Z` 이후 zero 기준 relative count 출력
- `MotorDegToSteeringDeg()`로 조향각 환산
- `[RS422]` 및 `[RS422][STAT]` 로그 출력

아직 구현되지 않은 것은 다음이다.

- RS422 값을 `PositionControl`의 실제 현재각으로 사용
- RS422 timeout/fault를 safety에 연결
- TIM2와 RS422 차이를 fault/warn으로 판정
- 차량 중앙 `center_count` 영구 저장

### 6.3 현재 벤치 안전 스위치

현재 테스트 편의를 위해 완화된 설정이 있다.

| 설정 | 현재 의미 |
|---|---|
| `APP_RUNTIME_ADC_POT_ENABLE = 0` | 가변저항 절대각 센서 미사용 |
| `APP_RUNTIME_AUTO_HOME_ON_BOOT = 0` | ADC 기반 부팅 홈잉 미사용 |
| `APP_RUNTIME_EMERGENCY_LATCH_ENABLE = 0` | 벤치에서 emergency latch 지속 방지 |
| `SENSOR_DIRECTION_PLAUSIBILITY_ENABLE = 0` | 방향 불일치 fault 임시 비활성화 |
| `APP_RUNTIME_CAN_ENABLE = 0` | CAN peripheral startup 비활성화 |
| `CAN_ENCODER_BRIDGE_ENABLE = 0` | CAN encoder bridge 비활성화 |

대회 실차용으로는 이 중 일부를 반드시 복구해야 한다.

## 7. 현재 확인된 사실

현재까지 확인된 것은 다음이다.

- STM32CubeIDE Debug 빌드가 통과한다.
- TIM2 인크리멘탈 엔코더 기반 제어는 현재 제어 루프에 연결되어 있다.
- Putty 키보드 명령으로 목표각을 넣을 수 있다.
- CSV, `[ENCDBG]`, `[RS422]` 로그가 출력된다.
- RS422는 0바이트 상태를 벗어나 실제 프레임 수신이 확인되었다.
- RS422 count는 코드상 motor/steering 각도로 환산되고 있다.
- `Z` 명령으로 현재 위치를 테스트 0도로 재정의할 수 있다.

하지만 아직 확인해야 하는 것이 더 중요하다.

- RS422와 TIM2가 같은 움직임에서 같은 방향/비율로 움직이는지 장시간 검증해야 한다.
- RS422 count scale이 실제 서보드라이브 설정과 정확히 맞는지 확인해야 한다.
- 12.5배 속도 요구를 만족하도록 pulse frequency, drive setting, PID를 같이 튜닝해야 한다.
- 차량 장착 후 물리 중앙 기준 `center_count`를 잡아야 한다.

## 8. 대회용 조향 모터 시스템까지 부족한 점

### 8.1 차량 중앙 기준 캘리브레이션

현재 벤치에서는 모터가 무한 회전 가능하고 차량에 장착되어 있지 않기 때문에 물리적 중앙이 없다.

대회 차량에서는 반드시 다음 절차가 필요하다.

```text
1. 바퀴를 실제 직진 중앙으로 정렬한다.
2. 그 순간 RS422 raw count를 읽는다.
3. 그 값을 center_count로 저장한다.
4. 이후 모든 조향각은 raw_count - center_count 기준으로 계산한다.
```

현재 `Z`는 테스트용 zero이고, 대회용으로는 이 값을 Flash 또는 backup SRAM에 저장하는 calibration 명령이 필요하다.

### 8.2 RS422와 TIM2 더블체크 로직

현재 RS422와 TIM2는 둘 다 읽히지만, 아직 자동 비교하지 않는다.

필요한 로직은 다음과 같다.

```text
diff_deg = tim2_steering_deg - rs422_steering_deg

if abs(diff_deg) > warn_threshold:
    warning

if abs(diff_deg) > fault_threshold for 일정 시간:
    fault latch
```

초기 권장 threshold는 벤치 로그를 보고 정해야 한다. 임의 시작점은 다음 정도가 현실적이다.

```text
warn: 1.0 ~ 2.0 deg
fault: 3.0 ~ 5.0 deg
persistence: 50 ~ 100 ms
```

### 8.3 속도 부족

현재 느린 가장 큰 이유는 PID output limit와 pulse frequency clamp가 낮기 때문이다.

현재 개념상 속도는 다음처럼 계산된다.

```text
10000 pulse/s * 0.003 motor deg/pulse = 30 motor deg/s
30 / 12.5 = 2.4 steering deg/s
```

12.5배 빠르게 하려면 대략 다음 수준이 필요하다.

```text
10000 Hz -> 125000 Hz
```

따라서 필요한 작업은 다음이다.

- `DEFAULT_OUTPUT_LIMIT` 단계 상승
- `PULSECONTROL_MAX_FREQ_HZ` 상승
- 서보드라이브 electronic gear / pulse input limit 확인
- 목표 도달 시 overshoot 방지를 위한 PID 재튜닝
- 큰 step 명령에서 settling time 측정

한 번에 12.5배로 올리기보다는 다음처럼 단계적으로 검증해야 한다.

```text
25000 Hz
50000 Hz
75000 Hz
100000 Hz
125000 Hz
```

### 8.4 Startup / Ready / Arm 상태기계

현재 벤치에서는 auto enable과 safety 완화가 들어가 있다. 대회용으로는 다음 상태기계가 필요하다.

```text
INIT
-> SENSOR_CHECK
-> CENTER_CALIBRATED
-> READY
-> ARMED
-> RUN
-> ESTOP_LATCH
```

특히 다음 조건이 닫혀야 한다.

- 센서가 준비되기 전에는 enable 금지
- zero/center가 없는 상태에서는 자동 주행 명령 거부
- ESTOP 후에는 명시적 clear 전까지 출력 금지
- watchdog 또는 통신 timeout 시 safe stop

### 8.5 Emergency / Fail-safe 복구

현재 `APP_RUNTIME_EMERGENCY_LATCH_ENABLE = 0`은 테스트 편의용이다.

대회용으로는 다음이 필요하다.

- emergency latch 복구
- fault reason code 정리
- clear 명령과 clear 조건 분리
- relay safe state 검증
- pulse output 강제 stop 검증
- 상위 제어기 timeout 시 neutral 또는 stop 정책 결정

### 8.6 센서 신뢰성

대회용 조향에서는 한 센서값만 믿기보다 최소한 다음을 확인해야 한다.

- RS422 frame timeout
- RS422 checksum 또는 frame validity, 프로토콜이 지원한다면 적용
- TIM2 stale 감지
- TIM2/RS422 cross-check
- direction command와 실제 count 변화 방향 비교
- 큰 가속도/속도 변화에 대한 plausibility check

현재는 일부 진단 코드가 있지만 벤치 편의상 꺼진 것도 있으므로 실차 전 복구가 필요하다.

### 8.7 상위 자율주행 제어기 연동

대회에서는 Putty 키보드가 아니라 상위 자율주행 stack이 목표 조향각을 보낸다.

필요한 것은 다음이다.

- UDP 또는 다른 통신 프로토콜 최종 선택
- packet format 확정
- 주기, timeout, stale command 정책
- 수동/자동/ESTOP mode 전이 정의
- 상위 목표각 단위는 반드시 `steering_deg`로 고정
- 입력 clamp `[-45, +45] deg` 또는 실제 차량 한계로 확정

### 8.8 실제 하드웨어 안전

소프트웨어가 맞아도 대회용으로는 하드웨어 안전이 필요하다.

- 기계적 end-stop
- 서보드라이브 torque/speed limit
- fuse / emergency switch / relay safe cut
- 조향 링크 유격 확인
- 전원 drop 시 출력 상태
- 노이즈/접지/차동 신호 배선 검증

### 8.9 증거 패키지

대회 제출 또는 발표를 위해서는 한 번의 run에 다음을 묶어야 한다.

- firmware commit/hash
- `project_params.h` 설정 snapshot
- Putty raw log
- CSV plot
- `[ENCDBG]` / `[RS422]` 비교 plot
- oscilloscope 또는 logic analyzer pulse waveform
- 실제 바퀴 각도 영상 또는 측정표
- fault/ESTOP 시험 결과

## 9. 권장 개발 순서

### Phase 1: 벤치 기준 맞추기

목표는 TIM2와 RS422를 같은 0도 기준으로 맞추는 것이다.

```text
1. 전원 ON
2. RS422 frame 수신 확인
3. 임의의 테스트 중앙 위치에서 Z 입력
4. 0, +10, -10, +20, -20, +30, -30 deg 명령
5. CSV, ENCDBG, RS422 로그 저장
6. TIM2_deg와 RS422_deg 차이 계산
```

### Phase 2: 속도 튜닝

목표는 12.5배 빠른 조향 속도 요구를 단계적으로 만족하는 것이다.

```text
1. DEFAULT_OUTPUT_LIMIT 상승
2. PULSECONTROL_MAX_FREQ_HZ 상승
3. 서보드라이브 pulse input limit 확인
4. overshoot와 settling time 측정
5. PID 재튜닝
```

### Phase 3: 센서 더블체크

목표는 TIM2와 RS422가 서로 감시하도록 만드는 것이다.

```text
1. RS422 relative angle 계산 안정화
2. TIM2-RS422 diff_deg 로그 추가
3. warn/fault threshold 적용
4. 일정 시간 이상 차이가 나면 fault latch
```

### Phase 4: 차량 중앙 캘리브레이션

목표는 테스트 0도가 아니라 실제 직진 중앙을 0도로 저장하는 것이다.

```text
1. 차량 장착
2. 바퀴 물리 중앙 정렬
3. RS422 center_count 저장
4. TIM2 reset 또는 offset 적용
5. 작은 각도부터 실제 바퀴 각도 검증
```

### Phase 5: 대회용 safety 복구

목표는 벤치 편의 스위치를 실차 안전 기준으로 되돌리는 것이다.

```text
APP_RUNTIME_EMERGENCY_LATCH_ENABLE = 1
SENSOR_DIRECTION_PLAUSIBILITY_ENABLE = 1
필요 시 ADC/추가 센서 cross-check 복구
startup auto-enable 제거
fault clear policy 추가
```

### Phase 6: 상위 제어기 통합

목표는 자율주행 stack에서 안정적으로 조향 명령을 받는 것이다.

```text
1. UDP packet 주기 확정
2. timeout 정책 적용
3. mode 전이 시험
4. manual override와 ESTOP 시험
5. 실제 주행 전 stand test
```

## 10. 현재 상태 한 줄 평가

현재 시스템은 TIM2 인크리멘탈 제어를 기준으로 pulse/direction 조향 제어가 동작하고, RS422 서보드라이브 위치값을 수신해 zero 기준 각도로 비교할 수 있게 된 벤치 bring-up 단계이다.

대학생 자작자율주행 대회용 조향 시스템이 되려면, RS422/TIM2 더블체크, 실제 차량 중앙 캘리브레이션, 속도 12.5배 튜닝, startup/ESTOP/fault latch 안전 상태기계, 상위 제어기 timeout 계약, 하드웨어 안전 검증이 추가로 필요하다.

## 11. 발표나 리뷰에서 말하면 좋은 요약

```text
현재는 인크리멘탈 엔코더 기반 폐루프 제어가 동작하고,
RS422 서보드라이브 위치값은 별도 수신과 zero 보정까지 구현했다.

바로 RS422를 제어 기준으로 바꾸지 않고,
TIM2와 RS422를 같은 0도 기준으로 맞춘 뒤 double-check로 검증하고 있다.

다음 핵심 과제는 속도 12.5배 튜닝, RS422/TIM2 cross-check fault,
차량 직진 중앙 calibration, 실차 safety state machine 완성이다.
```

