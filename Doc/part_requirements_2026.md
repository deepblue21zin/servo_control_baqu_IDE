# 파트별 REQ 및 인수기준

작성 기준일: 2026-03-12
적용 기간: 2026-03-12 ~ 2026-06-15
대상 프로젝트: `servo_control_baqu`

이 문서는 4개 파트로 나뉜 현재 프로젝트를, 실제 하달 가능한 수준의 요구사항 문서로 정리한 것이다.

## 1. 공통 원칙

### 1.1 시스템 역할 정의

이 프로젝트의 역할은 "상위 제어기에서 받은 목표 조향 명령을 STM32가 안전하게 추종하는 조향 액추에이터 서브컨트롤러"이다.

즉, 인지/경로계획을 만들지 않는다. 대신 아래를 제대로 만든다.

1. 위치 피드백
2. 폐루프 제어
3. startup/homing
4. fail-safe
5. 상위기 연동
6. 검증 근거

### 1.2 공통 시스템 REQ

- `REQ-SYS-001`: 모든 인터페이스는 단위를 명시해야 한다.
- `REQ-SYS-002`: 외부 명령 단위는 `steering_deg`로 통일하는 것을 권장한다.
- `REQ-SYS-003`: 내부 제어 계산 단위는 `motor_deg` 또는 `encoder_count`로 고정해야 한다.
- `REQ-SYS-004`: `gear_ratio`, `deg_per_count`, `pulse_per_deg`는 단일 헤더에서 관리해야 한다.
- `REQ-SYS-005`: 1ms 제어 루프 경로에는 블로킹 함수와 과도한 `printf`를 넣지 않는다.
- `REQ-SYS-006`: fault는 코드, 발생 조건, 복귀 조건을 함께 기록해야 한다.
- `REQ-SYS-007`: 각 파트는 월말마다 테스트 결과를 문서 또는 로그로 남겨야 한다.
- `REQ-SYS-008`: 요구사항 ID와 테스트 결과를 연결할 수 있어야 한다.

### 1.3 공통 제출물

각 파트 담당자는 월말 기준으로 아래 4개를 제출한다.

1. 코드 변경본
2. 변경 요약
3. 테스트 로그 또는 사진/영상
4. 남은 리스크 3개 이내 정리

## 2. 인터페이스 기준

### 권장 인터페이스 분리

| 계층 | 권장 단위 | 설명 |
|---|---|---|
| 상위기 명령 | `steering_deg` | 실제 조향축 기준 목표값 |
| 제어기 내부 | `motor_deg` | 기어비 반영 후 PID와 safety에 사용 |
| 센서 원본 | `encoder_count`, `adc_raw` | 가공 전 값 |
| 출력 | `pulse_hz`, `dir` | 드라이버로 전달되는 값 |

### 반드시 정리할 사항

현재 코드/문서에는 `-45 deg`와 `-4320 deg`가 함께 등장한다. 이는 조향축 기준과 모터축 기준이 섞인 상태이므로 2026-03-31 전까지 책임자를 지정해 정리해야 한다.

## 3. 파트 1: `position_control`

### 담당 범위

- 파일: `Core/Src/position_control.c`, `Core/Inc/position_control.h`
- 협업 파일: `Core/Src/main.c`
- 담당 목적: 1ms 폐루프 제어기와 안전 제어의 중심 로직 완성

### 월별 목표

| 마감일 | 목표 |
|---|---|
| 2026-03-31 | 제어 단위, 상태, 제한값 정의 완료 |
| 2026-04-30 | homing, timeout, estop과 연동 완료 |
| 2026-05-31 | 튜닝 및 반복 시험 완료 |
| 2026-06-15 | 면접 설명용 제어 구조 정리 |

### 상세 REQ

- `REQ-POS-001`: 제어기는 1ms 주기로 실행되는 것을 전제로 동작해야 한다.
- `REQ-POS-002`: 목표값 입력 단위와 내부 계산 단위를 명확히 구분해야 한다.
- `REQ-POS-003`: PID는 `Kp`, `Ki`, `Kd`, anti-windup, output saturation을 포함해야 한다.
- `REQ-POS-004`: `Disable`, `Enable`, `EmergencyStop` 호출 시 출력과 내부 상태가 일관되게 변해야 한다.
- `REQ-POS-005`: angle limit, tracking error limit 위반 시 즉시 fail-safe 경로로 진입해야 한다.
- `REQ-POS-006`: 현재각, 목표각, 오차, 출력, 안정화 여부, 마지막 fault를 조회 가능해야 한다.
- `REQ-POS-007`: homing 미완료 상태에서는 RUN 진입을 허용하지 않는 정책을 지원해야 한다.
- `REQ-POS-008`: tuning 값은 코드 여러 곳에 흩어지지 않고 단일 소스에서 관리되어야 한다.

### 인수 기준

1. `0 deg -> +10 deg -> 0 deg -> -10 deg -> 0 deg` 시험 로그가 존재한다.
2. enable/disable/estop 전환 시 출력이 예상대로 정지 또는 복귀한다.
3. 과대 목표값 또는 과대 추종오차 주입 시 fault가 발생한다.
4. 제어 상태를 문서와 UART 로그로 설명할 수 있다.

## 4. 파트 2: `encoder_reader` + `adc_potentiometer`

### 담당 범위

- 파일: `Core/Src/encoder_reader.c`, `Core/Inc/encoder_reader.h`
- 파일: `Core/Src/adc_potentiometer.c`, `Core/Inc/adc_potentiometer.h`
- 담당 목적: 상대위치와 절대위치를 신뢰 가능한 형태로 제공

### 월별 목표

| 마감일 | 목표 |
|---|---|
| 2026-03-31 | 센서 단위, 보정 방식, API 확정 |
| 2026-04-30 | unwrap 누적 및 ADC 유효성 판정 완료 |
| 2026-05-31 | 엔코더-ADC 교차검증 결과 확보 |
| 2026-06-15 | 센서 파이프라인 설명 자료 정리 |

### 상세 REQ

- `REQ-SEN-001`: 엔코더는 16비트 counter overflow/underflow를 고려한 누적 count를 제공해야 한다.
- `REQ-SEN-002`: 엔코더 API는 최소한 `raw_count`, `accum_count`, `motor_deg`를 설명 가능해야 한다.
- `REQ-SEN-003`: ADC는 보정 전 raw 값과 보정 후 angle 값을 모두 제공해야 한다.
- `REQ-SEN-004`: ADC는 disconnect, range error, jump, stuck 가능성을 진단할 수 있어야 한다.
- `REQ-SEN-005`: 엔코더와 ADC의 차이를 확인하는 cross-check API 또는 진단 로직이 있어야 한다.
- `REQ-SEN-006`: 센서 단위 변환 상수는 하드코딩하지 말고 문서와 코드에서 동일해야 한다.
- `REQ-SEN-007`: homing과 runtime에서 각각 어떤 센서값을 신뢰할지 정책을 명확히 해야 한다.

### 인수 기준

1. 장시간 정/역방향 회전 중 angle discontinuity가 없어야 한다.
2. ADC 보정 절차가 문서와 코드에서 일치해야 한다.
3. 센서 raw 값과 환산값을 UART 또는 로그로 확인할 수 있어야 한다.
4. 엔코더와 ADC 차이가 기준을 초과할 때 경고 또는 fault를 낼 수 있어야 한다.

## 5. 파트 3: `homing` + `relay_control`

### 담당 범위

- 파일: `Core/Src/homing.c`, `Core/Inc/homing.h`
- 파일: `Core/Src/relay_control.c`, `Core/Inc/relay_control.h`
- 협업 파일: `Core/Src/main.c`
- 담당 목적: 안전한 startup, 원점 설정, EMG/SVON 시퀀스 완성

### 월별 목표

| 마감일 | 목표 |
|---|---|
| 2026-03-31 | startup/homing/relay state 정의 완료 |
| 2026-04-30 | 실제 부팅 경로에 homing 통합 완료 |
| 2026-05-31 | fault 및 복귀 절차 검증 완료 |
| 2026-06-15 | 시연 시나리오 정리 |

### 상세 REQ

- `REQ-HOM-001`: 시스템 전원 인가 시 안전 초기 상태를 명확히 정의해야 한다.
- `REQ-HOM-002`: homing은 ADC 절대위치를 읽고 엔코더 기준점을 설정하는 절차를 포함해야 한다.
- `REQ-HOM-003`: homing 실패 시 제어기 RUN 진입을 금지해야 한다.
- `REQ-HOM-004`: `Relay_ServoOn`, `Relay_ServoOff`, `Relay_Emergency`, `Relay_EmergencyRelease`의 호출 순서와 지연시간을 문서화해야 한다.
- `REQ-HOM-005`: startup 중, homing 중, running 중, estop 중 relay 상태를 표로 정리해야 한다.
- `REQ-HOM-006`: runtime 중 emergency가 발생하면 homing 상태와 무관하게 즉시 안전정지해야 한다.
- `REQ-HOM-007`: recovery 조건을 만족하기 전에는 EMG release와 control enable을 허용하지 않아야 한다.

### 인수 기준

1. 전원 인가 후 homing 성공 시에만 RUN으로 진입한다.
2. ADC 이상 또는 homing 실패 시 RUN 진입이 차단된다.
3. emergency 발생 시 SVON/EMG 출력 상태를 실제 GPIO 기준으로 설명할 수 있다.
4. recovery 절차가 문서와 코드에서 동일하다.

## 6. 파트 4: `pulse_control`

### 담당 범위

- 파일: `Core/Src/pulse_control.c`, `Core/Inc/pulse_control.h`
- 담당 목적: 드라이버가 신뢰할 수 있는 pulse/direction 출력을 제공

### 월별 목표

| 마감일 | 목표 |
|---|---|
| 2026-03-31 | 주파수 계산식, 방향 정책, 한계값 확정 |
| 2026-04-30 | 연속 모드와 step 모드 안정화 |
| 2026-05-31 | endurance 및 glitch 확인 완료 |
| 2026-06-15 | 출력 파형 설명 자료 정리 |

### 상세 REQ

- `REQ-PUL-001`: 양수/음수 명령에 대해 방향 핀이 일관되게 동작해야 한다.
- `REQ-PUL-002`: 연속 제어 모드에서 주파수 변경이 안정적으로 반영되어야 한다.
- `REQ-PUL-003`: step 모드와 speed 모드의 상태 전이가 충돌하지 않아야 한다.
- `REQ-PUL-004`: 정지 명령 시 PWM 출력이 즉시 중지되어야 한다.
- `REQ-PUL-005`: ARR/CCR 계산은 타이머 클럭과 prescaler 기준으로 설명 가능해야 한다.
- `REQ-PUL-006`: 최소/최대 주파수, 최소 펄스폭, 방향 전환 정책을 문서화해야 한다.
- `REQ-PUL-007`: fault 또는 estop 시 잔류 펄스가 남지 않도록 stop 경로를 보장해야 한다.

### 인수 기준

1. 저속, 중속, 고속 구간에서 설정 주파수와 실제 출력이 크게 벗어나지 않는다.
2. 방향 전환 시 오동작이나 한쪽 고정 회전 문제가 없다.
3. estop 후 펄스 출력이 정지하고 busy 상태가 정상 복구된다.
4. 오실로스코프 또는 레지스터 로그로 출력 근거를 제시할 수 있다.

## 7. 통합 체크리스트

다음 항목은 4개 파트가 함께 맞춰야 한다.

1. 단위 체계가 문서와 코드에서 동일한가
2. startup/homing/ready/running/estop 상태 정의가 같은가
3. 센서 fault와 통신 fault가 동일한 fault 체계로 관리되는가
4. estop 후 복귀 절차가 한 곳에서 설명되는가
5. 월말마다 테스트 결과를 남기고 있는가

## 8. 최종 Definition of Done

2026-05-31 기준으로 아래를 만족하면 "개발 완료"로 본다.

1. 4개 파트가 통합된 상태에서 목표 조향 추종이 가능하다.
2. homing 성공/실패 경로가 분리되어 있다.
3. timeout, 센서 이상, 추종 실패, estop 경로가 모두 동작한다.
4. 파트별 REQ에 대한 시험 기록이 있다.
5. README와 문서만으로도 프로젝트 구조를 이해할 수 있다.

2026-06-15 기준으로 아래를 추가 만족하면 "포트폴리오 완료"로 본다.

1. 발표용 설명 자료가 있다.
2. 시연 영상 또는 로그 패키지가 있다.
3. 설계 결정, 한계, 다음 단계까지 설명 가능하다.
