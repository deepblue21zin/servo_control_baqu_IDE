# Keyboard Test Quick Reference

이 문서는 UART 키보드 테스트 모드에서 바로 써먹을 수 있는 명령, 로그 활용법, encoder 손회전 확인 절차, fail-safe 캡처 절차를 빠르게 정리한 문서다.

## 1. 현재 키보드 명령

- `A`: 목표 조향각을 왼쪽으로 `1 deg` 감소
- `D`: 목표 조향각을 오른쪽으로 `1 deg` 증가
- `S`: 목표 조향각을 `0 deg`로 설정
- `E`: position control enable
- `Q`: position control disable
- `X`: emergency stop
- `P`: 현재 `target/current/error/output/DIR/ENC` snapshot 출력
- `L`: 주기적 CSV 로그 on/off 토글
- `H`: 도움말 다시 출력
- 숫자 입력 후 `Enter`: 절대 목표 조향각 입력

예:
- `5` + `Enter`
- `-3.5` + `Enter`
- `0` + `Enter`

## 2. 주기적 CSV 로그 형식

CSV 로그가 켜져 있으면 아래 형식으로 주기적으로 출력된다.

```text
CSV_HEADER,ms,mode,target_deg,current_deg,error_deg,output,dir,enc
CSV,1234,1,-5.000,0.000,-5.000,-7400,0,32769
```

의미:
- `ms`: `HAL_GetTick()` 기준 시간
- `mode`: `PositionControl_GetMode()`
- `target_deg`: 조향축 기준 목표각
- `current_deg`: 조향축 기준 현재각
- `error_deg`: 조향축 기준 오차
- `output`: position control output
- `dir`: `PE10` 현재 상태
- `enc`: `TIM4 raw counter`

제출용 그래프는 가능하면 이 CSV 라인을 기준으로 만든다.

## 3. encoder 손회전 확인 절차

핵심 목표는 "내가 실제로 축을 움직였을 때 TIM4 raw counter가 변하는가"를 확인하는 것이다.

### 3-1. 가장 안전한 순서

1. PuTTY 연결
2. `L`로 CSV logging 켬
3. `P` 한 번 눌러 baseline 확인
4. 먼저 `X`를 눌러 emergency 상태로 보냄
5. 모터가 실제로 손으로 돌아가는지 천천히 확인

### 3-2. `X` 후에도 손으로 안 돌아갈 때

- 이 프로젝트에서 `Q`는 제어기만 disable하고 servo torque를 반드시 풀어주는 동작은 아니다.
- `X`는 `Relay_Emergency()`를 호출하므로 먼저 이 상태를 시도한다.
- 그래도 축이 기계적으로 안 돌아가면 억지로 비틀지 않는다.
- 그 경우는 아래 둘 중 하나로 진행한다.
  - servo drive 전원을 안전하게 끌 수 있으면 끄고, encoder 쪽 전원은 살아 있는지 확인한 뒤 손회전 시험
  - 손회전 대신 아주 작은 command(`-1`, `0`, `+1`)를 주고 `enc` 변화만 확인

### 3-3. 실제 확인 방법

1. `P`를 눌러 현재 `ENC` 값 기록
2. 축을 시계 방향으로 아주 조금 움직임
3. `P`를 다시 눌러 `ENC`와 `current_deg` 확인
4. 축을 반시계 방향으로 아주 조금 움직임
5. `P`를 다시 눌러 비교

### 3-4. 기대 결과

- 정상이라면:
  - `ENC`가 바뀐다
  - `current_deg`도 바뀐다
  - 시계/반시계에서 부호가 일관된다

- 비정상이라면:
  - `ENC`가 그대로다
  - `current_deg`가 계속 `0.00` 근처다
  - 시계/반시계 모두 같은 방향으로 변한다

이 경우는 sensor path 문제 가능성이 높다.

## 4. fail-safe 캡처 방법

fail-safe는 "막 고장을 내는 것"보다 "통제된 fault injection"으로 남기는 것이 좋다.

### 4-1. 이번 주에 가장 추천하는 4가지

1. `Q` disable test
- 절차:
  - `E` 후 작은 목표각 명령
  - `Q`
- 기대:
  - output 0
  - pulse stop
- 남길 것:
  - UART log
  - 가능하면 `PE9` 캡처

2. `X` emergency stop test
- 절차:
  - 작은 목표각 명령
  - `X`
- 기대:
  - emergency log
  - pulse stop
  - EMG line active
- 남길 것:
  - UART log
  - 가능하면 GPIO/logic analyzer 캡처

3. invalid input reject
- 절차:
  - `--100` 또는 비정상 입력
- 기대:
  - invalid target 메시지
- 남길 것:
  - UART log

4. startup inhibit
- 절차:
  - 전원 인가 직후 아무 명령 없이 관찰
- 기대:
  - ready 조건 전에는 pulse 없음
- 남길 것:
  - power-on log
  - `PE9` 무출력 캡처

## 5. 제출용으로 남기기 좋은 조합

- `latency_trace.png`
- `control_trace.png`
- 모터 움직임 짧은 영상 1개
- `Q`, `X`, invalid input` 로그 캡처 각 1개
- `encoder hand-turn` 확인 로그 1개
