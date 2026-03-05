# 모터 미동작 원인 분석 및 수정 이력

> 작성일: 2026-02-24
> 대상 하드웨어: STM32 NUCLEO-F429ZI + LS L7 서보드라이브 (XDL-L7SA004BAA) + 서보모터 (XML-FBL04AMK1)

---

## 목차

1. [하드웨어 구성 개요](#하드웨어-구성-개요)
2. [발견된 버그 목록](#발견된-버그-목록)
3. [현재 미해결 의심 원인](#현재-미해결-의심-원인)
4. [확인된 사항 요약](#확인된-사항-요약)

---

## 하드웨어 구성 개요

```
STM32 NUCLEO-F429ZI
  │
  ├── PE9  (TIM1_CH1 PWM)  ──→ YL-128 RS422 모듈 TXD
  │                               ↓ Y(+) / Z(-) 차동 출력
  │                          L7 드라이브 CN1-9 (PF+) / CN1-10 (PF-)
  │                               ↓ 펄스 입력 (PULS)
  │                          서보모터 회전 (1 펄스 = 1 스텝)
  │
  ├── PE11 (GPIO_Output)   ──→ L7 드라이브 CN1-11 (PR+) 방향 신호
  │                          CN1-12 (PR-) ← [문제: 플로팅]
  │
  ├── PD12/PD13 (TIM4)     ──→ 엔코더 A상/B상 (위치 피드백)
  │
  ├── PD14 (SVON, Active LOW) → 서보 ON/OFF
  └── PD15 (EMG,  Active LOW) → 비상정지
```

---

## 발견된 버그 목록

---

### BUG-01: main.c에서 PWM 조기 시작 (83kHz CCW 무제어 회전)

**증상:** 전원 인가 직후 모터가 역방향(CCW)으로 고속 회전
**발생 위치:** `Core/Src/main.c`

**원인:**
```c
// 기존 코드 (버그)
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // ARR=9 → 83kHz, DIR=LOW(CCW)
```
- `MX_TIM1_Init()`이 ARR=9 (초기값)로 타이머를 설정
- `HAL_TIM_PWM_Start()`를 바로 호출 → 방향, 속도 설정 없이 83kHz 펄스 출력
- PE11 (DIR)이 LOW 상태 → CCW 방향으로 무제어 회전

**수정:**
```c
// 삭제 → PulseControl_SetFrequency() 내부에서 방향/속도 설정 후 PWM 시작
```

---

### BUG-02: PE11이 TIM1_CH2 AF 모드 → GPIO 제어 불가

**증상:** `HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, ...)` 호출해도 방향 변경 안 됨
**발생 위치:** CubeMX 초기 설정

**원인:**
- CubeMX에서 PE11을 TIM1_CH2 대체기능(Alternate Function)으로 설정
- AF 모드에서는 `HAL_GPIO_WritePin()`이 무효 (하드웨어가 타이머가 제어)
- PulseControl 코드는 PE11을 GPIO로 가정하고 방향 제어 시도 → 실제로 반영 안 됨

**수정:**
```c
// main.c USER CODE BEGIN 2에 추가
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin  = GPIO_PIN_11;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // AF에서 GPIO_Output으로 변경
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
```

---

### BUG-03: PulseControl_Stop()에서 HAL 상태 꼬임

**증상:** `PulseControl_Stop()` 호출 후 재시작 시 PWM이 나오지 않음
**발생 위치:** `Core/Src/pulse_control.c`

**원인:**
```c
// 기존 코드 (버그)
HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);  // State = READY로 변경
HAL_TIM_PWM_Stop(p_htim1, TIM_CHANNEL_1);     // State가 이미 READY → HAL_ERROR
// → HAL 내부 state가 BUSY/READY 사이에서 불일치 발생
```

- `HAL_TIM_PWM_Stop_IT()`은 State를 READY로 바꿈
- 이후 `HAL_TIM_PWM_Stop()`은 State=READY를 보고 "이미 정지됨"으로 처리 → HAL_ERROR 반환
- HAL 내부 state 불일치로 이후 Start 호출이 실패

**수정:**
```c
void PulseControl_Stop(void) {
    __HAL_TIM_DISABLE_IT(p_htim1, TIM_IT_CC1);  // 인터럽트만 비활성화
    HAL_TIM_PWM_Stop(p_htim1, TIM_CHANNEL_1);   // 한 번만 호출로 정상 정지
    remaining_steps = 0;
    is_busy = 0;
}
```

---

### BUG-04: Safety 체크 순서 오류 (1-step 지연 버그)

**증상:** 비상정지 판단이 1ms 늦게 반응
**발생 위치:** `Core/Src/position_control.c` → `PositionControl_Update()`

**원인:**
```c
// 기존 코드 (버그)
state.current_angle = EncoderReader_GetAngleDeg();
if (!PositionControl_CheckSafety()) { ... }  // 이 시점 state.error = 이전 루프 값!
state.error = state.target_angle - state.current_angle;  // 오차 계산이 나중
```
- `CheckSafety()`가 `state.error`를 참조하는데, 아직 이번 루프의 오차가 계산되지 않음
- 결과적으로 비상정지 판단이 항상 1ms 전 오차를 기준으로 작동

**수정:**
```c
// 수정된 코드 - 오차 먼저 계산, 그 다음 안전 체크
state.current_angle = EncoderReader_GetAngleDeg();
state.error = state.target_angle - state.current_angle;  // 먼저 계산
if (!PositionControl_CheckSafety()) { ... }              // 현재 오차로 판단
```

---

### BUG-05: EmergencyStop 후 영구 잠금

**증상:** 비상정지 후 `PositionControl_Enable()` 재호출해도 제어 안 됨
**발생 위치:** `Core/Src/position_control.c`

**원인:**
```c
// 기존 Enable() 코드 (버그)
int PositionControl_Enable(void) {
    control_enabled = true;
    // fault_flag를 리셋하는 코드 없음!
    // → fault_flag = 1 또는 2 상태 그대로 유지
    ...
}

bool PositionControl_CheckSafety(void) {
    if (fabsf(state.error) > 150.0f) {
        fault_flag = 2;
        return false;  // fault_flag가 남아있어도 이 체크가 재판단함
    }
    return true;  // 조건 해제되면 정상 동작
}
```
- `fault_flag` 자체보다 `CheckSafety()`가 재판단하는 구조라 큰 문제는 없었으나
- `Enable()` 시점에 `fault_flag` 초기화 + Derivative Kick 방지 코드 필요

**수정:**
```c
int PositionControl_Enable(void) {
    control_enabled = true;
    fault_flag = 0;                                          // fault 초기화
    state.current_angle = EncoderReader_GetAngleDeg();
    pid_state.prev_error = state.target_angle - state.current_angle; // D킥 방지
    pid_state.integral   = 0.0f;
    pid_state.last_time_ms = HAL_GetTick();
    printf("[PosCtrl] Enabled (FLT cleared, angle=%.2f)\r\n", state.current_angle);
    return POS_CTRL_OK;
}
```

---

### BUG-06: PID Derivative Kick (미분 킥)

**증상:** `Enable()` 직후 모터가 순간적으로 최대속도로 폭주
**발생 위치:** `Core/Src/position_control.c` → `PID_Calculate()`

**원인:**
```
Enable() 호출 시:
  prev_error = 0 (초기화됨)
  현재 오차 = 10° (target=10°, current=0°)

첫 PID 계산:
  D항 = Kd × (error - prev_error) / dt
      = 20 × (10 - 0) / 0.001
      = 20 × 10000
      = 200,000 Hz  ← output_limit(10,000Hz)로 클램핑되지만
                       순간적으로 최대 출력 발생
```

**수정:**
```c
// Enable() 시점에 현재 오차로 prev_error 초기화
// → 첫 D항 = (error - error) / dt = 0
pid_state.prev_error = state.target_angle - state.current_angle;
```

---

### BUG-07: Safety 임계값 과도 (60° → 150°)

**증상:** PID 제어 중 오버슈트가 60° 넘으면 즉시 비상정지 → 테스트 불가
**발생 위치:** `Core/Src/position_control.c` → `PositionControl_CheckSafety()`

**원인:**
- 초기 임계값 60°는 너무 좁아 PID 튜닝 중 정상적인 오버슈트에도 비상정지 발동

**수정:**
```c
if (fabsf(state.error) > 150.0f) {  // 60° → 150°로 완화
    fault_flag = 2;
    return false;
}
```

---

## 현재 미해결 의심 원인

> 아래 항목들은 코드 수정 후에도 모터가 동작하지 않을 경우 확인해야 할 하드웨어/파라미터 문제입니다.

---

### CHECK-01: P4-03 파라미터 (가장 가능성 높음)

**설명:** L7 드라이브의 펄스 입력 방식 선택 파라미터

| P4-03 값 | 모드 | 설명 |
|---------|------|------|
| 0 (기본) | Open Collector | 24V 풀업 + 트랜지스터 컬렉터 방식 |
| **1** | **Line Driver** | **RS422 차동 신호 방식 (현재 배선에 맞는 설정)** |

**현재 상태:** 확인 안 됨
**확인 방법:** 드라이브 LCD 패널 → P4-03 값 확인 → 1로 설정

---

### CHECK-02: 신호 GND 공통 참조 미연결

**설명:** RS422 통신은 MCU와 드라이브가 GND를 공유해야 정상 동작

```
필요한 연결:
STM32 GND ────────── L7 드라이브 CN1-1 (SG, Signal Ground)
```

**현재 상태:** 연결 여부 미확인
**중요도:** 미연결 시 RS422 수신기가 공통 모드 전압 범위 초과 → 신호 미수신

---

### CHECK-03: RS422 모듈 전원 전압

**설명:** YL-128 모듈 내부 SN75176 칩은 5V 전용

| 공급 전압 | 결과 |
|---------|------|
| 3.3V | 출력 차동 전압 부족 → 드라이브가 신호 인식 못할 수 있음 |
| **5V** | **정상 RS422 레벨 출력** |

**확인 방법:** YL-128 VCC 핀에 연결된 전원 측정 → Nucleo 5V 핀(CN7-18)에 연결되어야 함

---

### CHECK-04: PR- (SIGN 음극) 플로팅

**설명:** 현재 방향 신호(SIGN) 배선 상태

```
현재 배선:
PE11 (GPIO) → YL-128 A단자 → CN1-11 (PR+) : PE11 전압 직접 인가
              YL-128 B단자 → CN1-12 (PR-) : B단자는 수신 입력(고임피던스) → 플로팅!

PR- 플로팅 시:
  차동 전압 = PR+ - PR- = 불확정
  → 드라이브 SIGN 수신기 판단 불가 → 일부 드라이브는 PULS 자체 무시
```

**권장 수정:**
```
CN1-12 (PR-) ── GND 직접 연결
→ PR- = 0V 고정, PR+ = PE11 (3.3V or 0V)
→ 명확한 차동 신호 확보
```

---

## 확인된 사항 요약

| 항목 | 상태 | 결과 |
|------|------|------|
| SVON 활성화 | ✅ 확인 | 모터 뻑뻑함 (서보 록 동작) |
| 드라이브 알람 | ✅ 확인 | 알람 없음, 정상 켜짐 |
| P4-00 (펄스 포맷) | ✅ 확인 | 2 (Pulse+Direction) |
| P2-02 (EMG 기능) | ✅ 확인 | 비활성화 |
| RS422 전원 | ✅ 확인 | 공급됨 |
| PE9 PWM 출력 | ✅ 확인 | 오실로스코프 3.3V 신호 확인 |
| TIM4 (엔코더) | ✅ 확인 | 연결됨 |
| P4-03 (입력 방식) | ❓ 미확인 | **반드시 확인 필요** |
| SG (신호 GND) 연결 | ❓ 미확인 | **반드시 확인 필요** |
| RS422 모듈 5V | ❓ 미확인 | 확인 권장 |
| PR- GND 연결 | ❌ 미연결 | PR- 플로팅 상태 |
