# Embedded C 코드 소유권 회복 계획

작성일: 2026-04-28  
대상: 시스템 SW / 제어 / 임베디드 펌웨어 취업 준비  
기준 프로젝트: `servo_control_baqu`

## 1. 이 문서의 목적

이 문서는 "AI가 코드를 만들고 내가 따라가는 느낌"을 줄이기 위해, 현재 조향 서브컨트롤러 프로젝트에서 반드시 직접 소유해야 할 C 코드 영역과 훈련 순서를 정리한다.

목표는 AI를 끊는 것이 아니다. 목표는 아래 상태를 만드는 것이다.

> AI가 만든 코드라도 내가 읽고, 위험한 부분을 찾고, 하드웨어 동작과 연결해서 설명하고, 핵심 부분은 직접 고칠 수 있는 상태

현업 기준으로 중요한 것은 "모든 코드를 외워서 처음부터 친다"가 아니다. 더 중요한 것은 모르는 코드가 나왔을 때 구조를 따라가고, 데이터시트와 레지스터를 확인하고, 로그/디버거/오실로스코프로 검증할 수 있는 능력이다.

## 2. 지금 가장 큰 문제 정의

현재 문제는 단순히 코딩 속도가 느린 것이 아니다.

핵심 문제는 다음 세 가지다.

1. 코드를 읽고 해석하는 시간이 길다.
2. AI가 만든 구현을 내가 충분히 검증하기 전에 받아들이는 흐름이 생긴다.
3. HAL/CubeMX 기반 코드를 쓰면서도 내부 레지스터 동작을 끝까지 추적하는 경험이 부족하다.

이 문제를 해결하려면 새 프로젝트를 많이 만드는 것보다, 현재 프로젝트의 핵심 C 코드 일부를 내 코드처럼 설명할 수 있게 만드는 것이 훨씬 효율적이다.

## 3. 취업 관점의 한 줄 전략

이 프로젝트의 취업용 전략은 아래처럼 잡는다.

> 조향 제어 프로젝트는 HAL 기반으로 완성도를 높이고, `TIM1 PWM`, `TIM2 encoder`, `state machine`, `safety`, `log evidence`는 C 코드와 레지스터 수준까지 직접 설명할 수 있게 만든다.

즉 레지스터 raw 프로젝트를 별도로 크게 하나 더 만드는 것이 아니라, 현재 프로젝트 안에서 "라이브러리 아래를 확인할 수 있는 사람"이라는 증거를 만든다.

## 4. 반드시 직접 소유해야 할 핵심 20%

아래 파일들은 면접 전에 반드시 직접 읽고 설명할 수 있어야 한다.

| 우선순위 | 파일 | 반드시 설명할 수 있어야 하는 것 |
|---|---|---|
| P0 | `Core/Src/encoder_reader.c` | TIM2 raw counter, 16-bit unwrap, `int16_t` delta 해석, `accum_count`, `offset_count`, motor degree 변환 |
| P0 | `Core/Src/pulse_control.c` | TIM1 PWM 주파수 계산, `PSC/ARR/CCR` 관계, direction GPIO, reverse guard, min/max frequency clamp |
| P0 | `Core/Src/position_control_safety.c` | angle limit, tracking error, velocity/timeout fail-safe 기준 |
| P0 | `Core/Src/position_control.c` | PID 출력, command lifecycle, enable/disable, emergency path |
| P0 | `Core/Src/app_runtime.c` | 1 ms loop, keyboard/UDP input source, CSV/ENCDBG 출력, watchdog refresh |
| P1 | `Core/Inc/project_params.h` | 시험 모드, safety profile, pulse/UDP/CAN/latency 파라미터 ownership |
| P1 | `Core/Inc/constants.h` | gear ratio, encoder scale, steering/motor unit 변환 기준 |
| P1 | `Core/Src/can_runtime.c` | CAN frame 처리, queue/status/send helper, timeout과 로그 |

이 파일들을 모두 완벽히 외울 필요는 없다. 하지만 아래 질문에는 답할 수 있어야 한다.

- 이 함수는 언제 호출되는가?
- 입력값은 무엇인가?
- 내부 상태는 무엇을 저장하는가?
- 하드웨어에 어떤 영향을 주는가?
- 로그나 출력으로 무엇을 확인할 수 있는가?
- 고장 나면 어떤 증상이 나타나는가?

## 5. 이 프로젝트에서 꼭 해야 할 작업

### 5.1 P0: `encoder_reader.c` 직접 해석 및 재구현

반드시 해야 한다.

- `EncoderReader_UpdateCount()`를 종이에 먼저 설명한다.
- `uint16_t raw - prev_raw_count`가 왜 wrap 상황에서도 동작하는지 설명한다.
- `int16_t signed_delta`로 바꾸는 이유를 말할 수 있어야 한다.
- `accum_count`, `offset_count`, `raw_count`, `delta_count`의 차이를 설명한다.
- AI 없이 16-bit encoder unwrap 함수만 따로 직접 작성한다.

직접 작성할 미니 함수 예시:

```c
int32_t unwrap_delta_u16(uint16_t now, uint16_t prev)
{
    return (int32_t)((int16_t)(now - prev));
}
```

완료 기준:

- `raw_count=65530`, `prev=10` 같은 역방향 wrap 예제를 직접 계산할 수 있다.
- `raw_count=10`, `prev=65530` 같은 정방향 wrap 예제를 직접 계산할 수 있다.
- `ENCDBG cnt/delta` 로그가 실제 엔코더 변화와 어떻게 연결되는지 설명할 수 있다.

### 5.2 P0: `pulse_control.c`와 TIM1 PWM 주파수 계산 소유

반드시 해야 한다.

- `PulseControl_GetTimerClockHz()`가 왜 APB2 prescaler를 확인하는지 설명한다.
- PWM 주파수 계산식을 외운다.

```text
pwm_hz = timer_clock_hz / ((PSC + 1) * (ARR + 1))
duty   = CCR1 / (ARR + 1)
```

- `PulseControl_ApplyPwmFrequency()`에서 `period_counts`, `autoreload`, `compare`가 어떻게 결정되는지 설명한다.
- `PULSECONTROL_MIN_FREQ_HZ`, `PULSECONTROL_MAX_FREQ_HZ`가 왜 필요한지 설명한다.
- direction reversal 때 바로 방향을 바꾸지 않고 stop/settle을 두는 이유를 설명한다.

완료 기준:

- 목표 주파수 1000 Hz를 넣었을 때 `PSC/ARR/CCR`가 어떤 의미인지 말할 수 있다.
- TIM1 PWM 출력을 HAL 함수 이름이 아니라 레지스터 관점으로 설명할 수 있다.
- 오실로스코프에서 주파수가 다르면 어떤 변수를 먼저 볼지 말할 수 있다.

### 5.3 P0: 레지스터 raw 레벨 확인 부록 만들기

프로덕션 코드를 전부 레지스터 방식으로 바꾸지 않는다. 대신 "내가 HAL 아래를 확인할 수 있다"는 증거를 만든다.

반드시 만들 부록은 두 개다.

#### TIM1 PWM 레지스터 확인

확인할 레지스터:

- `RCC`: TIM1/GPIOE clock enable
- `GPIOE_MODER`: PE9 alternate function 설정
- `GPIOE_AFR`: PE9 AF1 TIM1 설정
- `TIM1_PSC`: prescaler
- `TIM1_ARR`: auto reload
- `TIM1_CCR1`: duty compare
- `TIM1_CCMR1`: PWM mode
- `TIM1_CCER`: channel output enable
- `TIM1_BDTR`: main output enable
- `TIM1_CR1`: counter enable

산출물:

- `Doc/` 아래에 TIM1 PWM 레지스터 해설 문서 1개
- 가능하면 `Core/Src`가 아닌 별도 `Doc/snippets/` 또는 문서 코드블록에 CMSIS 예제
- 현재 `pulse_control.c`의 HAL 동작과 위 레지스터를 매핑한 표

#### TIM2 Encoder 레지스터 확인

확인할 레지스터:

- `RCC`: TIM2/GPIOA/GPIOB clock enable
- `GPIOA_MODER`, `GPIOB_MODER`: PA0/PB3 alternate function 설정
- `GPIOA_AFR`, `GPIOB_AFR`: TIM2 alternate function 설정
- `TIM2_SMCR`: encoder mode 설정
- `TIM2_CCMR1`: input capture channel mapping
- `TIM2_CCER`: polarity 설정
- `TIM2_CNT`: 현재 counter 값
- `TIM2_ARR`: counter range

산출물:

- `Doc/` 아래에 TIM2 encoder 레지스터 해설 문서 1개
- `encoder_reader.c`의 `__HAL_TIM_GET_COUNTER()`가 결국 `TIM2->CNT`를 읽는다는 설명
- overflow/underflow를 `int16_t` delta로 해석하는 이유

완료 기준:

- 면접에서 "HAL 말고 레지스터로 보면 어디를 봐야 하나요?"라는 질문에 TIM1/TIM2 기준으로 답할 수 있다.
- Reference Manual을 보면서 해당 bit field를 찾을 수 있다.

### 5.4 P0: safety/state machine을 내 말로 설명하기

반드시 해야 한다.

- 현재 startup auto-enable 상태를 설명한다.
- 향후 목표인 `startup -> homing -> ready -> running -> fault/estop` 흐름을 상태도로 그린다.
- `position_control_safety.c`의 fault 조건을 표로 정리한다.
- fault가 발생했을 때 pulse, relay, command lifecycle, log가 어떻게 변해야 하는지 설명한다.

완료 기준:

- "센서가 멈추면 어떻게 알아차리나요?"
- "통신 timeout이면 어떤 상태로 가나요?"
- "watchdog이 있다고 안전한가요?"
- "startup 때 바로 servo on 되는 구조가 왜 위험할 수 있나요?"

위 질문에 코드 파일 이름과 함께 답할 수 있어야 한다.

### 5.5 P1: 작은 C 구현을 매일 손으로 작성

아래 기능은 AI 없이 직접 작성해야 한다.

- 16-bit unwrap
- moving average filter
- timeout checker
- clamp 함수
- ring buffer
- simple state machine
- PID 한 step 계산
- bit set/clear/read macro

중요한 점은 큰 코드를 많이 쓰는 것이 아니다. 작고 정확한 함수를 빠르게 쓰고, 테스트 입력을 스스로 만들어 검증하는 것이다.

## 6. AI 사용 규칙

AI를 금지하지 않는다. 대신 역할을 바꾼다.

### 허용

- "이 함수의 흐름을 설명해줘"
- "내가 쓴 코드에서 버그만 찾아줘"
- "정답 코드는 주지 말고 힌트만 줘"
- "이 HAL 함수가 어떤 레지스터를 만지는지 추적 순서를 알려줘"
- "면접관이라면 이 코드에서 뭘 물어볼지 알려줘"

### 금지

- 내가 이해하지 못한 상태에서 모듈 전체를 AI 코드로 교체
- 컴파일만 되면 바로 수용
- 레지스터/하드웨어 영향 설명 없이 "동작함"으로 처리
- 로그나 테스트 없이 README에 완료라고 적기

### 추천 프롬프트

```text
이 코드를 리뷰해줘. 정답 코드를 바로 주지 말고,
1) 위험한 부분
2) 하드웨어에서 문제 될 수 있는 부분
3) 내가 직접 확인해야 할 레지스터/로그
만 알려줘.
```

```text
내가 직접 구현하려고 한다. 전체 코드는 주지 말고,
함수 입력/출력, 상태 변수, 테스트 케이스만 제안해줘.
```

## 7. 매일 60~90분 루틴

시간이 부족할 때는 아래 루틴만 반복한다.

### 1단계: 읽기 30분

한 파일에서 함수 하나만 고른다.

정리 형식:

```text
함수:
언제 호출:
입력:
내부 상태:
출력/부작용:
하드웨어 영향:
로그로 확인하는 법:
```

### 2단계: 손코딩 30분

선택한 함수의 핵심 아이디어만 작은 함수로 다시 쓴다.

예:

- `EncoderReader_UpdateCount()`를 보고 unwrap 함수만 작성
- `PulseControl_ApplyPwmFrequency()`를 보고 PWM 계산 함수만 작성
- `PositionControl`을 보고 PID 한 step만 작성

### 3단계: 검증 20분

직접 만든 테스트 입력을 넣는다.

예:

```text
prev=65530, now=10
prev=10, now=65530
prev=1000, now=1100
prev=1100, now=1000
```

그 다음 AI에게 코드 리뷰만 시킨다.

## 8. 2주 집중 일정

| 날짜 범위 | 목표 | 산출물 |
|---|---|---|
| 1~2일차 | `encoder_reader.c` 해석 | unwrap 설명 메모, 테스트 케이스 |
| 3~4일차 | `pulse_control.c` 해석 | PWM 주파수 계산표, TIM1 레지스터 매핑 |
| 5일차 | `position_control_safety.c` 해석 | fault 조건표 |
| 6일차 | `app_runtime.c` 1 ms loop 해석 | 입력/제어/로그 흐름도 |
| 7일차 | 복습 및 면접 질문 생성 | 예상 질문 20개 |
| 8~9일차 | TIM1 PWM raw 레지스터 부록 | HAL-to-register 매핑 문서 |
| 10~11일차 | TIM2 encoder raw 레지스터 부록 | HAL-to-register 매핑 문서 |
| 12일차 | ring buffer 또는 timeout checker 직접 구현 | 작은 C 코드와 테스트 |
| 13일차 | safety/state machine 설명 정리 | 상태도와 fault response 표 |
| 14일차 | 면접용 3분 설명 연습 | 프로젝트 설명 스크립트 |

## 9. 면접에서 가져갈 핵심 문장

아래 문장을 자연스럽게 말할 수 있으면 좋다.

> 본 프로젝트는 빠른 bring-up과 유지보수를 위해 STM32 HAL 기반으로 구현했습니다. 다만 TIM1 PWM과 TIM2 encoder는 HAL 내부가 어떤 레지스터를 설정하는지 직접 확인했고, PWM 주파수의 `PSC/ARR/CCR` 관계와 encoder counter unwrap은 직접 설명하고 검증할 수 있습니다.

> AI는 코드 초안 작성과 리뷰 보조로 사용했지만, `encoder_reader`, `pulse_control`, `position_control_safety`, `app_runtime`의 핵심 흐름은 제가 직접 읽고 테스트 케이스로 검증했습니다.

> 제어 프로젝트에서 중요한 것은 목표각을 한 번 움직이는 것이 아니라, 센서 truth, fault 조건, timeout, startup safety, 로그 evidence까지 닫는 것이라고 보고 그 부분을 중심으로 보완하고 있습니다.

## 10. 완료 체크리스트

아래 항목이 체크되면 "AI가 만든 코드를 따라가는 상태"에서 벗어났다고 볼 수 있다.

- [ ] `encoder_reader.c`의 unwrap 원리를 예제 숫자로 설명할 수 있다.
- [ ] `pulse_control.c`의 PWM 주파수 계산을 `PSC/ARR/CCR`로 설명할 수 있다.
- [ ] TIM1 PWM 관련 레지스터 이름과 역할을 말할 수 있다.
- [ ] TIM2 encoder 관련 레지스터 이름과 역할을 말할 수 있다.
- [ ] `position_control_safety.c`의 fault 조건을 표로 설명할 수 있다.
- [ ] `app_runtime.c`의 1 ms loop에서 입력, 제어, 로그, watchdog 순서를 설명할 수 있다.
- [ ] AI 없이 작은 C 함수 8개를 작성해봤다.
- [ ] AI에게 정답 코드를 받기 전에 내 코드 리뷰를 시키는 습관이 생겼다.
- [ ] 로그 한 줄을 보고 어느 모듈에서 나온 것인지 추적할 수 있다.
- [ ] 면접용 3분 설명에서 "HAL을 썼지만 내부 레지스터도 확인했다"를 구체적으로 말할 수 있다.

## 11. 우선순위 결론

지금 새 프로젝트를 크게 늘리지 않는다.

가장 먼저 할 일은 현재 프로젝트의 핵심 코드 소유권을 되찾는 것이다.

1. `encoder_reader.c`
2. `pulse_control.c`
3. `position_control_safety.c`
4. `app_runtime.c`
5. TIM1/TIM2 레지스터 확인 부록

이 다섯 가지가 끝나면, 이 프로젝트는 단순히 "AI로 만든 STM32 프로젝트"가 아니라 "내가 구조와 하드웨어 동작을 설명하고 검증할 수 있는 제어 펌웨어 프로젝트"가 된다.
