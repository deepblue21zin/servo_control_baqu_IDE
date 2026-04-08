# STM32CubeMonitor Debug Setup Guide

이 문서는 현재 `servo_control_baqu` 프로젝트에서 `STM32CubeMonitor`를 처음 붙일 때 필요한 최소 절차를 정리한다.

## 1. 현재 프로젝트 기준 핵심 정보

- MCU: `STM32F429ZI`
- Encoder feedback: `TIM2`
- Pulse output: `TIM1_CH1 -> PE9`
- Direction output: `PE10`
- Debug UART: `USART3` (`PD8/PD9`, ST-LINK VCP)

이번 반영으로 아래 디버그 변수는 이미 코드에 추가되어 있다.

- `dbg_enc_raw`
- `dbg_pos_mdeg`
- `dbg_target_mdeg`
- `dbg_err_mdeg`
- `dbg_pwm_cmd`
- `dbg_fault_flags`

파일 위치:

- `Core/Inc/debug_vars.h`
- `Core/Src/debug_vars.c`
- `Core/Src/position_control_diag.c`

단위는 아래처럼 해석하면 된다.

- `dbg_target_mdeg`, `dbg_pos_mdeg`, `dbg_err_mdeg`
  - 조향축 기준 `mdeg` (`0.001 deg`)
- `dbg_pwm_cmd`
  - 제어기 출력 주파수 명령의 signed debug 값
- `dbg_enc_raw`
  - `TIM2` raw counter (`0~65535`)
- `dbg_fault_flags`
  - 비트 플래그

## 2. DBG_LOOP 핀은 왜 아직 직접 안 넣었는가

`DBG_LOOP`는 CubeMX가 생성하는 `main.h`와 `gpio.c`에 핀 매크로를 추가해야 하므로, 핀 선택은 사용자가 `.ioc`에서 해주는 것이 가장 안전하다.

현재 코드에는 아래 조건부 훅이 이미 들어가 있다.

- `DBG_LOOP_Pin`
- `DBG_LOOP_GPIO_Port`

즉, `.ioc`에서 `DBG_LOOP`라는 label로 GPIO output 핀을 하나 추가하고 코드 재생성을 하면, `PositionControl_Update()` 시작/끝에서 자동으로 토글되게 준비해두었다.

## 3. CubeMX(.ioc)에서 DBG_LOOP 설정 방법

### 3.1 핀 고를 때 피해야 할 것

현재 프로젝트에서 이미 쓰는 핀은 피해야 한다.

- `PE9`: pulse output
- `PE10`: direction output
- `PA0/PB3`: encoder A/B
- `PD14/PD15`: SVON / EMG
- `PD8/PD9`: debug UART
- `PB0/PB7`: line driver enable
- RMII Ethernet 핀들

특히 `PB0`는 이미 `LD_DE`로 쓰고 있으므로 `DBG_LOOP`로 쓰면 안 된다.

### 3.2 설정 순서

1. `servo_control_baqu.ioc`를 연다.
2. `Pinout & Configuration` 화면으로 간다.
3. 보드에서 물리적으로 찍을 수 있는 남는 GPIO를 하나 고른다.
4. 해당 핀을 클릭해서 `GPIO_Output`으로 설정한다.
5. `System Core -> GPIO` 또는 핀 설정 창에서 `User Label`을 `DBG_LOOP`로 입력한다.
6. `Project -> Generate Code`를 눌러 코드 재생성한다.

### 3.3 재생성 후 확인할 것

아래 두 파일에 `DBG_LOOP`가 생겼는지 확인한다.

- `Core/Inc/main.h`
- `Core/Src/gpio.c`

정상이라면 `main.h`에 아래와 비슷한 매크로가 생긴다.

```c
#define DBG_LOOP_Pin GPIO_PIN_x
#define DBG_LOOP_GPIO_Port GPIOX
```

## 4. CubeIDE에서 먼저 확인할 것

CubeMonitor보다 먼저 CubeIDE Watch에서 아래 변수들이 실제로 변하는지 본다.

- `dbg_enc_raw`
- `dbg_pos_mdeg`
- `dbg_target_mdeg`
- `dbg_err_mdeg`
- `dbg_pwm_cmd`
- `dbg_fault_flags`

정상 패턴:

- 엔코더를 돌리면 `dbg_enc_raw`가 변한다.
- 목표를 바꾸면 `dbg_target_mdeg`가 변한다.
- 제어가 맞으면 `dbg_err_mdeg`가 0 쪽으로 줄어든다.
- 제어 출력을 내면 `dbg_pwm_cmd`가 변한다.

## 5. STM32CubeMonitor 설정 순서

### 5.1 준비

1. 펌웨어를 빌드하고 다운로드한다.
2. CubeIDE 디버그 세션은 종료한다.
3. 보드는 실행 상태로 둔다.

중요:

- 처음에는 `CubeIDE debugger`와 `CubeMonitor`를 동시에 붙이지 않는다.
- `CubeMonitor`만 단독으로 붙인다.

### 5.2 ELF에서 변수 가져오기

1. `STM32CubeMonitor` 실행
2. `myVariables` 열기
3. `Add new exe config` 또는 편집 아이콘 클릭
4. 프로젝트 `.elf` 파일 선택
5. 아래 변수만 먼저 체크
   - `dbg_target_mdeg`
   - `dbg_pos_mdeg`
   - `dbg_err_mdeg`
   - `dbg_pwm_cmd`
   - `dbg_enc_raw`
   - `dbg_fault_flags`
6. `Add` 또는 `Done`

처음에는 변수를 많이 넣지 않는 것이 좋다.

### 5.3 Probe 설정

1. `myProbeOut` 열기
2. `ST-LINK` 선택
3. `Add`
4. `Done`
5. `myProbeIn` 열기
6. 방금 추가한 probe 선택
7. `Done`
8. `Deploy`

### 5.4 Dashboard에서 시작

1. `Dashboard`로 이동
2. 차트 위젯을 추가
3. 아래 4개를 먼저 올린다.
   - `dbg_target_mdeg`
   - `dbg_pos_mdeg`
   - `dbg_err_mdeg`
   - `dbg_pwm_cmd`
4. `START` 클릭

처음 목표는 이것만 보면 된다.

- 목표각과 실제각이 따라오는지
- 오차가 줄어드는지
- 출력이 포화되는지
- 방향이 맞는지

## 6. 이 프로젝트에서 추천하는 해석 방법

CubeMonitor는 상태 추세를 보는 도구로 쓰는 것이 좋다.

- `CubeMonitor`
  - target / pos / error / pwm / fault 추세
- `DBG_LOOP + Logic Analyzer`
  - 제어 주기
  - 함수 실행 시간
  - pulse/direction glitch

즉, `CubeMonitor`를 오실로스코프처럼 쓰지 말고, 상태 관찰용으로 쓰는 것이 맞다.

## 7. 빠른 첫 테스트 추천

현재 프로젝트는 `APP_RUNTIME_INPUT_SOURCE = APP_RUNTIME_INPUT_SOURCE_KEYBOARD` 기준으로 UDP 없이도 벤치 테스트가 가능하다.

권장 순서:

1. UART로 `A/D` 키 입력으로 목표각 변경
2. 또는 UART에 `5`, `-3.5`, `0`처럼 숫자를 입력하고 `Enter`로 절대 목표각 지정
2. CubeMonitor에서 `dbg_target_mdeg`, `dbg_pos_mdeg`, `dbg_err_mdeg` 관찰
3. `DBG_LOOP` 핀을 logic analyzer로 확인
4. 필요 시 `DIR`, `PULSE`, `ENC_A`, `ENC_B`까지 같이 측정

## 8. 체크리스트

- `debug_vars` 값이 Watch에서 먼저 변하는가
- `CubeMonitor`가 `.elf`에서 변수를 정상 인식하는가
- `dbg_target_mdeg`와 `dbg_pos_mdeg`가 같은 축 기준으로 해석되는가
- `dbg_err_mdeg`가 step 후 줄어드는가
- `dbg_fault_flags`가 한계 조건에서 켜지는가
- `DBG_LOOP` 주기가 일정한가
