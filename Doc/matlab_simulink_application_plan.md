# MATLAB / Simulink 적용 계획

작성일: 2026-03-30  
대상 프로젝트: `servo_control_baqu`

## 1. 이 문서의 목적

이 문서는 현재 `servo_control_baqu` 프로젝트에 MATLAB / Simulink를 붙일 때, 무엇을 어떤 순서로 해야 하는지 실행 계획 형태로 정리한 문서다.

핵심 목적은 아래 3개다.

1. 현재 펌웨어를 무너지지 않게 유지하면서 host-side 분석 환경을 먼저 만든다.
2. 제어 알고리즘을 Simulink shadow 모델과 export-function 코드 생성 경로로 분리한다.
3. STM32 Monitor & Tune / PIL은 별도 샌드박스에서 진행해 기존 CubeIDE runtime과 충돌하지 않게 한다.

## 2. 결론 먼저

현재 프로젝트에는 **세 가지 모델 + 두 가지 스크립트** 전략이 가장 적합하다.

| 이름 | 역할 | 보드 연결 | 바로 시작 가능 여부 | 비고 |
|---|---|---|---|---|
| `baqu_params_fw.m` | 현재 펌웨어 truth 파라미터 | 없음 | 가능 | 현재 코드와 1:1 기준 |
| `baqu_params_design.m` | 설계 목표 파라미터 | 없음 | 가능 | future target / safety 실험용 |
| `import_putty_csv.m` | 기존 PuTTY / CSV 로그를 MATLAB로 읽기 | 없음 | 가능 | 현재 펌웨어 passive monitoring |
| `baqu_shadow.slx` | 로그 재생, 제어 가설 검증, 파라미터 스윕 | 없음 | 가능 | 가장 먼저 만들 모델 |
| `baqu_ctrl_export.slx` | 상위 제어 알고리즘만 export-function 코드 생성 | 없음 | 가능 | 현재 구조와 가장 잘 맞음 |
| `baqu_target_monitor.slx` | STM32 실시간 Monitor & Tune / External Mode 실험 | 있음 | 나중 | 별도 `.ioc` 샌드박스 필요 |

즉, **현재 펌웨어 모니터링**과 **Simulink External Mode**는 같은 일이 아니다.

- 현재 펌웨어를 그대로 보고 싶으면 `import_putty_csv.m` 또는 host-side Simulink 입력 블록으로 간다.
- Simulink External Mode / Monitor & Tune는 **Simulink가 생성한 타깃 코드**를 대상으로 한다.

## 3. 현재 코드 기준으로 꼭 알고 시작해야 하는 제약

### 3.1 기존 runtime이 이미 강하게 잡혀 있다

현재 프로젝트는 아래 흐름으로 동작한다.

- `main.c` -> `AppRuntime_Init()` -> `while (1) AppRuntime_RunIteration()`
- 1 ms fast tick 내부에서 controller / pulse / encoder / safety가 수행된다.
- 저수준 I/O는 이미 CubeIDE 쪽에 있다.

즉, Simulink가 전체 보드 스케줄러를 먹는 형태보다 **제어 알고리즘만 export-function으로 뽑는 방식**이 더 자연스럽다.

### 3.2 현재 파라미터 체계에는 스케일 혼선이 있다

현재 코드 기준으로 아래 값은 이미 확정적으로 보인다.

- `ENCODER_PPR = 12000`
- `ENCODER_QUADRATURE = 4`
- `ENCODER_COUNT_PER_REV = 48000`
- `DEG_PER_PULSE = 0.003`
- `MAX_PULSE_FREQ = 1000000`
- 실제 `pulse_control.c` clamp는 `100000 Hz`

따라서 MATLAB 파라미터에서 아래 둘은 분리해야 한다.

1. `encoder count per motor rev`
2. `command pulse per motor rev`

이 둘을 같은 변수로 쓰면 현재 firmware와 drive baseline이 섞여 버린다.

### 3.3 startup auto-enable이 아직 남아 있다

현재 baseline은 startup에서 아래가 자동으로 실행된다.

- `Relay_ServoOn()`
- `EncoderReader_Reset()`
- `PositionControl_SetTargetWithSource(...0 deg...)`
- `PositionControl_Enable()`

즉, Simulink 적용 문서에서도 startup / ready / arm / estop을 **현재 펌웨어와 동일한 값**과 **설계 목표값**으로 분리해서 봐야 한다.

### 3.4 UART3는 이미 현재 펌웨어가 사용 중이다

현재 프로젝트는 `huart3`를 아래 용도로 이미 사용한다.

- keyboard input
- startup 메시지
- CSV / debug / KB 로그

따라서 같은 이미지에서 `USART3`를 External Mode serial과 동시에 쓰는 것은 추천하지 않는다.

### 3.5 현재 `.ioc`는 Simulink 샌드박스로 바로 재사용하기엔 위험하다

현재 `servo_control_baqu.ioc`는 일반 CubeIDE 앱 기준이다.

- `ProjectManager.NoMain=false`
- `ProjectManager.UnderRoot=true`

Simulink target monitor 실험은 **기존 `.ioc` 복사본을 따로 만들어 샌드박스로 쓰는 것**이 안전하다.

## 4. 추천 목표 구조

### 4.1 Goal A: 현재 펌웨어를 MATLAB에서 보고 싶다

이 목표의 권장 방식은 **host-side passive monitoring**이다.

사용 경로:

1. PuTTY / CSV 로그를 `putty.log`로 저장
2. MATLAB `import_putty_csv.m`로 읽기
3. 테이블 / plot / signal reconstruction 수행

여기서는 Simulink External Mode가 필요 없다.

### 4.2 Goal B: 제어기 수학을 Simulink에서 먼저 검증하고 싶다

이 목표의 권장 방식은 **`baqu_shadow.slx`**다.

여기서는 보드 연결 없이 아래를 검증한다.

- 단위 체계
- target -> motor 변환
- error 계산
- pulse saturation
- timeout / fault 가설
- 파라미터 스윕

### 4.3 Goal C: 제어 알고리즘만 코드 생성하고 싶다

이 목표의 권장 방식은 **`baqu_ctrl_export.slx` export-function**이다.

여기서는 CubeIDE가 계속 아래를 맡는다.

- TIM1 pulse
- TIM2 encoder
- GPIO direction
- relay active-low 반전
- UART / Ethernet / logging

Simulink는 아래만 맡는다.

- target/current/error 계산
- PID / feedforward / saturation
- diagnostic flag
- estop request

### 4.4 Goal D: STM32에서 Simulink Monitor & Tune를 써보고 싶다

이 목표의 권장 방식은 **`baqu_target_monitor.slx` + 별도 샌드박스 `.ioc`**다.

이건 production firmware를 대체하는 실험용 타깃 앱으로 생각해야 한다.

## 5. 설치 계획

## 5.1 필수 설치

| 항목 | 필요 이유 |
|---|---|
| MATLAB | 기본 실행 환경 |
| Simulink | 모델 작성 |
| Simulink Coder | 모델 코드 생성 |
| Embedded Coder | `ert.tlc`, export-function, PIL |
| Embedded Coder Support Package for STMicroelectronics STM32 Processors | STM32 배포, External Mode, Monitor & Tune, PIL |

## 5.2 선택 설치

| 항목 | 언제 필요한가 |
|---|---|
| MATLAB Coder | MATLAB 함수 단위 코드 생성 / 일부 PIL 흐름을 별도로 쓸 때 |
| Stateflow | startup / ready / estop 상태기계를 모델링하고 싶을 때 |

## 5.3 설치 후 가장 먼저 확인할 것

1. Add-On Manager에서 STM32 support package가 실제 설치되었는지 확인한다.
2. Hardware setup을 실행한다.
3. Library Browser에서 STM32 블록이 보이는지 확인한다.

권장 진입:

- `stm32cube.tools.launchHardwareSetup`
- `slLibraryBrowser`

## 6. 저장소 안에서 추천하는 파일 배치

아래처럼 `Matlab/` 폴더를 따로 두는 것을 권장한다.

```text
Matlab/
  baqu_params_fw.m
  baqu_params_design.m
  import_putty_csv.m
  baqu_shadow.slx
  baqu_ctrl_export.slx
  baqu_target_monitor.slx
  docs/
    signal_map.md
    tuning_log.md
```

현재 turn에서는 문서만 추가하고, 실제 `Matlab/` 폴더와 파일 생성은 다음 단계 작업으로 본다.

## 7. 단계별 실행 계획

## 7.1 Stage 0: 파라미터 truth 정리

가장 먼저 해야 하는 일은 파라미터를 **current firmware 기준**과 **design target 기준**으로 분리하는 것이다.

### 산출물

- `baqu_params_fw.m`
- `baqu_params_design.m`

### 최소 포함 변수

```matlab
Ts = 0.001;

STEER_MAX_DEG = 45;
STEER_MIN_DEG = -45;
GEAR_RATIO = 12.5;

ENCODER_PPR = 12000;
ENCODER_QUAD = 4;
ENCODER_COUNT_PER_MOTOR_REV = ENCODER_PPR * ENCODER_QUAD;

FW_DEG_PER_PULSE = 0.003;
FW_CMD_PULSE_PER_MOTOR_REV = 360 / FW_DEG_PER_PULSE;

PULSE_FREQ_FW_MAX_HZ = 100000;
PULSE_FREQ_DRIVE_MAX_HZ = 1000000;
```

### 완료 기준

1. 현재 코드 상수와 MATLAB 상수가 1:1로 대응된다.
2. "현재 firmware 기준"과 "향후 설계 목표"가 같은 파일 안에서 섞이지 않는다.

## 7.2 Stage 1: 현재 펌웨어 passive monitoring부터 시작

이 단계는 Simulink보다 **MATLAB host 분석**이 먼저다.

### 이유

- 지금 펌웨어는 이미 CSV / KB / ENCDBG 로그를 잘 남긴다.
- 이 단계는 보드 코드 변경 없이 시작 가능하다.
- External Mode를 쓰지 않아도 바로 의미 있는 분석이 가능하다.

### 할 일

1. `putty.log` 또는 CSV 로그 샘플 확보
2. `import_putty_csv.m` 작성
3. 아래 신호를 table로 재구성

- `ms`
- `target_deg`
- `current_deg`
- `error_deg`
- `req_hz`
- `applied_hz`
- `cmd_id`
- `cmd_state`
- `cmd_result`
- 필요 시 `ENCDBG cnt/delta`

### 완료 기준

1. MATLAB에서 기존 펌웨어 로그를 바로 plot할 수 있다.
2. target/current/error/frequency overlay가 재현된다.
3. fault case와 normal case를 같은 축에서 비교할 수 있다.

## 7.3 Stage 2: `baqu_shadow.slx`

이 모델은 **보드 연결 없이** 만든다.

### 목표

1. 단위 체계 검증
2. 제어 수학 검증
3. saturation / timeout / estop 가설 검증
4. 파라미터 sweep

### 기본 설정

| 항목 | 값 |
|---|---|
| Solver Type | Fixed-step |
| Solver | discrete |
| Fixed-step size | `0.001` |
| Stop time | `10` 또는 `inf` |
| Signal logging | ON |
| Save to workspace | `logsout` 권장 |

### 추천 신호 이름

- `target_steer_deg`
- `target_motor_deg`
- `current_motor_deg`
- `error_motor_deg`
- `desired_pulse_freq_hz`
- `fault_active`
- `ready_state`

### 이 단계에서 하지 말 것

1. PWM edge 생성
2. GPIO 토글
3. active-low relay 반전
4. STM32 peripheral 블록으로 pulse/encoder 자체를 재구현

### 완료 기준

1. 10 deg steering 입력 시 motor 변환이 125 deg로 맞는다.
2. saturation이 현재 `100000 Hz` 기준으로 동작한다.
3. timeout / estop 입력 시 1 tick 안에 출력 0이 되는지 볼 수 있다.

## 7.4 Stage 3: `baqu_ctrl_export.slx`

이 단계가 현재 코드베이스와 가장 잘 맞는 Simulink 적용 방식이다.

### 핵심 원칙

- Simulink는 제어 수학만 생성한다.
- CubeIDE는 hardware I/O와 scheduler를 계속 유지한다.

### 추천 Inport

| 이름 | 단위 | 현재 프로젝트 연결 원천 |
|---|---|---|
| `current_motor_deg` | deg | `EncoderReader_GetAngleDeg()` |
| `target_motor_deg` | deg | 현재 target state |
| `enable_req` | bool | enable 상태 |
| `ready_in` | bool | startup / ready gate |
| `alarm_in` | bool | drive / fault 입력 |
| `estop_in` | bool | relay / emg 상태 |
| `rx_age_ms` | ms | 통신 age |
| `dt_s` | s | `0.001` |

### 추천 Outport

| 이름 | 단위 | 현재 프로젝트 연결 대상 |
|---|---|---|
| `desired_pulse_freq_hz` | Hz | `PulseControl_SetFrequency()` |
| `estop_request` | bool | `Relay_Emergency()` 또는 상위 fault manager |
| `diag_flags` | bitfield | CSV / UART / debug 변수 |

### 모델 설정

| 항목 | 값 |
|---|---|
| Solver | Fixed-step / discrete |
| System target file | `ert.tlc` |
| Execution domain | Export function |

### 통합 원칙

1. generated code는 adapter layer를 통해 기존 runtime에 연결한다.
2. Simulink generated API 이름은 wrapper에서 감싼다.
3. generated file을 바로 하드웨어 레이어와 섞지 않는다.

### 완료 기준

1. generated controller가 기존 1 ms fast tick 안에서 호출된다.
2. TIM1 / TIM2 / GPIO / relay 코드는 기존 C를 유지한다.
3. generated controller를 꺼도 기존 runtime 구조가 무너지지 않는다.

## 7.5 Stage 4: `baqu_target_monitor.slx`

이 단계는 Monitor & Tune / External Mode 실험용이다.

### 가장 중요한 원칙

현재 `servo_control_baqu.ioc`를 바로 쓰지 말고 **복사본**을 만든다.

추천 이름:

- `servo_control_baqu_simulink_sandbox.ioc`

### 샌드박스에서 할 일

1. Simulink target monitor용 `.ioc` 복사본 생성
2. CubeMX 설정 정리
3. External Mode serial UART 선정
4. Monitor & Tune 신호만 최소화

### Monitor & Tune에서 봐야 하는 신호

- `target_steer_deg`
- `current_motor_deg`
- `error_motor_deg`
- `desired_pulse_freq_hz`
- `applied_pulse_freq_hz`
- `command_state`
- `ready_state`
- `alarm_state`
- `estop_req`

### Monitor & Tune에서 보지 말아야 하는 신호

- raw pulse edge
- raw direction edge
- microsecond 단위 edge-level trace
- ISR마다 바뀌는 fast GPIO waveform

### 완료 기준

1. summary signal을 실시간으로 본다.
2. tunable parameter 변경이 정상 반영된다.
3. UART / XCP link가 안정적으로 유지된다.

## 7.6 Stage 5: PIL + CubeIDE Debug

이 단계는 제일 마지막이다.

### 권장 순서

1. subsystem 단위 PIL
2. referenced model PIL
3. top model 확장

### CubeIDE 디버그 원칙

1. Debug build 사용
2. Simulink generated artifact를 별도 build output으로 관리
3. 현재 production runtime과 디버깅 샌드박스를 분리

### 완료 기준

1. shadow vs PIL 수치 차이가 거의 0에 가깝다.
2. generated code 내부 변수와 기존 adapter 연결을 CubeIDE에서 추적할 수 있다.

## 8. 절대 하지 말아야 할 것

1. current production firmware에 External Mode를 "그냥 attach"하려고 시도하지 말 것
2. shadow 모델 안에서 PWM / encoder peripheral을 직접 다시 구현하지 말 것
3. active-low relay 논리를 모델 내부에 그대로 넣지 말 것
4. `fw_as_is`와 `design_target` 파라미터를 한 파일에 섞지 말 것
5. 현재 `USART3`를 쓰는 기존 로그/키보드 이미지와 External Mode serial을 같은 바이너리에서 동시에 기대하지 말 것

## 9. 실제 추천 시작 순서

### 이번 주에 할 일

1. `Doc/matlab_simulink_application_plan.md` 검토
2. `baqu_params_fw.m` 설계
3. `import_putty_csv.m` 설계
4. `baqu_shadow.slx` 생성
5. steering 0 / 5 / 10 deg step shadow test

### 다음 주에 할 일

1. `baqu_ctrl_export.slx` 생성
2. export-function generated code 구조 점검
3. adapter layer 설계
4. sandbox `.ioc` 복사본 준비

### 그 다음에 할 일

1. `baqu_target_monitor.slx`
2. External Mode / Monitor & Tune
3. subsystem PIL
4. CubeIDE debug

## 10. 체크리스트

### 문서 체크

- [ ] current firmware 파라미터와 design target 파라미터가 분리되었는가
- [ ] shadow / export / target monitor의 역할이 서로 섞이지 않는가
- [ ] UART / XCP / logging 경계가 문서에 적혀 있는가

### 모델 체크

- [ ] 모든 신호 이름에 단위가 붙는가
- [ ] fixed-step 1 ms가 유지되는가
- [ ] saturation과 timeout 기준이 명시되어 있는가

### 통합 체크

- [ ] generated controller가 hardware layer를 직접 만지지 않는가
- [ ] existing CubeIDE scheduler와 충돌하지 않는가
- [ ] sandbox `.ioc`와 production `.ioc`가 분리되었는가

## 11. 공식 참고 링크

- MathWorks STM32 support package overview  
  <https://www.mathworks.com/help/ecoder/stm32-spkg.html>
- Getting Started with STM32CubeMX and Simulink  
  <https://www.mathworks.com/help/ecoder/stmicroelectronicsstm32f4discovery/ug/Getting-started-stm32cubemx.html>
- Configure STM32 Processor-Based Boards using STM32CubeMX  
  <https://www.mathworks.com/help/ecoder/stmicroelectronicsstm32f4discovery/ug/STM32-CubeMX-Configuration.html>
- Monitor & Tune example for STM32  
  <https://la.mathworks.com/help/ecoder/stmicroelectronicsstm32f4discovery/ug/Monitor-Tune-example.html>
- PIL and External Mode example  
  <https://www.mathworks.com/help/ecoder/stmicroelectronicsstm32f4discovery/ug/code-verification-and-validation-with-pil-and-external-mode.html>
- Debug generated code in STM32CubeIDE  
  <https://www.mathworks.com/help/ecoder/stmicroelectronicsstm32f4discovery/ug/stm32cubeide.html>
- Export-function code generation  
  <https://www.mathworks.com/help/simulink/ug/generate-code-for-export-function-model-with-rate-based-model.html>

## 12. 한 줄 요약

현재 `servo_control_baqu`에 MATLAB / Simulink를 붙일 때 가장 안전하고 효과적인 경로는
**host-side log analysis -> shadow model -> export-function controller codegen -> sandbox target monitor -> PIL** 순서다.
