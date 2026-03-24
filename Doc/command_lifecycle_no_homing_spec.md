# Command Lifecycle Spec (No Homing)

작성일: 2026-03-22  
적용 대상: `servo_control_baqu`  
적용 범위: `homing` 기구/시퀀스가 아직 없는 상태에서, 현재 steering runtime에 바로 적용 가능한 명령 lifecycle 규격

## 1. 목적

이 문서는 현재 시스템에서 "목표 조향 명령 하나가 시작돼서 완료됐다"를 코드와 로그 기준으로 정의하기 위한 명세서다.

여기서 핵심은 단순히 `PositionControl_SetTarget()`이 호출되었다는 사실이 아니라:

1. 어떤 명령이 수락되었는지
2. 언제 시작됐는지
3. 지금 진행 중인지
4. 목표에 도달했는지
5. timeout / estop / fault로 끝났는지

를 시스템 수준에서 구분 가능하게 만드는 것이다.

본 문서는 `homing`이 아직 없는 상태를 전제로 하며, `homing complete`는 readiness 조건에 포함하지 않는다.

## 2. `command_id` 정의

### 2.1 정의

- `command_id`는 "수락된 조향 명령의 순번"이다.
- 타입은 `uint32_t`를 사용한다.
- 시스템 부팅 후 첫 번째로 수락된 명령은 `1`, 그 다음은 `2`로 증가한다.

### 2.2 무엇이 `command_id`를 증가시키는가

다음 조건을 모두 만족할 때 새로운 `command_id`를 발급한다.

1. 명령 source가 `UDP`, `Keyboard`, `Service`, `LocalTest` 중 하나로 식별된다.
2. 목표값이 허용 범위 내에 있다.
3. 시스템이 `READY_FOR_COMMAND` 조건을 만족한다.
4. 이전 명령이 `IDLE`, `REACHED`, `TIMEOUT`, `ABORTED`, `FAULTED` 중 하나로 종료됐거나, 현재 진행 중인 명령을 새 명령으로 교체하기로 정책이 정해졌다.

### 2.3 무엇이 `command_id`를 증가시키지 않는가

- 주기적 제어 루프 1회 실행
- CSV 출력
- latency 로그 출력
- 상태 조회 (`GET STATUS`)
- 동일 명령의 내부 재시도

### 2.4 명령 교체 정책

현재 진행 중(`ACTIVE`)인 명령 중 새 목표가 들어오면 기존 명령은 아래 규칙으로 종료한다.

- 기존 명령 상태: `ABORTED`
- 기존 명령 종료 이유: `REPLACED`
- 새 명령: 새로운 `command_id` 발급 후 `ACTIVE` 시작

즉, `command_id`는 "몇 번째 루프"가 아니라 "몇 번째 유효 명령"을 뜻한다.

## 3. Homing 제외 상태에서의 Ready 조건

현재 단계에서 명령 수락 가능(`READY_FOR_COMMAND`) 조건은 다음과 같이 정의한다.

### 3.1 필수 조건

- `PositionControl_Init()` 완료
- `EncoderReader_Init()` 완료
- `PositionControl`이 disabled 상태가 아님
- `CTRL_MODE_EMERGENCY` 상태가 아님
- 통신 명령 기반인 경우 RX timeout 상태가 아님
- target 값이 `steering_deg` 기준 허용 범위 내
- emergency relay가 release 가능한 정상 상태

### 3.2 아직 제외하는 조건

다음은 추후 `homing` 단계에서 readiness에 추가한다.

- homing 완료 여부
- absolute zero synchronization 완료 여부
- startup mechanical zero validation

즉 현재 단계에서는 "절대 원점 기준 준비 완료"가 아니라 "상대 위치 기반 bench command 수행 가능"만 readiness로 본다.

## 4. 명령 상태 정의

### 4.1 상태 enum

아래 상태를 추가한다.

- `CMD_IDLE`
- `CMD_ACTIVE`
- `CMD_REACHED`
- `CMD_TIMEOUT`
- `CMD_ABORTED`
- `CMD_FAULTED`

### 4.2 상태 의미

- `CMD_IDLE`: 현재 유효한 진행 명령 없음
- `CMD_ACTIVE`: 목표를 수락했고 실제 추종 중
- `CMD_REACHED`: 목표 도달 및 유지 조건 만족
- `CMD_TIMEOUT`: 제한 시간 내 도달 실패
- `CMD_ABORTED`: ESTOP, disable, command replace, operator cancel 등으로 중단
- `CMD_FAULTED`: 안전 조건 위반 또는 센서/제어 이상으로 실패 종료

## 5. 명령 시작 조건

### 5.1 시작 시점

아래 순간을 `CMD_START`로 정의한다.

- target가 정상 수락됨
- `command_id` 발급됨
- 내부 target이 갱신됨
- 상태가 `CMD_ACTIVE`로 전이됨

### 5.2 시작 시 기록해야 할 값

- `command_id`
- `source`
- `target_steering_deg`
- `target_motor_deg`
- `start_ms`
- `start_current_steering_deg`
- `start_error_deg`
- `start_mode`

## 6. 명령 완료 조건

현재 시스템에서는 아래 조건을 만족하면 `CMD_REACHED`로 본다.

### 6.1 기본 완료 조건

- `abs(error_steering_deg) <= 0.5 deg`
- 위 조건이 `100 ms` 이상 유지
- `fault_flag == 0`
- `CTRL_MODE_EMERGENCY`가 아님
- `CMD_REACHED` 전이 직후 `PulseControl_Stop()`으로 pulse 출력을 정지하고, 새 명령이 수락되기 전까지 완료 상태를 유지

### 6.2 권장 추가 조건

완료 판정을 더 안정적으로 하려면 아래도 함께 넣는 것이 좋다.

- `abs(output_hz) <= COMPLETE_OUTPUT_THRESHOLD_HZ`
- direction toggle이 마지막 완료 구간에서 발생하지 않음
- encoder raw 또는 current angle이 마지막 도달 구간에서 비정상 jump를 보이지 않음

## 7. 실패 종료 조건

### 7.1 Timeout

아래를 만족하면 `CMD_TIMEOUT`

- 상태가 `CMD_ACTIVE`
- `now_ms - start_ms > command_timeout_ms`
- 아직 `CMD_REACHED` 조건 미만족

초기 bench 권장값:

- `command_timeout_ms = 3000`

### 7.2 Abort

아래 중 하나면 `CMD_ABORTED`

- operator `disable`
- `ESTOP`
- comm timeout으로 mode가 `ESTOP` 전이
- 새 명령 수락으로 기존 명령 교체

### 7.3 Fault

아래 중 하나면 `CMD_FAULTED`

- angle over-limit
- tracking error over-limit
- encoder invalid or not initialized
- 추후: encoder stuck, sensor mismatch, implausible motion

## 8. 로그 형식

### 8.1 필수 이벤트 로그

다음 5개는 최소한 남겨야 한다.

```text
CMD_START,id=17,src=UDP,target_deg=5.0,target_motor_deg=60.0,start_ms=12345
CMD_REACHED,id=17,end_ms=12780,settling_ms=435,final_error_deg=0.12
CMD_TIMEOUT,id=18,end_ms=15900,elapsed_ms=3000,error_deg=3.40
CMD_ABORT,id=19,reason=ESTOP,end_ms=16200
CMD_FAULT,id=20,reason=TRACKING_ERROR,end_ms=17120,error_deg=4510.0
```

### 8.2 주기 로그와의 관계

- `CSV`는 loop telemetry
- `CMD_*`는 command lifecycle event

즉 둘은 분리해서 본다.

## 9. 데이터 구조 제안

다음과 같은 구조체를 권장한다.

```c
typedef enum {
    CMD_IDLE = 0,
    CMD_ACTIVE,
    CMD_REACHED,
    CMD_TIMEOUT,
    CMD_ABORTED,
    CMD_FAULTED
} CommandState_t;

typedef enum {
    CMD_SRC_NONE = 0,
    CMD_SRC_UDP,
    CMD_SRC_KEYBOARD,
    CMD_SRC_SERVICE,
    CMD_SRC_LOCALTEST
} CommandSource_t;

typedef enum {
    CMD_RESULT_NONE = 0,
    CMD_RESULT_REACHED,
    CMD_RESULT_TIMEOUT,
    CMD_RESULT_ESTOP,
    CMD_RESULT_DISABLED,
    CMD_RESULT_REPLACED,
    CMD_RESULT_FAULT_LIMIT,
    CMD_RESULT_FAULT_TRACKING
} CommandResult_t;

typedef struct {
    uint32_t command_id;
    CommandState_t state;
    CommandSource_t source;
    CommandResult_t result;
    float target_steering_deg;
    float target_motor_deg;
    float start_steering_deg;
    float final_steering_deg;
    float final_error_deg;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t timeout_ms;
} CommandLifecycle_t;
```

## 10. 수정 대상 파일

### 필수

- `Core/Inc/position_control.h`
- `Core/Src/position_control.c`
- `Core/Src/main.c`

### 선택

- `Core/Inc/debug_vars.h`
- `Core/Src/debug_vars.c`
- `Core/Src/ethernet_communication.c`

## 11. 파일별 수정 방향

### `position_control.h`

- `CommandState_t`
- `CommandSource_t`
- `CommandResult_t`
- `CommandLifecycle_t`
- `PositionControl_GetCommandLifecycle()`

### `position_control.c`

- target 수락 시 `CMD_START`
- 완료 조건 만족 시 `CMD_REACHED`
- timeout 검사
- abort/fault 종료 처리
- event log 출력

### `main.c`

- source 구분해서 target 전달
- `UDP`, `Keyboard` 각각 명령 source 지정
- ESTOP/disable/timeout 발생 시 lifecycle 종료 함수 호출

## 12. 이번 단계에서 하지 않는 것

아래는 본 문서 범위 밖이다.

- homing 완료를 readiness 조건으로 강제하는 것
- ADC absolute zero를 startup acceptance 조건으로 넣는 것
- homing 실패/복구 절차를 command lifecycle에 포함하는 것

이 항목들은 `homing` 기계 설계 및 startup state machine이 준비된 뒤 추가한다.

## 13. 인수 기준

현재 단계에서 아래를 만족하면 본 명세 적용 완료로 본다.

1. target 명령이 수락되면 `CMD_START`가 남는다.
2. 목표 도달 시 `CMD_REACHED`가 남는다.
3. 3초 초과 시 `CMD_TIMEOUT`이 남는다.
4. ESTOP / disable / comm timeout 시 `CMD_ABORT` 또는 `CMD_FAULT`가 남는다.
5. 각 이벤트가 같은 `command_id`로 추적 가능하다.

## 14. 한 줄 요약

지금 단계의 목표는 `homing 없는 상태에서도` "몇 번째 명령이 언제 시작됐고, 성공했는지 실패했는지"를 시스템이 스스로 말할 수 있게 만드는 것이다.
