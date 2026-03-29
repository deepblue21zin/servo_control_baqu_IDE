# Autonomous Steering Sub-Controller Documentation Index

이 문서는 `servo_control_baqu` 문서 묶음의 현재 기준점을 정리한다. 최신 기준은 2026-03-29 코드와 bench 관찰값이다.

## 1. Current Runtime Summary

### 1.1 Runtime Contract

- control loop: 1 ms
- external unit: `steering_deg`
- internal unit: `motor_deg`, `enc_count`, `pulse_hz`
- pulse path: `PE9(TIM1_CH1)` + `PE10(direction GPIO)`
- encoder path: `PA0/PB3 -> TIM2 encoder mode`
- debug path: `[ENCDBG]`가 실제 TIM2 `cnt/delta/A/B`를 출력
- lifecycle: `CMD_START / REACHED / TIMEOUT / ABORT / FAULT`

### 1.2 Current Bench Defaults

- `APP_RUNTIME_KEYBOARD_TEST_MODE = 1`
- `APP_RUNTIME_ENCODER_DIAG_ENABLE = 1`
- `APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE = 0`
- `APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE = 1`
- startup 시 `Relay_ServoOn()`과 `PositionControl_Enable()`가 자동으로 실행됨

즉 현재 baseline은 keyboard bench와 encoder diag가 켜진 bring-up 중심 빌드다.

### 1.3 Current Bench Status

- 소프트웨어상 target, output, requested/applied Hz는 정상적으로 변한다.
- 실제 encoder truth는 `TIM2`, `PA0/PB3`, `[ENCDBG]` 기준으로 확인해야 한다.
- virtual feedback 경로는 코드에 존재하지만 현재 기본값은 `OFF`다.
- 최근 하드웨어 점검에서는 encoder 채널 진폭 불균형이 관찰되어, real encoder chain closure가 아직 미완료다.

## 2. Recommended Reading Order

1. [`../README.md`](../README.md)  
   전체 현재 상태를 가장 빠르게 파악할 수 있다.
2. [`code_modules.md`](code_modules.md)  
   각 모듈의 역할과 ownership 관점 설명을 볼 수 있다.
3. [`command_lifecycle_no_homing_spec.md`](command_lifecycle_no_homing_spec.md)  
   현재 lifecycle 정책과 남은 gap을 볼 수 있다.
4. [`REQ/steering_project_req_ownership_guide.html`](REQ/steering_project_req_ownership_guide.html)  
   목표 상태전이, ownership, REQ를 현업식으로 정리한 문서다.
5. [`steering_portal/index.html`](steering_portal/index.html)  
   현재 구현, evidence, 현재 점수를 시각적으로 확인할 수 있다.
6. [`doxygen/html/index.html`](doxygen/html/index.html)  
   파일별 역할, 입력/출력, 핵심 함수/변수를 빠르게 탐색할 수 있다.
7. [`change_code/2026-03-29.md`](change_code/2026-03-29.md)  
   당일 변경 이력을 순서대로 확인할 수 있다.

## 3. Core Documents

| 문서 | 설명 | 현재 relevance |
|---|---|---|
| [`code_modules.md`](code_modules.md) | 모듈 역할과 구조 메모 | 높음 |
| [`command_lifecycle_no_homing_spec.md`](command_lifecycle_no_homing_spec.md) | no-homing lifecycle 규격 | 높음 |
| [`REQ/steering_project_req_ownership_guide.html`](REQ/steering_project_req_ownership_guide.html) | REQ / ownership / target state machine | 높음 |
| [`hardware_pinmap.md`](hardware_pinmap.md) | 실제 핀/배선 확인 | 높음 |
| [`latency_measurement_spec.md`](latency_measurement_spec.md) | latency 측정 기준 | 높음 |
| [`latency_contract.md`](latency_contract.md) | timing contract | 높음 |
| [`latency_data_evidence.md`](latency_data_evidence.md) | evidence 관리 | 높음 |
| [`steering_portal/index.html`](steering_portal/index.html) | 설명 포털 | 높음 |
| [`doxygen/html/index.html`](doxygen/html/index.html) | 코드 브라우저와 역할 요약 | 높음 |

## 4. Current Gaps To Keep In Mind

- real encoder truth가 아직 hardware closure를 못 끝냈다.
- startup auto-enable 제거 전이라 startup safety 설명이 약하다.
- watchdog timeout이 약 32.8 s로 steering safe-state 기준에는 너무 길다.
- `project_params.h`가 생겼지만 현재 app/runtime 전체가 완전히 단일 파라미터 파일로 정리되진 않았다.

## 5. Change Traceability

- 날짜별 코드/문서 변경은 `change_code/YYYY-MM-DD.md`에 누적한다.
- 같은 날짜에 대체된 항목은 취소선으로 남기고 대체 이유를 바로 아래 적는다.
- README, portal, spec을 같이 바꿨다면 같은 날짜 문서에서 한 세트로 추적한다.

## 6. One-Line Use Guide

현재 상태를 설명해야 할 때는 `README -> code_modules -> REQ guide -> steering_portal -> doxygen -> change_code/2026-03-29.md` 순서로 보는 것이 가장 빠르다.
