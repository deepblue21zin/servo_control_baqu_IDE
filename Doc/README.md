# Autonomous Steering Sub-Controller Documentation Index

이 문서는 `servo_control_baqu` 문서 묶음의 현재 기준점을 정리한다. 최신 기준은 2026-04-06 문서/설정 갱신본이며, `26-04-05` keyboard bench log 재판정과 UDP 시험 전환 설정까지 포함해 본다.

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

- `APP_RUNTIME_INPUT_SOURCE = APP_RUNTIME_INPUT_SOURCE_KEYBOARD`
- `APP_RUNTIME_ENCODER_DIAG_ENABLE = 1`
- `APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE = 0`
- `APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE = 1`
- startup 시 `Relay_ServoOn()`과 `PositionControl_Enable()`가 자동으로 실행됨

즉 현재 baseline은 keyboard bench와 encoder diag가 켜진 bring-up 중심 빌드다. 실제 UDP 시험으로 넘어갈 때는 `project_params.h`에서 `APP_RUNTIME_INPUT_SOURCE`만 `APP_RUNTIME_INPUT_SOURCE_UDP`로 바꾸면 된다.

### 1.3 Current Bench Status

- 소프트웨어상 target, output, requested/applied Hz는 정상적으로 변한다.
- 실제 encoder truth는 `TIM2`, `PA0/PB3`, `[ENCDBG]` 기준으로 확인해야 한다.
- virtual feedback 경로는 코드에 존재하지만 현재 기본값은 `OFF`다.
- `26-04-05` run에서는 `-20 / 20 / 30 / 40 deg` settled point와 linear `enc_cnt`가 확인되어 final angle 정합성은 좋아졌다.
- 반대로 rapid step 구간은 `CMD_ABORT ... REPLACED`가 반복되고 40 / 30 / 20 deg settling time이 약 14~16 s라 transient tuning은 아직 부족하다.

## 2. Recommended Reading Order

1. [`../README.md`](../README.md)  
   전체 현재 상태를 가장 빠르게 파악할 수 있다.
2. [`code_modules.md`](code_modules.md)  
   각 모듈의 역할과 ownership 관점 설명을 볼 수 있다.
3. [`command_lifecycle_no_homing_spec.md`](command_lifecycle_no_homing_spec.md)  
   현재 lifecycle 정책과 남은 gap을 볼 수 있다.
4. [`REQ/steering_project_req_ownership_guide.html`](REQ/steering_project_req_ownership_guide.html)
   목표 상태전이, ownership, REQ를 현업식으로 정리한 문서다.
5. [`matlab_simulink_application_plan.md`](matlab_simulink_application_plan.md)
   현재 프로젝트에 MATLAB / Simulink를 어떤 순서와 경계로 붙여야 하는지 정리한 실행 계획 문서다.
6. [`members/position_control.html`](members/position_control.html)
   position control 담당자에게 바로 전달할 상세 REQ와 현재 구현/부족점 정리다.
7. [`members/adc_encoder.html`](members/adc_encoder.html)
   ADC + encoder 담당자에게 바로 전달할 상세 REQ와 현재 구현/부족점 정리다.
8. [`members/pulse_control.html`](members/pulse_control.html)
   pulse/direction 담당자의 actuator contract와 실험 증거 요구사항을 정리한 문서다.
9. [`members/homing_relay.html`](members/homing_relay.html)
   startup, homing, relay safety 담당자의 state machine 중심 할당 문서다.
10. [`members/ethernet_integration.html`](members/ethernet_integration.html)
   ethernet, mode transition, timeout, system integration 담당자의 상세 할당 문서다.
11. [`members/verification_tooling.html`](members/verification_tooling.html)
   latency, debug vars, plotting, portal, evidence automation 담당자의 상세 할당 문서다.
12. [`steering_portal/index.html`](steering_portal/index.html)
   현재 구현, evidence, 코드 영역별 역할 설명, 주요 소스 파일별 상세 브리프, 현재 점수를 시각적으로 확인할 수 있다.
13. [`putty/index.html`](putty/index.html)
   `putty.log`를 bridge live 또는 polling으로 읽고 타입별 분류, 자동 해석, recording 저장까지 한 화면에서 처리하는 로컬 뷰어다.
14. [`../putty_log/start_putty_live_viewer.ps1`](../putty_log/start_putty_live_viewer.ps1)
   Python bridge, 브라우저, PuTTY logging을 한 번에 띄우는 런처다.
15. [`doxygen/html/index.html`](doxygen/html/index.html)
   파일별 역할, 입력/출력, 핵심 함수/변수를 빠르게 탐색할 수 있다.
16. [`change_code/2026-04-06.md`](change_code/2026-04-06.md)
   당일 변경 이력을 순서대로 확인할 수 있다.

## 3. Core Documents

| 문서 | 설명 | 현재 relevance |
|---|---|---|
| [`code_modules.md`](code_modules.md) | 모듈 역할과 구조 메모 | 높음 |
| [`command_lifecycle_no_homing_spec.md`](command_lifecycle_no_homing_spec.md) | no-homing lifecycle 규격 | 높음 |
| [`REQ/steering_project_req_ownership_guide.html`](REQ/steering_project_req_ownership_guide.html) | REQ / ownership / target state machine | 높음 |
| [`matlab_simulink_application_plan.md`](matlab_simulink_application_plan.md) | MATLAB / Simulink 적용 계획 | 높음 |
| [`members/position_control.html`](members/position_control.html) | position control 담당 상세 할당서 | 높음 |
| [`members/adc_encoder.html`](members/adc_encoder.html) | ADC + encoder 담당 상세 할당서 | 높음 |
| [`members/pulse_control.html`](members/pulse_control.html) | pulse control 담당 상세 할당서 | 높음 |
| [`members/homing_relay.html`](members/homing_relay.html) | homing + relay 담당 상세 할당서 | 높음 |
| [`members/ethernet_integration.html`](members/ethernet_integration.html) | ethernet / system integration 담당 상세 할당서 | 높음 |
| [`members/verification_tooling.html`](members/verification_tooling.html) | verification / tooling 담당 상세 할당서 | 높음 |
| [`hardware_pinmap.md`](hardware_pinmap.md) | 실제 핀/배선 확인 | 높음 |
| [`latency_measurement_spec.md`](latency_measurement_spec.md) | latency 측정 기준 | 높음 |
| [`latency_contract.md`](latency_contract.md) | timing contract | 높음 |
| [`latency_data_evidence.md`](latency_data_evidence.md) | evidence 관리 | 높음 |
| [`steering_portal/index.html`](steering_portal/index.html) | 설명 포털 | 높음 |
| [`putty/index.html`](putty/index.html) | live PuTTY 로그 분류/해석/recording 뷰어 | 높음 |
| [`../putty_log/start_putty_live_viewer.ps1`](../putty_log/start_putty_live_viewer.ps1) | bridge + browser + PuTTY 런처 | 높음 |
| [`doxygen/html/index.html`](doxygen/html/index.html) | 코드 브라우저와 역할 요약 | 높음 |

## 4. Current Gaps To Keep In Mind

- real encoder truth는 최근 log 기준으로 내부 일관성은 좋아졌지만 stale/cross-check/scope closure를 아직 못 끝냈다.
- startup auto-enable 제거 전이라 startup safety 설명이 약하다.
- watchdog timeout이 약 32.8 s로 steering safe-state 기준에는 너무 길다.
- `project_params.h`가 생겼지만 현재 app/runtime 전체가 완전히 단일 파라미터 파일로 정리되진 않았다.

## 5. Change Traceability

- 날짜별 코드/문서 변경은 `change_code/YYYY-MM-DD.md`에 누적한다.
- 같은 날짜에 대체된 항목은 취소선으로 남기고 대체 이유를 바로 아래 적는다.
- README, portal, spec을 같이 바꿨다면 같은 날짜 문서에서 한 세트로 추적한다.

## 6. One-Line Use Guide

현재 상태를 설명해야 할 때는 `README -> REQ guide -> MATLAB/Simulink 적용 계획 -> members 할당서 6종 -> steering_portal(코드 영역 설명 포함) -> putty viewer/launcher -> doxygen -> change_code/2026-04-06.md` 순서로 보는 것이 가장 빠르다.
