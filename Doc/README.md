# Autonomous Steering Sub-Controller Documentation Index

이 문서는 `servo_control_baqu` 문서 묶음의 현재 기준점을 정리한다. 최신 기준은 2026-03-25 코드와 bench 관찰값이다.

## 1. Current Runtime Summary

### 1.1 Runtime Contract

- control loop: 1 ms
- external unit: `steering_deg`
- internal unit: `motor_deg`, `enc_count`, `pulse_hz`
- pulse path: `PE9(TIM1_CH1)` + `PE10(direction GPIO)`
- encoder path: `PD12/PD13 -> TIM4 encoder mode`
- lifecycle: `CMD_START / REACHED / TIMEOUT / ABORT / FAULT`

### 1.2 What Changed In The Latest Baseline

- `main.c`는 CubeMX 부트 진입점으로 얇아졌고, 앱 startup과 super-loop는 `app_runtime.c`로 분리됐다.
- `position_control.c`는 제어 코어 중심으로 남고, 문자열/출력/debug var 미러링은 `position_control_diag.c`로 분리됐다.
- `encoder_reader.c`는 centered raw 해석에서 누적 `unwrap` count 방식으로 전환됐다.
- `pulse_control.c`는 실제 TIM1 클럭 계산, reverse guard, status 조회 API를 가진다.
- `app_runtime.c`의 CSV와 keyboard snapshot은 `enc_cnt`, `enc_raw`, `req_hz`, `applied_hz`, `RUN`, `REV`, `command lifecycle`을 함께 남긴다.
- 엔코더 런타임 진단 `[ENCDBG]` 훅은 코드에 남아 있지만 기본값은 `OFF`다.

### 1.3 Current Bench Status

- 손으로 축을 돌리면 TIM4 `CNT` 변화가 확인된다.
- `PE9` pulse output은 확인된다.
- 드라이브는 `P-RUN`, `P4-00 = 2`를 보이지만 `St-06 = 0`, `St-04` 정지 상태가 확인됐다.
- 따라서 현재 bench 핵심 이슈는 "드라이브가 command pulse를 내부 명령으로 카운트하지 못하는 상태"다.

## 2. Recommended Reading Order

1. [`../README.md`](../README.md)
   프로젝트 전체 요약과 현재 runtime 상태를 가장 빠르게 파악할 수 있다.
2. [`command_lifecycle_no_homing_spec.md`](command_lifecycle_no_homing_spec.md)
   현재 lifecycle 정책과 남은 gap을 볼 수 있다.
3. [`steering_portal/index.html`](steering_portal/index.html)
   시각화된 현재 상태, REQ, evidence, 코드 탐색 포털이다.
4. [`doxygen/html/index.html`](doxygen/html/index.html)
   파일별 역할, 받는 정보, 보내는 정보, 핵심 함수/변수를 랜딩에서 바로 볼 수 있다.
5. [`change_code/2026-03-25.md`](change_code/2026-03-25.md)
   최근 코드와 문서 변경 이력을 순서대로 확인할 수 있다.

## 3. Core Documents

| 문서 | 설명 | 현재 relevance |
|---|---|---|
| [`command_lifecycle_no_homing_spec.md`](command_lifecycle_no_homing_spec.md) | no-homing lifecycle 규격 | 높음 |
| [`code_modules.md`](code_modules.md) | 모듈별 역할과 과거 구조 메모 | 중간 |
| [`implementation_team_spec.md`](implementation_team_spec.md) | 구현 분담/요구사항 정리 | 중간 |
| [`latency_measurement_spec.md`](latency_measurement_spec.md) | latency 측정 기준 | 높음 |
| [`latency_contract.md`](latency_contract.md) | timing contract | 높음 |
| [`latency_data_evidence.md`](latency_data_evidence.md) | evidence 관리 | 높음 |
| [`hardware_pinmap.md`](hardware_pinmap.md) | 실제 핀/배선 확인 | 높음 |
| [`steering_portal/index.html`](steering_portal/index.html) | 설명 포털 | 높음 |
| [`doxygen/html/index.html`](doxygen/html/index.html) | 소스 브라우저와 역할 요약 | 높음 |

## 4. Current Gaps To Keep In Mind

- boot-time auto-enable 제거 전이라 startup safety 설명이 약하다.
- fault latch / clear policy가 아직 정식 계약으로 닫히지 않았다.
- `constants.h`의 `MAX_PULSE_FREQ = 1000000`과 실제 `pulse_control.c` clamp `100000` 사이의 계약 차이가 남아 있다.
- UART logging이 blocking path라 strict timing 검증과 bench motion 관찰을 분리해서 봐야 한다.

## 5. Change Traceability

- 날짜별 코드/문서 변경은 `change_code/YYYY-MM-DD.md`에 누적한다.
- 같은 날짜에 대체된 항목은 취소선으로 남기고 대체 이유를 바로 아래 적는다.
- README, portal, spec, doxygen 랜딩을 같이 바꿨다면 같은 날짜 문서에서 한 세트로 추적한다.

## 6. One-Line Use Guide

현재 상태를 설명해야 할 때는 `README -> command lifecycle spec -> steering_portal -> doxygen index -> change_code/2026-03-25.md` 순서로 보는 것이 가장 빠르다.
