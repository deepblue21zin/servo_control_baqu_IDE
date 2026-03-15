# MClab Summer Intern PDF Outline

이 문서는 DGIST MClab 여름 인턴 제출용 PDF를 빠르게 만들기 위한 목차와 그림 배치 가이드다. 현재 프로젝트 상태를 기준으로 "무엇을 강하게 말하고, 무엇은 솔직하게 한계로 남길지"를 포함한다.

## 1. 추천 방향

현재 상태에서는 이 프로젝트를 "완성된 상용 조향 제어기"로 포장하기보다 아래처럼 잡는 것이 가장 강하다.

- `STM32 기반 조향 액추에이터 서브컨트롤러 bring-up`
- `1ms closed-loop control architecture`
- `latency profiling and debug instrumentation`
- `pulse/direction actuator interface verification`
- `sensor-path debugging and fail-safe planning`

즉, 구현과 검증 체계를 잘 만든 프로젝트로 보이게 하는 것이 핵심이다.

## 2. 추천 분량

- 4~6 pages
- 그림 5~7개
- 표 2~3개
- 부록 또는 QR/video 링크 선택사항

## 3. 페이지별 구성

### Page 1. Project Overview

넣을 것:
- 프로젝트 제목
- 한 줄 요약
- 시스템 구성 그림
- 본인 역할 요약

그림 배치:
- 상단: 시스템 블록도 1개
- 하단 좌측: 하드웨어 사진 1개
- 하단 우측: 핵심 성과 3줄

추천 문장:
- "Built an STM32F429-based steering actuator sub-controller with 1ms loop timing, pulse/direction output, UART/latency instrumentation, and fail-safe bring-up workflow."

### Page 2. Architecture and Control Flow

넣을 것:
- `command -> position_control -> pulse_control -> drive -> encoder feedback` 흐름
- 핵심 코드 모듈 설명
- `steering_deg` / `motor_deg` / `encoder_count` 단위 체계

그림 배치:
- 전체 데이터 흐름도 1개
- 신호 경로 표 1개

핵심 메시지:
- 제어기, 센서, 액추에이터, safety path를 분리해서 설계했다

### Page 3. Timing and Runtime Evidence

넣을 것:
- latency 결과 표
- `Sense`, `Control`, `Actuate`, `Comms` 그래프
- 측정 환경 설명

그림 배치:
- 좌측: `latency_trace.png`
- 우측: 결과 요약 표

표에 넣을 값 예시:
- `Sense avg/p99`
- `Control avg/p99`
- `Actuate avg/p99`
- runtime batch 수
- total measured time

중요:
- `deadline miss`가 있다면 숨기지 말고 "interactive logging condition"이라고 명시한다.

### Page 4. Debugging and Validation

넣을 것:
- `target/current/error/output/DIR/ENC` snapshot 기반 debugging 방법
- encoder/current stuck 문제를 어떻게 좁혀갔는지
- logic analyzer / PuTTY / CubeMonitor 사용 흐름

그림 배치:
- 상단: `control_trace.png`
- 하단: 대표 UART snapshot 2~3개

핵심 메시지:
- 문제를 감으로 찾은 것이 아니라, 로그와 계측으로 원인을 줄여갔다

### Page 5. Safety and Fail-safe

넣을 것:
- `Q`, `X`, invalid input reject, startup inhibit` 결과
- 어떤 조건에서 pulse를 막아야 하는지
- homing / relay / estop 계획

그림 배치:
- 좌측: fail-safe 테스트 표
- 우측: `PE9/PE10` 혹은 UART/logic analyzer 캡처

표 예시:
- trigger
- expected response
- observed response
- evidence

### Page 6. Lessons and Next Steps

넣을 것:
- 지금까지 검증 완료된 것
- 아직 미완성인 것
- 다음 단계

강하게 쓰면 좋은 것:
- latency evidence 구축
- structured debug logging
- modular architecture

솔직하게 써야 하는 것:
- encoder feedback path는 추가 확인 필요
- homing/full fail-safe는 계속 개발 중

## 4. 지금 그림으로 넣기 좋은 자료

현재 바로 넣을 수 있는 것:
- `putty_log/plot_putty_log.py`로 만든 `latency_trace.png`
- `putty_log/plot_putty_log.py`로 만든 `control_trace.png`
- UART snapshot 캡처
- 보드/모터/배선 사진

이번 주 안에 추가하면 좋은 것:
- 모터 움직임 짧은 영상 1개
- `Q` / `X` fail-safe 캡처
- encoder hand-turn 확인 로그

## 5. PDF에서 피해야 할 표현

- "완전히 검증된 steering controller"
- "closed-loop tracking 완료"
- "full fail-safe implemented"

대신 추천 표현:
- "bring-up and validation in progress"
- "runtime instrumentation and timing evidence completed"
- "sensor path issue isolated through structured logging"

## 6. 제출 직전 체크리스트

- 제목이 과장되지 않았는가
- 숫자 근거가 최소 3개 이상 있는가
- latency graph가 있는가
- control/debug graph가 있는가
- fail-safe 캡처가 최소 1개 있는가
- 현재 한계를 정직하게 적었는가
