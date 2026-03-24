# Project Instructions

## Change Log Rule

- 코드가 수정될 때마다 반드시 `Doc/change_code/YYYY-MM-DD.md` 파일을 만들거나 같은 날짜 파일을 갱신한다.
- 날짜 형식은 `YYYY-MM-DD.md`를 사용한다.
- 같은 날짜에 추가 수정이 있으면 기존 항목을 지우지 말고 누적한다.
- 같은 날짜의 이전 수정 내용이 이후 수정으로 대체되면, 대체된 기존 항목은 `~~취소선~~` 처리하고 바로 아래에 왜 대체되었는지 짧게 적는다.
- 각 날짜 문서에는 최소한 아래를 남긴다.
  - 수정 시간
  - 수정 파일
  - 무엇을 어떻게 바꿨는지
  - 수정 이유
  - 영향 범위

## Portal / README Sync Rule

- 사용자가 `index.html`, 포털, README, 문서 업데이트를 요청하면 텍스트 설명만 수정하지 않는다.
- 아래 항목 중 변경 영향이 있는 것은 항상 함께 갱신한다.
  - `Doc/steering_portal/index.html`의 본문 설명과 상태 카드
  - `Doc/steering_portal/app.js`의 데이터 모델
  - 완성도 점수, 평가 문구, bar/score/gauge 등 시각화 수치
  - 동적 시뮬레이션의 기준값, 상태 문구, 계산식
  - 로그 예시, CSV 예시, evidence 설명
  - `README.md`, `Doc/README.md`의 구조/동작/상태 설명
- 즉 "내용 업데이트"는 기본적으로 `설명 + 시각화 + 점수/게이지 + 시뮬레이션 + README 정합성`까지 한 세트로 본다.
