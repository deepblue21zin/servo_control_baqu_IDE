const PORTAL_STEERING_GEAR_RATIO = 12.5;
const PORTAL_ENCODER_COUNT_PER_STEERING_DEG = (PORTAL_STEERING_GEAR_RATIO * 48000) / 360;
const PORTAL_PULSE_PER_STEERING_DEG = PORTAL_STEERING_GEAR_RATIO / 0.003;

const portalData = {
  kpis: [
    { label: "Loop period", value: "1 ms", note: "SysTick based control loop" },
    { label: "Encoder mode", value: "TIM2 + virtual option", note: "PA0/PB3 real path with optional pulse-integrated bench feedback" },
    { label: "Bench default", value: "Keyboard input", note: "현재 기본은 keyboard bench + real TIM2 diag이며, 실제 UDP 시험 전환은 project_params.h의 입력 소스 설정 한 곳에서 바꾼다." },
    { label: "Latest bench", value: "2026-04-05", note: "real TIM2 log 기준 -20 / 20 / 30 / 40 deg가 약 0.02 deg 이내로 정착했지만, large-step settling은 아직 14~16 s 수준이다." }
  ],
  latency: [
    { name: "Sense", avg: 4.075, p99: 4.117 },
    { name: "Control", avg: 3.996, p99: 9.467 },
    { name: "Actuate", avg: 7.777, p99: 7.800 },
    { name: "Comms", avg: 2.350, p99: 2.389 }
  ],
  actualTrace: {
    labels: ["370.28", "370.38", "370.48", "370.58", "370.68", "370.78", "370.88", "370.98", "371.08", "371.18", "371.28"],
    target: [20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20],
    current: [20.510, 20.438, 20.364, 20.306, 20.251, 20.226, 20.197, 20.137, 20.007, 19.984, 19.987]
  },
  evaluation: {
    score: 76,
    verdict: "26-04-05 real TIM2 bench log 기준으로 최종 정착각은 꽤 정확하지만, transient speed와 startup/fail-safe closure가 아직 현업 완성도를 제한한다.",
    summary:
      "현업 관점에서 이 프로젝트는 이제 단순한 구조 설명용 데모를 넘어, real TIM2 bench log로 실제 settled angle accuracy를 보여준 steering sub-controller baseline이라고 설명할 수 있다. 2026-04-05 run에서는 -20 / 20 / 30 / 40 deg가 약 0.02 deg 이내로 정착했고 `CMD_REACHED`, linear `enc_cnt`, `CSV + ENCDBG` evidence도 남았다. 다만 rapid step에서는 `CMD_ABORT ... REPLACED`가 많고 large-step settling time이 14~16 s라 transient tuning, startup state machine, fault latch, sensor stale/cross-check가 아직 남아 있어 현재 평점은 약 76%가 적절하다.",
    interviewer:
      "면접에서는 '목표각 변환이 맞고 real feedback으로 최종각까지 닫힌다'는 점을 먼저 말하고, 그 다음 '응답 속도와 safety closure는 아직 bring-up 중'이라고 정직하게 이어가는 편이 가장 강하다. 특히 `26-04-05` 로그에서 `CMD_REACHED`, steady-state error, `CMD_ABORT ... REPLACED`, settling time을 함께 보여주면 제어기 해석 능력과 자기평가 능력을 동시에 드러낼 수 있다.",
    categories: [
      {
        name: "Architecture / Readability",
        score: 91,
        detail: "main.c를 얇은 부트 엔트리로 남기고 app_runtime.c, position_control_diag.c, project_params.h 기반 설정 집중화까지 진행되며 구조가 한 단계 더 정리됐다. 다음 단계는 app_runtime 안의 bench console과 telemetry를 다시 쪼개는 것이다."
      },
      {
        name: "Observability / Evidence",
        score: 92,
        detail: "CSV, keyboard snapshot, command lifecycle log, latency batch, portal, change log에 더해 Doc/putty live viewer와 bridge launcher까지 이어져 있어 분석 가능한 증거 체계가 강하다. 최근 26-04-05 run은 steady-state accuracy와 slow transient를 같은 로그에서 함께 복원할 수 있다는 점까지 보여줬다."
      },
      {
        name: "Actuator Interface",
        score: 79,
        detail: "requested/applied Hz, reverse guard, direction polarity macro가 추가되며 actuator 계약이 좋아졌고, recent log에서도 실제 motion이 target reach로 이어지는 점이 보인다. 다만 waveform matrix와 drive monitor 기반 proof는 아직 남아 있다."
      },
      {
        name: "Sensor Truth",
        score: 62,
        detail: "현재 sensor path는 `TIM2(PA0/PB3)` real path와 optional virtual feedback path로 나뉜다. 26-04-05 log에서는 settled angle과 `enc_cnt` scaling이 내부적으로 일관되게 보였지만, stale fault, cross-check, scope-correlated authoritative closure는 아직 남아 있다."
      },
      {
        name: "Safety / Startup / Fail-safe",
        score: 56,
        detail: "ESTOP, lifecycle trace, failsafe profile 스위치는 존재한다. 반면 startup auto-enable, latched fault, clear policy, homing/ready gate, cross-check fault는 아직 미완성이다."
      },
      {
        name: "Verification Closure",
        score: 77,
        detail: "명령 단위 추적과 pulse/encoder 상태 관찰은 좋고, 이번 run 하나만으로도 최종각 정확도와 settling time을 같이 설명할 수 있다. 다만 submission-ready manifest와 drive/input waveform closure는 아직 남아 있다."
      }
    ],
    reasons: [
      {
        title: "최종 정착 각도 정합성은 분명히 좋아졌다",
        detail: "26-04-05 run에서 -20 / 20 / 30 / 40 deg가 모두 매우 작은 steady-state error로 정착했고, `CMD_REACHED`와 `enc_cnt`도 함께 남았다."
      },
      {
        title: "현재 병목이 속도와 운전 계약 쪽으로 더 선명해졌다",
        detail: "rapid step에서는 `CMD_ABORT ... REPLACED`가 많고, large-step settling time이 14~16 s 수준이라 현재 병목이 단순 변환 오류보다 transient response와 command pacing 쪽이라는 점이 분명해졌다."
      },
      {
        title: "startup safety와 fault policy는 아직 설명형 수준이다",
        detail: "구조 분리와 fail-safe profile은 좋아졌지만 boot-time auto-enable이 남아 있고, fault latch / clear policy / stale sensor taxonomy가 아직 정식 계약으로 닫히지 않았다."
      },
      {
        title: "production steering claim에는 마지막 sensor/actuator evidence pack이 필요하다",
        detail: "최근 로그는 강하지만, stale/cross-check/scope waveform/drive monitor까지 같은 run ID로 묶인 authoritative pack이 있어야 100%에 가까워진다."
      }
    ],
    to100: [
      {
        title: "Real encoder chain을 authoritative하게 닫아야 한다",
        detail: "`TIM2(PA0/PB3)`, `[ENCDBG]`, scope waveform, stale/cross-check 결과를 같은 run으로 묶어 real feedback을 최종 증명해야 한다."
      },
      {
        title: "Startup / Arm contract를 만들어야 한다",
        detail: "INIT -> READY -> ARMED -> RUN -> ESTOP_LATCH 형태로 startup auto-enable을 제거하고, readiness 확인 전에는 enable되지 않도록 바꿔야 한다."
      },
      {
        title: "Fault latch와 clear policy를 정식화해야 한다",
        detail: "ESTOP, tracking limit, timeout, stale sensor, implausible motion을 reason code와 clear 절차까지 포함해 운영 계약으로 정리해야 한다."
      },
      {
        title: "Transient response와 speed contract를 다시 튜닝해야 한다",
        detail: "현재 40 / 30 / 20 deg large step settling이 14~16 s 수준이므로 output limit, gain, step pacing, pulse contract를 함께 재조정해 응답 시간을 줄여야 한다."
      },
      {
        title: "Evidence manifest와 async logging을 도입해야 한다",
        detail: "blocking UART 대신 DMA + ring buffer를 넣고, raw log / PNG / portal snapshot / commit hash를 같은 run ID로 묶어 제출형 evidence pack을 만들어야 한다."
      }
    ]
  },
  analysisFlow: {
    steps: [
      {
        step: "Step 1",
        title: "Upper command와 단위를 먼저 맞춘다",
        summary: "상위 입력은 `steering_deg`이고, app runtime에서 `motor_deg`로 변환되어 controller로 들어간다. 먼저 gear ratio와 pulse conversion contract가 같은 기준을 쓰는지 본다.",
        vars: [
          "AutoDrive_Packet_t.steering_angle",
          "STEERING_GEAR_RATIO = 12.5",
          "SteeringDegToMotorDeg()",
          "DEG_PER_PULSE = 0.003 motor_deg"
        ],
        files: ["ethernet_communication.c", "constants.h", "app_runtime.c"],
        status: "적용됨. 외부 steering_deg -> 내부 motor_deg 경로는 정리됐지만 naming consistency는 더 다듬을 수 있다."
      },
      {
        step: "Step 2",
        title: "센서 truth를 먼저 확인한다",
        summary: "현재는 `enc_raw`만 보지 않고 `enc_cnt`, `GetAngleDeg()`, `[ENCDBG] cnt/delta/A/B`를 함께 봐야 한다. 코드에는 `TIM2(PA0/PB3)` real path와 optional virtual feedback path가 같이 존재한다.",
        vars: [
          "EncoderReader_GetRawCounter()",
          "EncoderReader_GetCount()",
          "EncoderReader_GetAngleDeg()",
          "ADC_Pot_GetAngle()"
        ],
        files: ["encoder_reader.c", "adc_potentiometer.c", "position_control.c"],
        status: "부분 적용. TIM2 real path와 virtual bench path는 존재하고 26-04-05 log에서는 -20 / 20 / 30 / 40 deg settled point와 linear enc_cnt가 보였다. 다만 stale/cross-check/scope closure는 아직 남아 있다."
      },
      {
        step: "Step 3",
        title: "error -> output -> direction -> drive monitor를 비교한다",
        summary: "현재는 state.output만 보는 것이 아니라 requested/applied Hz, reverse guard, direction, real `[ENCDBG]`와 scope를 함께 봐야 한다. 그래야 motion bottleneck이 controller 쪽인지 real sensor truth 쪽인지 분리된다.",
        vars: [
          "state.target_angle / current_angle / error",
          "PulseControl_SetFrequency()",
          "PulseControl_GetStatus()",
          "PE9 pulse / PE10 direction / [ENCDBG] / scope"
        ],
        files: ["position_control.c", "pulse_control.c", "app_runtime.c"],
        status: "적용됨. requested/applied Hz, reverse guard, encoder motion correlation은 보였고 large step도 최종 도달한다. 대신 rapid step에서는 command replacement가 많아 transient tuning이 남아 있다."
      },
      {
        step: "Step 4",
        title: "lifecycle / timing / fail-safe trace를 붙인다",
        summary: "마지막에는 command lifecycle, CSV, latency batch를 함께 본다. 현재는 event trace가 좋아졌고 다음 단계는 richer fault taxonomy와 startup clear policy다.",
        vars: [
          "CSV_HEADER / CSV rows",
          "CMD_START / CMD_REACHED / CMD_TIMEOUT / CMD_ABORT / CMD_FAULT",
          "LATENCY_BATCH_BEGIN / LATENCY_STAGE / LATENCY_BATCH_END",
          "PositionControl_EmergencyStop()"
        ],
        files: ["app_runtime.c", "position_control.c", "position_control_diag.c", "latency_profiler.c"],
        status: "적용됨. recent run 하나만으로도 `CMD_REACHED`, steady-state error, settling time, `CMD_ABORT ... REPLACED`를 함께 설명할 수 있을 만큼 evidence skeleton이 좋아졌다."
      }
    ],
    phases: [
      {
        name: "Command In",
        description: "UDP 또는 keyboard에서 목표값과 mode를 받는다.",
        vars: ["steering_angle", "mode", "misc", "g_last_rx_tick"]
      },
      {
        name: "Unit Convert",
        description: "상위 steering_deg를 내부 motor_deg로 변환한다.",
        vars: ["STEERING_GEAR_RATIO", "SteeringDegToMotorDeg()", "MotorDegToSteeringDeg()"]
      },
      {
        name: "Sense",
        description: "TIM2 raw와 누적 count를 읽어 current angle을 만든다. bench에서는 optional virtual feedback도 선택할 수 있고, ADC는 아직 startup/homing 보조 경로다.",
        vars: ["enc_raw", "enc_cnt", "EncoderReader_GetAngleDeg()", "ADC_Pot_GetAngle()"]
      },
      {
        name: "Control",
        description: "오차 계산, 안전 체크, PID 계산을 1ms loop 안에서 수행한다.",
        vars: ["state.error", "PositionControl_CheckSafety()", "PID_Calculate()"]
      },
      {
        name: "Actuate",
        description: "signed output을 pulse_hz와 direction으로 바꿔 TIM1과 line-driver로 보낸다.",
        vars: ["requested_frequency_hz", "applied_frequency_hz", "reverse_guard_active", "PE9", "PE10"]
      },
      {
        name: "Evidence",
        description: "CSV, lifecycle event, latency batch, keyboard snapshot, debug vars로 현재 상태를 바깥에 남긴다.",
        vars: ["CSV rows", "CMD_*", "LATENCY_STAGE", "[KB][snapshot]", "dbg_*"]
      }
    ],
    logs: [
      {
        title: "CSV trace 예시",
        snippet:
          "CSV_HEADER,ms,mode,target_deg,current_deg,error_deg,output,dir,enc_cnt,enc_raw,req_hz,applied_hz,out_active,rev_guard,cmd_id,cmd_state,cmd_result\nCSV,371276,1,20.000,19.987,0.013,0,0,33311,543,0,0,0,0,99,2,1",
        note: "26-04-05 settled example이다. CSV 한 줄만 봐도 target/current/error, encoder count, pulse state, lifecycle state를 같은 시점으로 묶을 수 있다. 단, virtual feedback이 켜지면 `current/enc_*` 해석은 `[ENCDBG]`와 함께 해야 한다."
      },
      {
        title: "Latency batch 예시",
        snippet:
          "LATENCY_BATCH_BEGIN,seq=32,samples=2000,core_hz=180000000,deadline_miss=1251\nLATENCY_STAGE,seq=32,name=Sense,count=2000,avg_us=4.078,p99_us=4.117,max_us=4.833\nLATENCY_STAGE,seq=32,name=Control,count=2000,avg_us=3.728,p99_us=3.728,max_us=4.411\nLATENCY_STAGE,seq=32,name=Actuate,count=2000,avg_us=7.767,p99_us=7.778,max_us=15.833",
        note: "26-04-05 run에서도 latency batch는 유효하게 남았다. 다만 deadline miss와 blocking UART logging 영향을 strict timing 검증과 분리해서 해석해야 한다."
      },
      {
        title: "Lifecycle event 예시",
        snippet:
          "CMD_START,id=97,src=KEYBOARD,target_deg=40.000,target_motor_deg=500.000,start_ms=308365,start_deg=5.915,start_error_deg=34.085\nCMD_REACHED,id=97,end_ms=324453,settling_ms=16088,final_deg=39.989,final_error_deg=0.011",
        note: "recent lifecycle event는 '최종각은 맞지만 느리다'는 사실을 그대로 보여준다. 다음 단계는 timeout / stale sensor / startup inhibit를 더 세분화해 reason code를 풍부하게 만드는 것이다."
      }
    ]
  },
  reqs: [
    {
      id: "REQ-P0-001",
      priority: "P0",
      owner: "Sensor / C",
      title: "unwrap count 장기 검증과 stale sensor 진단을 추가한다.",
      detail: "unwrap 자체는 구현됐으므로, 이제는 long-run wrap 구간과 commanded motion 대비 stale sensor를 판별하는 진단을 넣어야 한다."
    },
    {
      id: "REQ-P0-002",
      priority: "P0",
      owner: "Actuator / D",
      title: "real encoder chain과 signal integrity를 bench에서 닫는다.",
      detail: "`TIM2(PA0/PB3)`, line receiver output, scope waveform, `[ENCDBG]`를 한 세트로 비교해 authoritative real encoder truth를 닫아야 한다."
    },
    {
      id: "REQ-P0-003",
      priority: "P0",
      owner: "Startup / B",
      title: "boot-time auto enable을 제거하고 startup state machine을 만든다.",
      detail: "INIT -> HOMING -> READY -> ARMED -> RUN -> ESTOP_LATCH 구조로 정리하고, homing 성공 전 enable 금지를 강제한다."
    },
    {
      id: "REQ-P0-004",
      priority: "P0",
      owner: "Control / A",
      title: "fault latch / clear policy를 command lifecycle과 연결한다.",
      detail: "timeout, estop, tracking fault, stale sensor, startup inhibit를 reason code와 clear 절차까지 포함해 운영 계약으로 닫아야 한다."
    },
    {
      id: "REQ-P1-001",
      priority: "P1",
      owner: "Actuator / D",
      title: "pulse contract를 문서와 코드에서 하나로 통일한다.",
      detail: "`constants.h`의 1 MHz와 `pulse_control.c`의 100 kHz clamp를 current runtime contract와 future design target으로 분리해 정리해야 한다."
    },
    {
      id: "REQ-P1-002",
      priority: "P1",
      owner: "Sensor / C",
      title: "encoder-ADC cross-check와 implausible motion 진단을 넣는다.",
      detail: "commanded direction과 encoder delta 부호가 일정 시간 이상 맞지 않으면 sensor 또는 actuator fault 후보로 남겨야 한다."
    },
    {
      id: "REQ-P1-003",
      priority: "P1",
      owner: "Integration / E",
      title: "keyboard bench, UDP mode, timeout, ESTOP를 하나의 system contract로 묶는다.",
      detail: "현재는 keyboard mode 기본값과 UDP mode가 공존한다. mode transition과 recovery 조건을 같은 상태 모델로 정리해야 한다."
    },
    {
      id: "REQ-P1-004",
      priority: "P1",
      owner: "Validation / F",
      title: "CSV, latency, drive monitor, change log를 evidence pack으로 자동 연결한다.",
      detail: "실험 baseline, plots, raw logs, command lifecycle, drive monitor 값을 같은 태그로 묶으면 인수/포트폴리오 설명력이 높아진다."
    },
    {
      id: "REQ-P2-001",
      priority: "P2",
      owner: "Actuator / D",
      title: "direction reversal setup/hold를 scope evidence로 남긴다.",
      detail: "1 ms reverse guard가 실제 line-driver와 드라이브 입력에서 어떻게 보이는지 wave capture로 남겨야 한다."
    },
    {
      id: "REQ-P2-002",
      priority: "P2",
      owner: "Validation / F",
      title: "UART logging을 async path로 분리한다.",
      detail: "DMA + ring buffer로 바꾸어 motion bring-up과 strict timing 측정을 서로 덜 방해하게 해야 한다."
    }
  ],
  teams: [
    {
      owner: "Team A",
      title: "Motion Control & Fault Manager",
      scope: "position_control.c, lifecycle, timeout, fault reason",
      deliverable: "fault latch, clear policy, wrong-direction / stale sensor diagnostics",
      evidence: "state transition log, timeout case, fault injection trace",
      brief: "../members/position_control.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team B",
      title: "Startup / Homing / Relay Safety",
      scope: "homing.c, relay_control.c, startup gating",
      deliverable: "INIT -> READY -> ARMED -> RUN contract and recovery sequence",
      evidence: "startup GPIO trace, ready/arm log, homing fallback flow",
      brief: "../members/homing_relay.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team C",
      title: "Sensor Integrity",
      scope: "encoder_reader.c, adc_potentiometer.c, count/angle health",
      deliverable: "unwrap validation, stale detection, encoder-ADC cross-check",
      evidence: "long-run encoder log, mismatch detection trace, sign verification",
      brief: "../members/adc_encoder.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team D",
      title: "Pulse / Direction / Driver Timing",
      scope: "pulse_control.c, TIM1 contract, PF/PR path, drive monitor",
      deliverable: "requested/applied Hz contract, real encoder `[ENCDBG]` closure, reverse-guard wave proof",
      evidence: "scope capture, drive monitor table, commanded vs applied frequency report",
      brief: "../members/pulse_control.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team E",
      title: "Communication & System Integration",
      scope: "ethernet_communication.c, app_runtime.c mode transitions, timeout behavior",
      deliverable: "keyboard/UDP/system mode contract and recovery gating",
      evidence: "mode transition log, timeout fail-safe proof, packet-to-target trace",
      brief: "../members/ethernet_integration.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team F",
      title: "Verification & Tooling",
      scope: "latency_profiler.c, position_control_diag.c, debug_vars, plotting scripts, documentation portal",
      deliverable: "evidence automation, async logging support, portal maintenance, PuTTY live viewer and bridge upkeep",
      evidence: "latency report, PNG plots, portal snapshot, live viewer recording JSON, bridge launcher, submission-ready artifact set",
      brief: "../members/verification_tooling.html",
      briefLabel: "상세 할당서"
    }
  ],
  modules: [
    {
      id: "main",
      title: "main.c",
      subtitle: "CubeMX boot wrapper and top-level entrypoint",
      owner: "Integration / Runtime",
      inputs: ["HAL reset", "SystemClock_Config()", "MX_* peripheral initialization completion"],
      outputs: ["AppRuntime_Init()", "AppRuntime_RunIteration()", "__io_putchar() UART bridge"],
      risks: ["blocking __io_putchar()", "CubeMX merge points still need discipline"],
      files: ["../../Core/Src/main.c", "../../Core/Inc/main.h"],
      doxygen: "../doxygen/html/main_8c_source.html",
      snippet: "MX_GPIO_Init();\\nMX_USART3_UART_Init();\\nMX_TIM1_Init();\\nMX_TIM2_Init();\\nMX_LWIP_Init();\\nAppRuntime_Init();\\nwhile (1) {\\n    AppRuntime_RunIteration();\\n}",
      notes: "최근 리팩터링 이후 main.c는 CubeMX 재생성 경계와 앱 진입점 역할만 남겼다. 실제 startup, super-loop, bench telemetry는 app_runtime.c가 맡는다."
    },
    {
      id: "runtime",
      title: "app_runtime.c",
      subtitle: "Application startup, super-loop services, bench console, CSV and DIAG",
      owner: "Integration / Runtime",
      inputs: ["interrupt_flag", "keyboard UART bytes", "EthComm mode/data", "PositionControl/Pulse/Encoder state"],
      outputs: ["PositionControl_Enable()/Update()", "PositionControl_SetTargetWithSource()", "CSV/DIAG logs", "IWDG refresh"],
      risks: ["startup auto-enable still present", "bench console and telemetry are still combined", "blocking UART logging"],
      files: ["../../Core/Src/app_runtime.c", "../../Core/Inc/app_runtime.h"],
      doxygen: null,
      snippet: "void AppRuntime_RunIteration(void) {\\n    MX_LWIP_Process();\\n    AppRuntime_KeyboardProcessInput();\\n    AppRuntime_ServiceUdpComms();\\n    if (interrupt_flag != 0U) {\\n        interrupt_flag = 0U;\\n        AppRuntime_ServiceFastTick();\\n    }\\n    AppRuntime_ServicePeriodicCsv();\\n    HAL_IWDG_Refresh(&hiwdg);\\n}",
      notes: "현재 앱 운영의 중심 파일이다. startup 시퀀스, keyboard bench, UDP mode transition, fast tick, telemetry가 한데 모여 있어 이후 `bench_console`과 `telemetry`로 더 쪼갤 후보이기도 하다."
    },
    {
      id: "position",
      title: "position_control.c",
      subtitle: "PID, safety check, lifecycle, emergency stop",
      owner: "Control / A",
      inputs: ["target_angle[motor_deg]", "current_angle from encoder", "dt from HAL_GetTick"],
      outputs: ["state.output -> PulseControl_SetFrequency()", "fault -> Relay_Emergency()"],
      risks: ["fault latch 부족", "startup contract 미완성", "HAL_GetTick 기반 dt 정밀도 한계"],
      files: ["../../Core/Src/position_control.c", "../../Core/Inc/position_control.h"],
      doxygen: "../doxygen/html/position__control_8c_source.html",
      snippet: "state.current_angle = EncoderReader_GetAngleDeg();\\nstate.error = state.target_angle - state.current_angle;\\nif (!PositionControl_CheckSafety()) {\\n    PositionControl_CommandFinish(CMD_FAULTED, fault_result, HAL_GetTick());\\n    PositionControl_EmergencyStop();\\n    return;\\n}\\nstate.output = PID_Calculate(state.error, dt);\\nPulseControl_SetFrequency((int32_t)state.output);",
      notes: "현재 lifecycle trace는 충분히 좋아졌다. 다음 핵심은 stale sensor와 startup gating을 같은 fault policy로 묶는 것이다."
    },
    {
      id: "positiondiag",
      title: "position_control_diag.c",
      subtitle: "Command string helpers, debug var mirroring, state summary",
      owner: "Validation / F",
      inputs: ["PositionControl_State_t", "CommandLifecycle_t", "control_enabled/mode/fault_flag"],
      outputs: ["dbg_* globals", "bench summary printf", "readable lifecycle/error strings"],
      risks: ["stats/callback APIs are still placeholder stubs", "printf path is still blocking"],
      files: ["../../Core/Src/position_control_diag.c", "../../Core/Inc/position_control_diag.h"],
      doxygen: null,
      snippet: "void PositionControlDiag_UpdateDebugVars(const PositionControl_State_t* state, ...) {\\n    dbg_enc_raw = (int32_t)EncoderReader_GetRawCounter();\\n    dbg_pos_mdeg = PositionControlDiag_DegToMilliDeg(MotorDegToSteeringDeg(state->current_angle));\\n    dbg_target_mdeg = PositionControlDiag_DegToMilliDeg(MotorDegToSteeringDeg(state->target_angle));\\n    dbg_fault_flags = PositionControlDiag_BuildDebugFaultFlags(...);\\n}",
      notes: "진단 책임을 따로 뺀 덕분에 position_control.c 핫패스가 더 얇아졌다. 현재는 문자열/출력 계층이고, 앞으로는 async telemetry adapter 후보가 될 수 있다."
    },
    {
      id: "encoder",
      title: "encoder_reader.c",
      subtitle: "TIM2 raw counter / optional virtual feedback -> motor angle",
      owner: "Sensor / C",
      inputs: ["TIM2 raw counter", "optional virtual feedback count"],
      outputs: ["current_angle[motor_deg]", "enc_cnt", "enc_raw"],
      risks: ["long-run wrap validation 미완료", "raw와 count 의미 혼동 가능", "stale/cross-check 미완료"],
      files: ["../../Core/Src/encoder_reader.c", "../../Core/Inc/encoder_reader.h"],
      doxygen: "../doxygen/html/encoder__reader_8c_source.html",
      snippet: "uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);\\nint16_t delta = (int16_t)(raw - encoder_last_raw);\\nencoder_count += (int32_t)delta;\\nencoder_last_raw = raw;\\nreturn encoder_count - encoder_offset;",
      notes: "이제 encoder는 centered raw 직접 해석이 아니라 unwrap count를 사용한다. 26-04-05 log에서는 settled point와 linear enc_cnt가 보였고, 현재 과제는 stale/cross-check까지 포함한 authoritative sensor truth closure다."
    },
    {
      id: "pulse",
      title: "pulse_control.c",
      subtitle: "PE9 pulse, PE10 direction, reverse guard, status API",
      owner: "Actuator / D",
      inputs: ["signed pulse_hz", "step count", "direction enum"],
      outputs: ["TIM1 PWM", "GPIO direction", "applied Hz status", "reverse_guard_active"],
      risks: ["100 kHz current clamp", "real encoder truth unresolved", "logging load와 bench 상호작용"],
      files: ["../../Core/Src/pulse_control.c", "../../Core/Inc/pulse_control.h"],
      doxygen: "../doxygen/html/pulse__control_8c_source.html",
      snippet: "requested_frequency_hz = freq_hz;\\nPulseControl_ServiceReverseGuard();\\nif (target_direction != current_direction) {\\n    PulseControl_BeginReverseGuard(target_direction, target_frequency_hz);\\n    return;\\n}\\nPulseControl_StartContinuousOutput(target_frequency_hz);",
      notes: "최근 변경으로 requested/applied Hz와 reverse guard를 볼 수 있게 됐고, 26-04-05 log에서도 large step이 최종각까지 도달하는 점이 보였다. 이제 남은 핵심은 waveform proof와 transient speed 개선이다."
    },
    {
      id: "comms",
      title: "ethernet_communication.c",
      subtitle: "UDP parsing, mode handling, steering unit conversion",
      owner: "Integration / E",
      inputs: ["UDP command text / packets"],
      outputs: ["SteerMode_t", "AutoDrive_Packet_t", "status text"],
      risks: ["binary CRC contract 미완료", "keyboard default와 UDP mode 통합 필요"],
      files: ["../../Core/Src/ethernet_communication.c", "../../Core/Inc/ethernet_communication.h"],
      doxygen: "../doxygen/html/ethernet__communication_8c_source.html",
      snippet: "if (g_current_mode == STEER_MODE_MANUAL) {\\n    g_latest_pkt.steering_angle = joy_to_deg(joy_y);\\n    g_new_data = true;\\n} else if (g_current_mode == STEER_MODE_ESTOP) {\\n    g_emergency_request = true;\\n}\\n...\\nif (pc_speed == SPEED_ESTOP_SENTINEL) {\\n    g_emergency_request = true;\\n}",
      notes: "외부 명령 계약은 steering_deg 기준으로 꽤 정리됐다. 현재 app_runtime.c가 이 파일의 mode/data를 소비해 runtime 상태전이를 수행한다."
    },
    {
      id: "homing",
      title: "relay_control.c + homing.c",
      subtitle: "Power/safety outputs and startup zero-reference skeleton",
      owner: "Safety / B",
      inputs: ["ADC angle", "startup event", "operator reset"],
      outputs: ["encoder offset", "SVON/EMG relay action"],
      risks: ["main runtime에 실제 gating 미통합", "failure clear path 약함"],
      files: ["../../Core/Src/homing.c", "../../Core/Src/relay_control.c"],
      doxygen: "../doxygen/html/homing_8c_source.html",
      snippet: "float pot_angle = ADC_Pot_GetAngle();\\nEncoderReader_Reset();\\nint32_t offset_count = (int32_t)(pot_angle * ENCODER_COUNT_PER_REV / FULL_ROTATION_DEG);\\nEncoderReader_SetOffset(offset_count);\\nhoming_status = HOMING_STATUS_COMPLETE;",
      notes: "현재는 startup safety의 skeleton이다. 실제 운영 계약으로 보려면 ready/arm gating과 recovery까지 묶어야 한다."
    },
    {
      id: "latency",
      title: "latency_profiler.c + debug_vars.h",
      subtitle: "Measured evidence and debug observability",
      owner: "Validation / F",
      inputs: ["DWT cycle counter", "debug globals", "1 ms deadline tick"],
      outputs: ["LATENCY_STAGE logs", "dbg_* watch variables", "plots"],
      risks: ["strict measurement session과 interactive session 분리 필요"],
      files: ["../../Core/Src/latency_profiler.c", "../../Core/Inc/debug_vars.h"],
      doxygen: "../doxygen/html/latency__profiler_8c_source.html",
      snippet: "dt = DWT->CYCCNT - buf->start_cycle;\\nbuf->samples[buf->sample_count] = dt;\\n...\\nout_stats->avg_us = cycles_to_us(avg_cycles);\\nout_stats->p99_us = cycles_to_us(p99_cycles);\\nout_stats->max_us = cycles_to_us(buf->max_cycles);",
      notes: "이 프로젝트의 강점은 코드뿐 아니라 evidence pipeline이다. 최근에는 26-04-05 log 기반 plot과 portal trace까지 맞춰 설명할 수 있게 됐고, 다음 단계는 run manifest 자동화다."
    }
  ],
  runtimeAtlas: [
    {
      title: "main.c",
      file: "Core/Src/main.c",
      role: "CubeMX가 관리하는 초기화 코드와 앱 런타임 호출 경계를 유지한다.",
      receives: ["HAL reset + clock init", "MX_GPIO/MX_TIM/MX_LWIP 초기화 완료"],
      sends: ["AppRuntime_Init()", "AppRuntime_RunIteration()", "USART3 기반 __io_putchar()"],
      functions: ["main()", "SystemClock_Config()", "__io_putchar()"],
      variables: ["사용자 static state 거의 없음", "UART 리다이렉션은 huart3에 의존"]
    },
    {
      title: "app_runtime.c",
      file: "Core/Src/app_runtime.c",
      role: "실제 애플리케이션 startup과 super-loop를 운영하는 통합 supervisor다.",
      receives: ["interrupt_flag", "keyboard UART bytes", "EthComm latest packet / mode", "PositionControl/Pulse/Encoder 상태"],
      sends: ["PositionControl_Update()", "PositionControl_SetTargetWithSource()", "CSV/DIAG/UART 로그", "IWDG refresh"],
      functions: ["AppRuntime_Init()", "AppRuntime_RunIteration()", "AppRuntime_ServiceFastTick()", "AppRuntime_ServiceUdpComms()"],
      variables: ["g_debug_print_divider", "g_latency_report_seq", "g_keyboard_target_steer_deg", "g_periodic_csv_enabled"]
    },
    {
      title: "ethernet_communication.c",
      file: "Core/Src/ethernet_communication.c",
      role: "UDP 패킷을 runtime이 소비할 수 있는 mode/data 상태로 변환한다.",
      receives: ["LwIP UDP payload", "manual joy_y / PC steer packet", "service text command"],
      sends: ["AutoDrive_Packet_t", "SteerMode_t", "emergency request flag", "last_rx_tick"],
      functions: ["EthComm_Init()", "EthComm_UDP_Init()", "EthComm_HasNewData()", "EthComm_GetLatestData()"],
      variables: ["g_latest_pkt", "g_current_mode", "g_emergency_request", "g_last_rx_tick"]
    },
    {
      title: "position_control.c",
      file: "Core/Src/position_control.c",
      role: "PID 계산, lifecycle 상태전이, safety check, ESTOP를 수행하는 제어 코어다.",
      receives: ["target_angle[motor_deg]", "EncoderReader current angle", "HAL_GetTick() based dt"],
      sends: ["PulseControl_SetFrequency()", "Relay_Emergency()", "CommandLifecycle_t update"],
      functions: ["PositionControl_Update()", "PositionControl_SetTargetWithSource()", "PositionControl_Enable()", "PositionControl_EmergencyStop()"],
      variables: ["state", "command_lifecycle", "pending_command_source", "fault_flag"]
    },
    {
      title: "position_control_diag.c",
      file: "Core/Src/position_control_diag.c",
      role: "제어 코어에서 분리된 diagnostic helper 레이어다.",
      receives: ["PositionControl_State_t snapshot", "command lifecycle", "control mode / fault info"],
      sends: ["dbg_* globals", "상태 요약 printf", "readable enum labels"],
      functions: ["PositionControlDiag_UpdateDebugVars()", "PositionControlDiag_PrintStateSummary()", "PositionControlDiag_CommandStateString()"],
      variables: ["diag_level", "stats", "dbg_enc_raw", "dbg_fault_flags"]
    },
    {
      title: "encoder_reader.c",
      file: "Core/Src/encoder_reader.c",
      role: "TIM2 raw counter를 unwrap count와 motor angle로 바꾸고, bench에서는 optional virtual feedback도 받을 수 있다.",
      receives: ["TIM2->CNT raw counter", "virtual feedback count", "offset_count"],
      sends: ["current motor_deg", "enc_cnt", "enc_raw"],
      functions: ["EncoderReader_Init()", "EncoderReader_GetAngleDeg()", "EncoderReader_GetCount()", "EncoderReader_SetOffset()"],
      variables: ["encoder_last_raw", "encoder_count", "encoder_offset", "encoder_initialized"]
    },
    {
      title: "pulse_control.c",
      file: "Core/Src/pulse_control.c",
      role: "signed pulse_hz를 TIM1 PWM과 direction GPIO로 바꾸는 액추에이터 인터페이스다.",
      receives: ["signed pulse_hz", "step count", "target direction"],
      sends: ["PE9 pulse", "PE10 direction", "requested/applied Hz status", "reverse guard state"],
      functions: ["PulseControl_SetFrequency()", "PulseControl_SendSteps()", "PulseControl_Stop()", "PulseControl_GetStatus()"],
      variables: ["requested_frequency_hz", "applied_frequency_hz", "pending_direction", "pending_frequency_hz", "remaining_steps"]
    },
    {
      title: "relay_control.c + homing.c",
      file: "Core/Src/relay_control.c / Core/Src/homing.c",
      role: "SVON/EMG 릴레이 출력과 startup zero-reference skeleton을 담당한다.",
      receives: ["operator reset", "ADC absolute angle", "startup event"],
      sends: ["Relay_ServoOn/Off()", "Relay_Emergency()", "encoder offset"],
      functions: ["Relay_Init()", "Relay_ServoOn()", "Relay_Emergency()", "Homing_Start()"],
      variables: ["homing_status", "offset_count", "GPIO relay state"]
    }
  ],
  codeAreaFlow: [
    {
      step: "1",
      title: "입력 소스 결정",
      detail: "부팅 시 `project_params.h` 기준으로 keyboard bench build인지, 실제 UDP 입력 build인지 먼저 정한다. 지금 기본값은 keyboard다."
    },
    {
      step: "2",
      title: "명령과 센서 수집",
      detail: "`app_runtime.c`가 keyboard 또는 UDP에서 목표를 받고, `encoder_reader.c`가 TIM2 raw를 count / motor_deg로 바꿔 제어기에 넘긴다."
    },
    {
      step: "3",
      title: "제어와 안전 판정",
      detail: "`position_control.c`가 오차를 계산하고, `position_control_safety.c` 결과를 반영해 계속 구동할지 멈출지 정한다."
    },
    {
      step: "4",
      title: "출력과 증거 남기기",
      detail: "`pulse_control.c`가 pulse/direction을 내보내고, `position_control_diag.c`와 latency/log 경로가 현재 상태를 UART / dbg_* / portal evidence로 남긴다."
    }
  ],
  codeAreas: [
    {
      label: "Boot / Supervisor",
      title: "부트와 전체 런타임을 묶는 상위 감독 계층",
      files: ["main.c", "app_runtime.c", "project_params.h"],
      role: "이 영역은 시스템이 어떤 입력 경로를 쓸지 정하고, 초기화 순서와 1 ms 서비스 루프를 한곳에서 묶는 상위 조정자다.",
      why: "keyboard bench와 실제 UDP 시험을 같은 코드베이스에서 오가려면 모드 선택과 startup 순서를 한 파일 묶음에서 통제해야 한다.",
      how: [
        "CubeMX 초기화가 끝나면 `main.c`가 `AppRuntime_Init()`와 `AppRuntime_RunIteration()`만 호출한다.",
        "`project_params.h`의 입력 소스 설정이 keyboard/UDP 경로 선택 기준이 된다.",
        "`app_runtime.c`가 fast tick, 입력 처리, watchdog refresh, CSV/DIAG 출력을 묶어 super-loop처럼 운영한다."
      ],
      focus: ["APP_RUNTIME_INPUT_SOURCE", "AppRuntime_Init()", "AppRuntime_RunIteration()", "[BOOT] input source 로그"]
    },
    {
      label: "Input / Comms",
      title: "목표값을 받아 runtime 계약으로 바꾸는 입력 계층",
      files: ["ethernet_communication.c", "app_runtime.c"],
      role: "상위에서 온 steering 명령이나 bench용 keyboard 입력을 지금 런타임이 소비할 수 있는 target / mode / timeout 정보로 바꾸는 영역이다.",
      why: "지금은 keyboard로 시험하지만 실제 테스트에서는 UDP를 써야 하므로, 입력 경로가 바뀌어도 아래 제어기가 같은 target 형식을 받게 만들어야 한다.",
      how: [
        "keyboard bench에서는 문자 입력을 목표 조향각으로 바꿔 `PositionControl_SetTargetWithSource()`까지 연결한다.",
        "UDP build에서는 `ethernet_communication.c`가 packet 길이, sender, mode, steering angle을 검사하고 최신 packet 상태를 유지한다.",
        "`app_runtime.c`는 현재 build가 어떤 입력 소스를 쓰는지에 따라 필요한 service path만 호출한다."
      ],
      focus: ["EthComm_GetLatestData()", "g_last_rx_tick", "[KB][target]", "APP_RUNTIME_INPUT_SOURCE_IS_KEYBOARD"]
    },
    {
      label: "Sensor / Feedback",
      title: "실제 위치를 각도와 count로 환산하는 피드백 계층",
      files: ["encoder_reader.c", "adc_potentiometer.c"],
      role: "모터가 지금 어디에 있는지 판단하기 위해 TIM2 raw counter를 unwrap count와 motor_deg로 바꾸고, 필요하면 ADC 절대각 보조값도 읽는 영역이다.",
      why: "제어기 입장에서는 현재 위치가 숫자로 일관되게 들어와야 오차 계산이 가능하고, bench에서는 real encoder truth와 virtual feedback을 구분해 해석해야 한다.",
      how: [
        "`encoder_reader.c`는 `TIM2->CNT`의 변화량을 누적해 `enc_cnt`, `enc_raw`, `motor_deg`를 만든다.",
        "실제 bring-up에서는 `[ENCDBG] cnt/delta/A/B`로 하드웨어 진실성을 함께 확인한다.",
        "`adc_potentiometer.c`와 `homing.c`는 아직 startup readiness에 완전 통합되진 않았지만 absolute reference 보조 경로로 남아 있다."
      ],
      focus: ["EncoderReader_GetCount()", "EncoderReader_GetAngleDeg()", "[ENCDBG]", "ADC_Pot_GetAngle()"]
    },
    {
      label: "Control",
      title: "목표와 현재값 차이를 실제 구동 명령으로 바꾸는 제어 코어",
      files: ["position_control.c", "pid_controller.c"],
      role: "현재값과 목표값 차이를 계산하고, PID 결과를 signed pulse_hz로 만들어 actuator 쪽에 넘기는 핵심 제어 영역이다.",
      why: "조향축은 gear ratio와 pulse 환산이 크기 때문에, 이 영역이 오차를 얼마나 공격적으로 줄일지와 명령 단위를 어떻게 통일할지가 전체 움직임을 결정한다.",
      how: [
        "외부 `steering_deg`는 runtime에서 `motor_deg`로 변환된 뒤 `position_control.c`에 target으로 들어간다.",
        "`PositionControl_Update()`는 매 1 ms마다 `current_angle`, `error`, PID output, saturation을 갱신한다.",
        "출력은 `PulseControl_SetFrequency()`로 내려가며, `DEFAULT_OUTPUT_LIMIT`, `Kp`, deadband가 체감 속도와 응답성을 좌우한다."
      ],
      focus: ["state.target_angle", "state.current_angle", "PID_Calculate()", "DEFAULT_OUTPUT_LIMIT"]
    },
    {
      label: "Safety / Startup",
      title: "멈춰야 할 때 멈추고 startup 위험을 줄이는 보호 계층",
      files: ["position_control_safety.c", "relay_control.c", "homing.c"],
      role: "제어가 계속 가능할지 판단하고, fault가 나면 relay / emergency path로 넘겨 실제 동작을 끊는 보호 영역이다.",
      why: "조향 계통은 단순히 움직이는 것보다 잘못 움직일 때 어떻게 멈추는지가 더 중요하므로, fail-safe와 startup gating을 별도 책임으로 봐야 한다.",
      how: [
        "`position_control_safety.c`는 angle / tracking / velocity limit을 평가해 unsafe 여부를 제어 코어에 넘긴다.",
        "`PositionControl_EmergencyStop()`은 출력 0, pulse stop, relay emergency, lifecycle fault 기록까지 한 번에 처리한다.",
        "현재 `homing.c`와 ready/arm 계약은 skeleton 수준이라, startup auto-enable 제거와 fault latch는 앞으로 더 닫아야 한다."
      ],
      focus: ["PositionControl_CheckSafety()", "PositionControl_EmergencyStop()", "Relay_Emergency()", "Homing_Start()"]
    },
    {
      label: "Actuator",
      title: "제어 출력을 실제 pulse / direction 신호로 바꾸는 출력 계층",
      files: ["pulse_control.c", "tim.c"],
      role: "제어기에서 내려온 signed frequency를 실제 드라이브가 받을 수 있는 PE9 pulse와 PE10 direction 신호로 변환하는 영역이다.",
      why: "제어 계산이 맞아도 실제 구동은 이 계층이 만드는 waveform 품질에 달려 있으므로, requested Hz와 applied Hz를 분리해 보는 것이 중요하다.",
      how: [
        "`pulse_control.c`는 direction 부호를 해석하고 reverse guard를 적용한 뒤 TIM1 frequency를 설정한다.",
        "현재 런타임 계약은 10 ~ 100000 Hz clamp와 1 ms reverse guard를 사용한다.",
        "실제 bench에서는 drive monitor, scope, encoder 변화량을 함께 비교해 명령이 motion으로 이어졌는지 확인해야 한다."
      ],
      focus: ["PulseControl_SetFrequency()", "requested_frequency_hz", "applied_frequency_hz", "reverse_guard_active"]
    },
    {
      label: "Diagnostics / Evidence",
      title: "지금 상태를 사람이 이해할 수 있는 증거로 바꾸는 관측 계층",
      files: ["position_control_diag.c", "latency_profiler.c", "debug_vars.h"],
      role: "현재 명령이 어떤 상태인지, 제어가 얼마나 걸렸는지, fault가 왜 났는지를 UART / dbg_* / plots로 남기는 관측 영역이다.",
      why: "이 프로젝트는 단순히 움직이는 코드보다도 원인과 결과를 설명할 수 있는 evidence pack이 강점이므로, 이 계층이 있어야 bench 해석과 문서화가 가능하다.",
      how: [
        "`position_control_diag.c`는 command lifecycle과 state/result/fault 정보를 readable string과 debug var로 바꾼다.",
        "`latency_profiler.c`는 Sense / Control / Actuate / Comms 시간을 수집해 batch 통계로 남긴다.",
        "portal, PuTTY viewer, change log는 이 로그를 사람이 다시 해석하기 쉽게 묶는 바깥 레이어다."
      ],
      focus: ["CMD_START / CMD_REACHED / CMD_TIMEOUT / CMD_FAULT", "dbg_*", "LATENCY_STAGE", "Doc/change_code"]
    }
  ],
  moduleBriefs: [
    {
      file: "position_control.c",
      title: "폐루프 제어, command lifecycle, emergency 처리를 묶는 핵심 코어",
      summary: "이 파일은 실제 목표 추종을 수행하는 중심 모듈이다. target/current/error를 관리하고, PID 출력으로 pulse 명령을 만들며, 명령의 시작과 종료 이유까지 lifecycle로 남긴다.",
      inputs: ["target_angle[motor_deg]", "EncoderReader current angle", "HAL_GetTick() 기반 dt", "Safety evaluator 결과"],
      outputs: ["PulseControl_SetFrequency()", "CMD_START/REACHED/TIMEOUT/ABORT/FAULT 로그", "fault_flag / control_mode", "Relay_Emergency() 호출"],
      flow: [
        "제어가 켜져 있으면 1 ms 주기로 현재 각도와 속도를 계산하고 error를 갱신한다.",
        "active command가 timeout을 넘었는지 먼저 검사하고, 넘으면 즉시 emergency 경로로 전환한다.",
        "그 다음 safety evaluator 결과를 반영해 계속 구동 가능한지 확인한다.",
        "정상일 때만 PID_Calculate()를 수행해 signed output을 만들고 pulse_control로 전달한다.",
        "도달, 교체, disable, fault 같은 종료 이유를 command lifecycle에 기록한다."
      ],
      focus: ["PositionControl_Update()", "PositionControl_CommandStart()", "PositionControl_CommandFinish()", "PositionControl_EmergencyStop()", "controller_stats"],
      caution: "현재 timeout 기본값과 velocity limit 기본값이 0이면 일부 fail-safe가 사실상 꺼진 상태가 될 수 있고, startup ready gate와 강한 fault latch는 아직 미완성이다."
    },
    {
      file: "position_control_diag.c",
      title: "상태를 사람이 읽을 수 있는 문자열과 debug 변수로 바꾸는 진단 레이어",
      summary: "이 파일은 제어 코어를 직접 움직이지 않고, 현재 상태를 밖에서 보기 쉽게 번역하는 역할을 맡는다. command state/result/source를 문자열로 바꾸고 dbg_* 전역 변수를 동기화한다.",
      inputs: ["PositionControl_State_t snapshot", "control_enabled / control_mode", "fault_flag", "command lifecycle"],
      outputs: ["dbg_pos_mdeg", "dbg_target_mdeg", "dbg_err_mdeg", "dbg_fault_flags", "[PosCtrl] 상태 요약 printf"],
      flow: [
        "현재 각도, 목표각, 오차를 steering 기준 mdeg로 바꿔 debug global에 넣는다.",
        "fault_flag와 emergency/disable 상태를 조합해 bitmask fault flags를 만든다.",
        "command source/state/result enum을 짧은 문자열로 바꿔 로그에서 바로 읽게 한다.",
        "bench 중에는 compact summary 한 줄을 출력해 현재 상태를 빠르게 확인하게 돕는다."
      ],
      focus: ["PositionControlDiag_UpdateDebugVars()", "PositionControlDiag_PrintStateSummary()", "PositionControlDiag_CommandStateString()", "dbg_fault_flags"],
      caution: "이 모듈은 어디까지나 관측 계층이라 동작을 막거나 멈추지 않는다. 따라서 실제 fail-safe 판단은 position_control / safety 쪽과 함께 봐야 한다."
    },
    {
      file: "position_control_safety.c",
      title: "하드웨어를 직접 건드리지 않고 위험 여부만 판정하는 safety evaluator",
      summary: "이 파일은 각도, 추종 오차, 속도 제한을 평가해 안전한지 아닌지만 반환한다. 정지 동작 자체는 하지 않고, 판단 결과를 position_control이 받아 emergency stop으로 연결한다.",
      inputs: ["current_angle", "tracking_error", "measured_velocity_deg_per_s", "SafetyLimits_t"],
      outputs: ["PositionControlSafetyResult_t", "fault_flag", "POS_CTRL_ERR_*", "CMD_RESULT_FAULT_*"],
      flow: [
        "초기화 시 active safety limits snapshot을 만든다.",
        "입력된 limit 값을 절대값 기준으로 normalize해 저장한다.",
        "평가 시 soft angle margin, tracking error, velocity를 순서대로 검사한다.",
        "위반 시 어떤 fault인지 결과 코드만 돌려주고, actuator 쪽 정지는 상위 제어가 수행하게 둔다."
      ],
      focus: ["PositionControlSafety_Init()", "PositionControlSafety_SetLimits()", "PositionControlSafety_Evaluate()", "watchdog_timeout_ms"],
      caution: "현재 기본값에서 max_velocity와 watchdog_timeout_ms가 0이면 해당 항목은 비활성처럼 동작한다. 즉 safety skeleton은 있지만 모든 보호막이 기본으로 켜져 있는 것은 아니다."
    },
    {
      file: "pulse_control.c",
      title: "signed output을 실제 pulse / direction 파형으로 바꾸는 액추에이터 인터페이스",
      summary: "이 파일은 제어기의 출력값을 드라이브가 이해할 수 있는 PE9 pulse와 PE10 direction으로 바꾼다. 단순 PWM 출력이 아니라 direction 전환 시 reverse guard와 requested/applied 상태를 함께 관리한다.",
      inputs: ["signed pulse_hz", "step count", "target direction", "HAL tick"],
      outputs: ["PE9 pulse", "PE10 direction", "requested_frequency_hz", "applied_frequency_hz", "reverse_guard_active"],
      flow: [
        "입력 부호를 보고 CW/CCW 방향과 목표 주파수를 결정한다.",
        "주파수는 clamp를 거치고, 방향 반전이 필요하면 곧바로 바꾸지 않고 reverse guard로 대기한다.",
        "연속 구동 모드에서는 continuous output을 시작하고, step 모드에서는 PWM interrupt로 남은 스텝 수를 감소시킨다.",
        "현재 requested/applied Hz와 busy/output_active 상태를 별도 status 구조로 외부에 제공한다."
      ],
      focus: ["PulseControl_SetFrequency()", "PulseControl_SendSteps()", "PulseControl_GetStatus()", "requested_frequency_hz", "applied_frequency_hz"],
      caution: "제어기가 큰 Hz를 요청해도 실제 applied는 clamp와 reverse guard에 의해 달라질 수 있다. bench에서는 requested와 applied를 같이 봐야 병목 위치를 정확히 알 수 있다."
    },
    {
      file: "encoder_reader.c",
      title: "TIM2 raw counter를 unwrap count와 motor angle로 바꾸는 피드백 변환기",
      summary: "이 파일은 엔코더 하드웨어 카운터를 그대로 쓰지 않고, 16-bit delta를 누적해 long-run count와 motor_deg로 변환한다. bench에서는 optional virtual feedback도 같은 API로 제공한다.",
      inputs: ["TIM2->CNT raw counter", "offset_count", "virtual accum count", "HAL tick"],
      outputs: ["enc_raw", "enc_cnt", "delta_count", "motor_deg", "sample age"],
      flow: [
        "하드웨어 카운터 차이를 int16 delta로 해석해 wrap-around를 흡수하며 누적 count를 만든다.",
        "offset_count를 빼서 기준점을 맞춘 뒤 motor_deg로 환산한다.",
        "virtual feedback이 켜지면 하드웨어 대신 가상 accum count를 active source로 사용한다.",
        "sample API를 통해 raw / delta / accum / angle / age_ms를 한 번에 제공한다."
      ],
      focus: ["EncoderReader_UpdateCount()", "EncoderReader_GetAngleDeg()", "EncoderReader_GetSample()", "EncoderReader_EnableVirtualFeedback()", "[ENCDBG]"],
      caution: "virtual feedback을 켜면 같은 함수 이름으로도 real TIM2가 아닌 가상 count가 보일 수 있다. 실제 센서 진실성 확인은 `[ENCDBG]`와 하드웨어 파형을 함께 봐야 한다."
    },
    {
      file: "ethernet_communication.c",
      title: "UDP 패킷을 steering runtime이 쓰는 mode / target 계약으로 바꾸는 통신 계층",
      summary: "이 파일은 LwIP UDP 수신 데이터를 직접 제어기에 넘기지 않고, sender / packet size / mode 규칙을 통과한 값만 최신 steering packet 상태로 정리한다. 현재 기본 빌드에서는 keyboard 경로가 active지만 실제 시험 전환 시 핵심이 되는 파일이다.",
      inputs: ["UDP payload", "sender IP", "port", "ASMS mode packet", "PC steer/speed/misc packet"],
      outputs: ["g_latest_pkt", "g_current_mode", "g_last_rx_tick", "g_emergency_request", "g_new_data"],
      flow: [
        "수신 콜백에서 pbuf를 즉시 복사하고 free한 뒤, IPv4 sender와 packet length를 먼저 확인한다.",
        "ASMS 5B packet은 mode와 joystick 값을 처리하고, MANUAL 모드일 때 steering_angle로 변환한다.",
        "PC 9B packet은 AUTO 모드에서만 받아 steer/speed/misc를 최신 packet으로 저장한다.",
        "ESTOP bit가 오면 emergency_request를 세우고 mode를 ESTOP으로 전환한다."
      ],
      focus: ["udp_recv_cb()", "EthComm_UDP_Init()", "EthComm_HasNewData()", "EthComm_GetLatestData()", "g_last_rx_tick"],
      caution: "지금 bench 기본은 keyboard라 이 경로가 주 서비스 경로가 아니다. 하지만 실제 시험에서는 sender/length/mode 규칙이 바로 시스템 동작을 좌우하므로 contract 정리가 중요하다."
    },
    {
      file: "adc_potentiometer.c",
      title: "절대각 보조 센서를 raw / voltage / calibrated angle과 validity로 읽는 ADC 계층",
      summary: "이 파일은 포텐셔미터 ADC 값을 단순 raw로만 주지 않고, 보정된 각도와 validity bit까지 함께 계산한다. homing과 센서 cross-check를 준비하는 데 중요한 보조 모듈이다.",
      inputs: ["ADC1 sample", "보정 범위 min/max raw", "min/max angle", "diagnostic thresholds"],
      outputs: ["raw", "voltage", "calibrated_angle_deg", "validity bit", "sample age"],
      flow: [
        "ADC raw를 읽고 설정된 min/max raw, min/max angle 기준으로 각도로 환산한다.",
        "disconnect, range, jump, stuck 여부를 이전 샘플과 비교해 validity bit로 만든다.",
        "sample 구조체에서는 raw/voltage/angle/age/validity를 한 번에 제공한다.",
        "calibrate 함수는 현재 raw를 기준으로 최소/최대 각도 보정점을 갱신한다."
      ],
      focus: ["ADC_Pot_GetSample()", "ADC_Pot_EvaluateValidity()", "ADC_Pot_GetAngle()", "ADC_Pot_Calibrate()", "ADC_POT_INVALID_*"],
      caution: "현재는 startup/homing 보조 경로 성격이 강하고, position_control의 상시 safety gate에 완전히 통합되진 않았다. 그래서 중요한 센서이지만 아직 주 피드백 체인은 아니다."
    },
    {
      file: "homing.c",
      title: "ADC 절대각을 기준으로 encoder offset을 잡는 startup zero-reference skeleton",
      summary: "이 파일은 부팅 시 절대각을 읽어 encoder count 기준점을 맞추는 homing 역할을 맡는다. 현재는 간단한 zero-find skeleton이며, full startup state machine까지는 아직 연결되지 않았다.",
      inputs: ["ADC_PotSample", "SteeringDegToMotorDeg()", "EncoderReader_Reset()"],
      outputs: ["encoder offset", "homing status", "[Homing] logs"],
      flow: [
        "homing 시작 시 상태를 IN_PROGRESS로 바꾸고 ADC sample을 읽는다.",
        "ADC의 steering_deg를 motor_deg로 변환해 encoder 기준 count offset으로 바꾼다.",
        "먼저 EncoderReader_Reset()으로 count 기준을 초기화한 뒤 offset을 적용한다.",
        "성공하면 COMPLETE, 샘플 취득 실패면 ERROR 상태를 남긴다."
      ],
      focus: ["Homing_FindZero()", "Homing_GetStatus()", "Homing_Reset()", "offset_count"],
      caution: "현재 이 모듈은 존재하지만 실제 startup gating과 readiness contract 안으로 완전히 들어와 있지 않다. 즉 코드상 기능은 있지만 시스템 운영 정책은 아직 미완성이다."
    },
    {
      file: "relay_control.c",
      title: "SVON / EMG 핀을 직접 제어하는 가장 하드웨어 가까운 safety 출력 계층",
      summary: "이 파일은 servo on/off와 emergency 라인을 GPIO level로 직접 제어한다. active-low 계약이 명확해서 상위 제어기나 safety가 실제 하드웨어를 끊을 때 마지막으로 의존하는 모듈이다.",
      inputs: ["servo on/off 요청", "emergency 요청"],
      outputs: ["SVON pin level", "EMG pin level"],
      flow: [
        "초기화 시 기본 상태를 Servo OFF, EMG released로 둔다.",
        "ServoOn은 active-low라 LOW를 써서 구동 준비 상태로 바꾼다.",
        "Emergency는 active-low라 LOW를 써서 실제 정지 요청을 만든다.",
        "상위 모듈은 복잡한 GPIO 세부값 대신 이 함수를 통해 안전 출력을 일관되게 사용한다."
      ],
      focus: ["Relay_Init()", "Relay_ServoOn()", "Relay_ServoOff()", "Relay_Emergency()", "Relay_EmergencyRelease()"],
      caution: "기능은 단순하지만 영향은 가장 직접적이다. 그래서 startup auto-enable, fault latch, clear policy 같은 상위 정책이 정리되지 않으면 이 모듈이 너무 쉽게 호출될 수 있다."
    }
  ]
};

function makeEl(tag, className, text) {
  const el = document.createElement(tag);
  if (className) el.className = className;
  if (text !== undefined) el.textContent = text;
  return el;
}

function renderKpis() {
  const host = document.getElementById("kpi-strip");
  portalData.kpis.forEach((kpi) => {
    const card = makeEl("article", "kpi-card");
    card.append(makeEl("span", "metric-label", kpi.label));
    card.append(makeEl("strong", "", kpi.value));
    card.append(makeEl("p", "caption", kpi.note));
    host.append(card);
  });
}

function renderLatencyBars() {
  const host = document.getElementById("latency-bars");
  const max = Math.max(...portalData.latency.map((stage) => stage.p99));
  portalData.latency.forEach((stage) => {
    const row = makeEl("div", "latency-row");
    row.append(makeEl("strong", "", stage.name));
    const meter = makeEl("div", "latency-meter");
    const fill = makeEl("div", "latency-fill");
    fill.style.width = `${(stage.p99 / max) * 100}%`;
    meter.append(fill);
    row.append(meter);
    row.append(makeEl("span", "caption", `avg ${stage.avg.toFixed(3)} / p99 ${stage.p99.toFixed(3)} us`));
    host.append(row);
  });
}

function drawTrace(svgId, labels, target, current, options = {}) {
  const svg = document.getElementById(svgId);
  const width = 640;
  const height = svgId === "ideal-trace" ? 240 : 260;
  const margin = { top: 24, right: 20, bottom: 36, left: 48 };
  const innerW = width - margin.left - margin.right;
  const innerH = height - margin.top - margin.bottom;
  const minY = -50;
  const maxY = 50;
  const ns = "http://www.w3.org/2000/svg";

  svg.innerHTML = "";

  const root = document.createElementNS(ns, "g");
  root.setAttribute("transform", `translate(${margin.left},${margin.top})`);

  for (let i = 0; i <= 4; i += 1) {
    const line = document.createElementNS(ns, "line");
    const y = (innerH / 4) * i;
    line.setAttribute("x1", "0");
    line.setAttribute("x2", innerW);
    line.setAttribute("y1", y);
    line.setAttribute("y2", y);
    line.setAttribute("stroke", "rgba(33,92,147,0.10)");
    root.append(line);
  }

  const xScale = (index) => (labels.length <= 1 ? 0 : (innerW * index) / (labels.length - 1));
  const yScale = (value) => innerH - ((value - minY) / (maxY - minY)) * innerH;
  const buildPath = (series) =>
    series.map((value, index) => `${index === 0 ? "M" : "L"} ${xScale(index).toFixed(2)} ${yScale(value).toFixed(2)}`).join(" ");

  const targetPath = document.createElementNS(ns, "path");
  targetPath.setAttribute("d", buildPath(target));
  targetPath.setAttribute("fill", "none");
  targetPath.setAttribute("stroke", "#215c93");
  targetPath.setAttribute("stroke-width", "3");
  root.append(targetPath);

  const currentPath = document.createElementNS(ns, "path");
  currentPath.setAttribute("d", buildPath(current));
  currentPath.setAttribute("fill", "none");
  currentPath.setAttribute("stroke", "#0f7b79");
  currentPath.setAttribute("stroke-width", "3");
  if (options.dashed) currentPath.setAttribute("stroke-dasharray", "8 6");
  root.append(currentPath);

  labels.forEach((label, index) => {
    if (index % Math.max(1, Math.floor(labels.length / 6)) !== 0 && index !== labels.length - 1) return;
    const text = document.createElementNS(ns, "text");
    text.setAttribute("x", xScale(index));
    text.setAttribute("y", innerH + 24);
    text.setAttribute("text-anchor", "middle");
    text.setAttribute("font-size", "12");
    text.setAttribute("fill", "#56728c");
    text.textContent = label;
    root.append(text);
  });

  [50, 25, 0, -25, -50].forEach((value) => {
    const text = document.createElementNS(ns, "text");
    text.setAttribute("x", "-8");
    text.setAttribute("y", yScale(value) + 4);
    text.setAttribute("text-anchor", "end");
    text.setAttribute("font-size", "12");
    text.setAttribute("fill", "#56728c");
    text.textContent = `${value}`;
    root.append(text);
  });

  const legend = document.createElementNS(ns, "g");
  legend.setAttribute("transform", `translate(${innerW - 180}, 0)`);
  [
    { color: "#215c93", text: options.targetLabel || "target" },
    { color: "#0f7b79", text: options.currentLabel || "current" }
  ].forEach((entry, index) => {
    const y = 12 + index * 20;
    const line = document.createElementNS(ns, "line");
    line.setAttribute("x1", "0");
    line.setAttribute("x2", "18");
    line.setAttribute("y1", y);
    line.setAttribute("y2", y);
    line.setAttribute("stroke", entry.color);
    line.setAttribute("stroke-width", "3");
    legend.append(line);

    const text = document.createElementNS(ns, "text");
    text.setAttribute("x", "24");
    text.setAttribute("y", y + 4);
    text.setAttribute("font-size", "12");
    text.setAttribute("fill", "#56728c");
    text.textContent = entry.text;
    legend.append(text);
  });

  root.append(legend);
  svg.append(root);
}

function formatSigned(value, digits = 1, unit = "deg") {
  return `${value >= 0 ? "+" : ""}${value.toFixed(digits)} ${unit}`;
}

function setDialRotation(el, degrees) {
  el.style.transform = `translate(-50%, -88%) rotate(${degrees}deg)`;
}

function setDialMarker(el, degrees) {
  el.style.transform = `rotate(${degrees}deg) translateY(-78px)`;
}

function buildIdealTrace(targetDeg, durationSec) {
  const points = 32;
  const effectiveDuration = Math.max(durationSec, 0.2);
  const labelDigits = effectiveDuration >= 10 ? 1 : 2;
  const labels = Array.from({ length: points }, (_, index) =>
    `${(effectiveDuration * (index / (points - 1))).toFixed(labelDigits)}s`
  );
  const targetSeries = Array(points).fill(targetDeg);
  const currentSeries = Array.from({ length: points }, (_, index) => targetDeg * (index / (points - 1)));
  drawTrace("ideal-trace", labels, targetSeries, currentSeries, {
    targetLabel: "target (ideal)",
    currentLabel: "ideal response"
  });
}

function renderReqs() {
  const host = document.getElementById("req-table");
  portalData.reqs.forEach((req) => {
    const row = makeEl("article", "req-row");
    row.append(makeEl("div", "req-id", req.id));
    row.append(makeEl("span", `prio ${req.priority.toLowerCase()}`, req.priority));
    row.append(makeEl("div", "", req.owner));
    const detailBox = makeEl("div", "");
    detailBox.append(makeEl("strong", "", req.title));
    detailBox.append(makeEl("p", "caption", req.detail));
    row.append(detailBox);
    host.append(row);
  });
}

function renderEvaluation() {
  const data = portalData.evaluation;
  const summaryHost = document.getElementById("evaluation-summary");
  const interviewerHost = document.getElementById("evaluation-interviewer");
  const barsHost = document.getElementById("evaluation-bars");
  const reasonsHost = document.getElementById("evaluation-reasons");
  const to100Host = document.getElementById("evaluation-to100");

  summaryHost.innerHTML = `
    <p class="eyebrow">Interview Verdict</p>
    <div class="score-value">${data.score}%</div>
    <p><strong>현재 평점:</strong> ${data.verdict}</p>
    <p class="score-note">${data.summary}</p>
  `;

  interviewerHost.innerHTML = `
    <p class="eyebrow">Why This Score</p>
    <h3>면접관 코멘트</h3>
    <p>${data.interviewer}</p>
    <p class="caption">참고로 이 점수는 "production-complete autonomous steering subsystem" 기준이다. bring-up / debug / tooling 프로젝트만 놓고 보면 더 높게 평가될 수 있다.</p>
  `;

  data.categories.forEach((entry) => {
    const row = makeEl("div", "eval-row");
    row.innerHTML = `
      <div class="eval-head">
        <strong>${entry.name}</strong>
        <span class="eval-score">${entry.score} / 100</span>
      </div>
      <div class="eval-meter">
        <div class="eval-fill" style="width:${entry.score}%;"></div>
      </div>
      <p class="caption">${entry.detail}</p>
    `;
    barsHost.append(row);
  });

  data.reasons.forEach((entry) => {
    const note = makeEl("article", "eval-note");
    note.innerHTML = `<strong>${entry.title}</strong><p class="caption">${entry.detail}</p>`;
    reasonsHost.append(note);
  });

  data.to100.forEach((entry) => {
    const note = makeEl("article", "eval-note");
    note.innerHTML = `<strong>${entry.title}</strong><p class="caption">${entry.detail}</p>`;
    to100Host.append(note);
  });
}

function renderAnalysisFlow() {
  const data = portalData.analysisFlow;
  const stepHost = document.getElementById("analysis-step-grid");
  const loopHost = document.getElementById("analysis-loop-view");
  const logHost = document.getElementById("analysis-log-grid");

  data.steps.forEach((entry) => {
    const card = makeEl("article", "analysis-step-card");
    card.innerHTML = `
      <span class="step-pill">${entry.step}</span>
      <h3>${entry.title}</h3>
      <p>${entry.summary}</p>
      <div class="badge-row">${entry.vars.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
      <p class="caption" style="margin-top:12px;">Files: ${entry.files.join(", ")}</p>
      <p class="caption">${entry.status}</p>
    `;
    stepHost.append(card);
  });

  loopHost.innerHTML = `
    <div class="analysis-phase-grid">
      ${data.phases.map((entry) => `
        <article class="analysis-phase">
          <h4>${entry.name}</h4>
          <p>${entry.description}</p>
          <div class="badge-row">${entry.vars.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
        </article>
      `).join("")}
    </div>
  `;

  data.logs.forEach((entry) => {
    const card = makeEl("article", "log-card");
    card.innerHTML = `
      <h3>${entry.title}</h3>
      <pre class="log-snippet">${entry.snippet}</pre>
      <p class="caption">${entry.note}</p>
    `;
    logHost.append(card);
  });
}

function renderTeams() {
  const host = document.getElementById("team-grid");
  portalData.teams.forEach((team) => {
    const card = makeEl("article", "team-card");
    card.append(makeEl("span", "team-owner", team.owner));
    card.append(makeEl("h3", "", team.title));
    card.append(makeEl("p", "", team.scope));
    card.append(makeEl("p", "caption", `Deliverable: ${team.deliverable}`));
    card.append(makeEl("p", "caption", `Evidence: ${team.evidence}`));
    if (team.brief) {
      const link = makeEl("a", "team-brief-link", team.briefLabel || "상세 할당서");
      link.href = team.brief;
      card.append(link);
    }
    host.append(card);
  });
}

function renderCodeAreas() {
  const flowHost = document.getElementById("code-area-flow");
  const gridHost = document.getElementById("code-area-grid");

  portalData.codeAreaFlow.forEach((entry) => {
    const step = makeEl("article", "code-area-step");
    step.innerHTML = `
      <span class="step-pill">Step ${entry.step}</span>
      <h4>${entry.title}</h4>
      <p>${entry.detail}</p>
    `;
    flowHost.append(step);
  });

  portalData.codeAreas.forEach((entry) => {
    const card = makeEl("article", "code-area-card");
    card.innerHTML = `
      <div class="code-area-head">
        <span class="team-owner">${entry.label}</span>
        <h3>${entry.title}</h3>
      </div>
      <p>${entry.role}</p>
      <p class="caption"><strong>왜 필요한가:</strong> ${entry.why}</p>
      <div class="badge-row">${entry.files.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
      <div class="runtime-io-grid">
        <div class="runtime-io-box">
          <h4>어떻게 작동하나</h4>
          <ol class="number-list">${entry.how.map((item) => `<li>${item}</li>`).join("")}</ol>
        </div>
        <div class="runtime-io-box">
          <h4>지금 보면 좋은 포인트</h4>
          <div class="badge-row">${entry.focus.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
        </div>
      </div>
    `;
    gridHost.append(card);
  });
}

function renderModuleBriefs() {
  const host = document.getElementById("module-brief-grid");

  portalData.moduleBriefs.forEach((entry) => {
    const card = makeEl("article", "module-brief-card");
    card.innerHTML = `
      <span class="runtime-file-tag">${entry.file}</span>
      <h3>${entry.title}</h3>
      <p>${entry.summary}</p>
      <div class="runtime-io-grid">
        <div class="runtime-io-box">
          <h4>받는 것</h4>
          <ul class="plain-list">${entry.inputs.map((item) => `<li>${item}</li>`).join("")}</ul>
        </div>
        <div class="runtime-io-box">
          <h4>내보내는 것</h4>
          <ul class="plain-list">${entry.outputs.map((item) => `<li>${item}</li>`).join("")}</ul>
        </div>
      </div>
      <div class="module-brief-block">
        <h4>핵심 동작</h4>
        <ol class="number-list">${entry.flow.map((item) => `<li>${item}</li>`).join("")}</ol>
      </div>
      <div class="module-brief-block">
        <h4>지금 중요하게 볼 포인트</h4>
        <div class="badge-row">${entry.focus.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
      </div>
      <p class="caption"><strong>현재 주의점:</strong> ${entry.caution}</p>
    `;
    host.append(card);
  });
}

function renderRuntimeAtlas() {
  const flowHost = document.getElementById("runtime-flow");
  const gridHost = document.getElementById("runtime-atlas-grid");

  portalData.runtimeAtlas.forEach((entry) => {
    const stage = makeEl("article", "runtime-stage");
    stage.innerHTML = `
      <span class="runtime-file-tag">${entry.title}</span>
      <h3>${entry.role}</h3>
      <p class="caption">${entry.file}</p>
    `;
    flowHost.append(stage);

    const card = makeEl("article", "runtime-atlas-card");
    card.innerHTML = `
      <span class="runtime-file-tag">${entry.title}</span>
      <p class="runtime-role">${entry.role}</p>
      <div class="runtime-io-grid">
        <div class="runtime-io-box">
          <h4>받는 정보</h4>
          <ul class="plain-list">${entry.receives.map((item) => `<li>${item}</li>`).join("")}</ul>
        </div>
        <div class="runtime-io-box">
          <h4>보내는 정보</h4>
          <ul class="plain-list">${entry.sends.map((item) => `<li>${item}</li>`).join("")}</ul>
        </div>
      </div>
      <div class="runtime-list-block">
        <h4>핵심 함수</h4>
        <div class="badge-row">${entry.functions.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
      </div>
      <div class="runtime-list-block">
        <h4>핵심 변수 / 상태</h4>
        <div class="badge-row">${entry.variables.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
      </div>
    `;
    gridHost.append(card);
  });
}

function selectModule(moduleId) {
  const module = portalData.modules.find((entry) => entry.id === moduleId);
  const host = document.getElementById("module-detail");
  if (!module) return;

  document.querySelectorAll("[data-module-id]").forEach((el) => {
    el.classList.toggle("active", el.dataset.moduleId === moduleId);
  });

  host.innerHTML = `
    <h3>${module.title}</h3>
    <p class="module-meta">${module.subtitle}</p>
    <p><strong>Owner:</strong> ${module.owner}</p>
    <p>${module.notes}</p>
    <div class="dual-grid">
      <article class="card">
        <h3>Inputs</h3>
        <ul class="plain-list">${module.inputs.map((item) => `<li>${item}</li>`).join("")}</ul>
      </article>
      <article class="card">
        <h3>Outputs / Risks</h3>
        <ul class="plain-list">${module.outputs.map((item) => `<li>${item}</li>`).join("")}</ul>
        <p class="caption" style="margin-top:12px;">Risks: ${module.risks.join(", ")}</p>
      </article>
    </div>
    <pre class="code-snippet">${module.snippet}</pre>
    <div class="module-links">
      ${module.doxygen ? `<a href="${module.doxygen}">Doxygen source</a>` : ""}
      ${module.files.map((file) => `<a href="${file}">${file.split("/").slice(-2).join("/")}</a>`).join("")}
    </div>
  `;
}

function renderModules() {
  const mapHost = document.getElementById("module-map");
  const listHost = document.getElementById("module-list");
  portalData.modules.forEach((module, index) => {
    const chip = makeEl("button", "module-chip");
    chip.type = "button";
    chip.dataset.moduleId = module.id;
    chip.innerHTML = `<strong>${module.title}</strong><p class="caption">${module.subtitle}</p>`;
    chip.addEventListener("click", () => selectModule(module.id));
    mapHost.append(chip);

    const button = makeEl("button", "module-button");
    button.type = "button";
    button.dataset.moduleId = module.id;
    button.innerHTML = `<strong>${module.title}</strong><p class="caption">${module.owner}</p>`;
    button.addEventListener("click", () => selectModule(module.id));
    listHost.append(button);

    if (index === 0) selectModule(module.id);
  });
}

function setupSimulation() {
  const targetSlider = document.getElementById("target-slider");
  const freqSlider = document.getElementById("freq-slider");
  const replayButton = document.getElementById("replay-sim");
  const targetReadout = document.getElementById("target-readout");
  const freqReadout = document.getElementById("freq-readout");
  const motorTargetValue = document.getElementById("motor-target-value");
  const encoderTargetValue = document.getElementById("encoder-target-value");
  const pulseTargetValue = document.getElementById("pulse-target-value");
  const timeTargetValue = document.getElementById("time-target-value");
  const simStatus = document.getElementById("sim-status");
  const simProgressFill = document.getElementById("sim-progress-fill");
  const steeringNeedle = document.getElementById("steering-needle");
  const motorNeedle = document.getElementById("motor-needle");
  const steeringTargetMarker = document.getElementById("steering-target-marker");
  const motorTargetMarker = document.getElementById("motor-target-marker");
  const steeringCaption = document.getElementById("steering-caption");
  const motorCaption = document.getElementById("motor-caption");
  const freqStatus = document.getElementById("freq-status");

  let simRafId = 0;
  let simRestartTimer = 0;
  const simState = {
    targetDeg: Number(targetSlider.value),
    freqHz: Number(freqSlider.value),
    targetMotorDeg: Number(targetSlider.value) * PORTAL_STEERING_GEAR_RATIO,
    pulseCount: Math.round(Math.abs(Number(targetSlider.value)) * PORTAL_PULSE_PER_STEERING_DEG),
    encoderCount: Math.round(Math.abs(Number(targetSlider.value)) * PORTAL_ENCODER_COUNT_PER_STEERING_DEG),
    timeSec: 0,
    durationMs: 0,
    startTs: 0
  };

  const cancelSimulation = () => {
    if (simRafId) {
      cancelAnimationFrame(simRafId);
      simRafId = 0;
    }
    if (simRestartTimer) {
      clearTimeout(simRestartTimer);
      simRestartTimer = 0;
    }
  };

  const refreshMetrics = () => {
    const target = Number(targetSlider.value);
    const freq = Number(freqSlider.value);
    const motorDeg = target * PORTAL_STEERING_GEAR_RATIO;
    const encoderCount = Math.round(Math.abs(target) * PORTAL_ENCODER_COUNT_PER_STEERING_DEG);
    const pulseCount = Math.round(Math.abs(target) * PORTAL_PULSE_PER_STEERING_DEG);
    const timeSec = pulseCount / Math.max(freq, 1);
    const motorRev = motorDeg / 360;

    simState.targetDeg = target;
    simState.freqHz = freq;
    simState.targetMotorDeg = motorDeg;
    simState.encoderCount = encoderCount;
    simState.pulseCount = pulseCount;
    simState.timeSec = timeSec;
    simState.durationMs = Math.max(timeSec * 1000, 220);

    targetReadout.textContent = `${target.toFixed(1)} deg`;
    freqReadout.textContent = `${freq.toLocaleString()} Hz`;
    motorTargetValue.textContent = `${motorDeg.toFixed(0)} deg`;
    encoderTargetValue.textContent = `${encoderCount.toLocaleString()}`;
    pulseTargetValue.textContent = `${pulseCount.toLocaleString()}`;
    timeTargetValue.textContent = `${timeSec.toFixed(2)} s`;
    setDialMarker(steeringTargetMarker, target * 2);
    setDialMarker(motorTargetMarker, motorDeg);
    buildIdealTrace(target, timeSec);

    if (freq >= 100000) {
      freqStatus.textContent = "현재 firmware의 상한값 부근이다. 실제 코드에서는 100000 Hz clamp와 1 ms reverse guard를 사용하며, final bench closure는 real encoder truth를 authoritative하게 닫는 일이다.";
    } else {
      freqStatus.textContent = "현재 firmware contract 안쪽 값이다. 포털 시뮬레이션은 현재 코드의 pulse contract를 설명하기 위한 모델이며, 실제 bench에서는 drive monitor와 함께 해석해야 한다.";
    }
    steeringCaption.textContent = `Target ${formatSigned(target)} / Current ${formatSigned(0)}`;
    motorCaption.textContent = `Target ${formatSigned(motorDeg, 0)} / Current ${formatSigned(0, 0)} / ${motorRev.toFixed(2)} rev target`;
  };

  const renderProgress = (progress) => {
    const currentSteeringDeg = simState.targetDeg * progress;
    const currentMotorDeg = simState.targetMotorDeg * progress;
    const currentMotorRev = currentMotorDeg / 360;
    const errorDeg = simState.targetDeg - currentSteeringDeg;
    const currentPulse = Math.round(simState.pulseCount * progress);
    const elapsedSec = simState.timeSec * progress;

    setDialRotation(steeringNeedle, currentSteeringDeg * 2);
    setDialRotation(motorNeedle, currentMotorDeg);
    simProgressFill.style.width = `${(progress * 100).toFixed(1)}%`;
    steeringCaption.textContent = `Target ${formatSigned(simState.targetDeg)} / Current ${formatSigned(currentSteeringDeg)} / Error ${formatSigned(errorDeg)}`;
    motorCaption.textContent = `Target ${formatSigned(simState.targetMotorDeg, 0)} / Current ${formatSigned(currentMotorDeg, 0)} / ${currentMotorRev.toFixed(2)} rev`;

    if (progress >= 1) {
      simStatus.textContent = `complete · ${simState.timeSec.toFixed(2)} s · ${simState.pulseCount.toLocaleString()} pulse`;
    } else {
      simStatus.textContent = `running · ${elapsedSec.toFixed(2)} / ${simState.timeSec.toFixed(2)} s · ${currentPulse.toLocaleString()} / ${simState.pulseCount.toLocaleString()} pulse`;
    }
  };

  const animate = (timestamp) => {
    if (!simState.startTs) simState.startTs = timestamp;
    const elapsed = timestamp - simState.startTs;
    const progress = simState.durationMs <= 0 ? 1 : Math.min(elapsed / simState.durationMs, 1);
    renderProgress(progress);

    if (progress < 1) {
      simRafId = requestAnimationFrame(animate);
    } else {
      simRafId = 0;
      simState.startTs = 0;
    }
  };

  const startSimulation = () => {
    cancelSimulation();
    refreshMetrics();
    simState.startTs = 0;
    renderProgress(0);
    simRafId = requestAnimationFrame(animate);
  };

  const scheduleSimulation = () => {
    refreshMetrics();
    simStatus.textContent = "armed · slider 값 반영 후 재생 준비";
    simProgressFill.style.width = "0%";
    if (simRestartTimer) clearTimeout(simRestartTimer);
    simRestartTimer = setTimeout(() => {
      simRestartTimer = 0;
      startSimulation();
    }, 180);
  };

  targetSlider.addEventListener("input", scheduleSimulation);
  freqSlider.addEventListener("input", scheduleSimulation);
  targetSlider.addEventListener("change", startSimulation);
  freqSlider.addEventListener("change", startSimulation);
  replayButton.addEventListener("click", startSimulation);

  refreshMetrics();
  startSimulation();
}

function setupNav() {
  document.querySelectorAll(".nav-link").forEach((button) => {
    button.addEventListener("click", () => {
      const target = document.getElementById(button.dataset.target);
      if (target) target.scrollIntoView({ behavior: "smooth", block: "start" });
    });
  });

  const observer = new IntersectionObserver((entries) => {
    entries.forEach((entry) => {
      if (!entry.isIntersecting) return;
      document.querySelectorAll(".nav-link").forEach((button) => {
        button.classList.toggle("active", button.dataset.target === entry.target.id);
      });
    });
  }, { threshold: 0.28 });

  document.querySelectorAll("main section[id]").forEach((section) => observer.observe(section));
}

function init() {
  renderKpis();
  renderLatencyBars();
  drawTrace("actual-trace", portalData.actualTrace.labels, portalData.actualTrace.target, portalData.actualTrace.current, {
    targetLabel: "logged target",
    currentLabel: "logged current"
  });
  renderAnalysisFlow();
  renderEvaluation();
  renderReqs();
  renderTeams();
  renderRuntimeAtlas();
  renderCodeAreas();
  renderModuleBriefs();
  renderModules();
  setupSimulation();
  setupNav();
}

document.addEventListener("DOMContentLoaded", init);
