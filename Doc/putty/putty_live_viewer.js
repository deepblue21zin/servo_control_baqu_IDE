const TYPE_DEFS = [
  { key: "all", label: "All", note: "모든 기록" },
  { key: "latency", label: "Latency", note: "LATENCY_BATCH / STAGE" },
  { key: "keyboard", label: "Keyboard", note: "[KB] target / snapshot / action" },
  { key: "command", label: "Command", note: "CMD_START / REACHED / TIMEOUT / ABORT / FAULT" },
  { key: "csv", label: "CSV", note: "주기 상태 샘플" },
  { key: "encoder", label: "ENCDBG", note: "실제 TIM2 encoder 진단" },
  { key: "system", label: "System", note: "BOOT / VENC / header / 기타" }
];

const viewerState = {
  sourceMode: "none",
  sourceName: "",
  fetchPath: "putty_log/putty.log",
  fileHandle: null,
  liveEnabled: false,
  pollIntervalMs: 2000,
  liveTimerId: 0,
  eventSource: null,
  sourceVersion: 0,
  previousText: "",
  parsed: emptyParsedResult(),
  filterType: "all",
  selectedKey: "",
  observedKeys: new Set(),
  recording: {
    active: false,
    startedAt: 0,
    startedIso: "",
    sourceName: "",
    events: []
  },
  recordings: []
};

function emptyParsedResult() {
  return {
    rawText: "",
    rawLines: [],
    events: [],
    counts: buildEmptyCounts(),
    insights: [],
    totalLines: 0,
    lastUpdatedAt: 0
  };
}

function buildEmptyCounts() {
  return TYPE_DEFS.reduce((acc, typeDef) => {
    if (typeDef.key !== "all") acc[typeDef.key] = 0;
    return acc;
  }, {});
}

function qs(id) {
  return document.getElementById(id);
}

function makeEl(tag, className, text) {
  const el = document.createElement(tag);
  if (className) el.className = className;
  if (typeof text === "string") el.textContent = text;
  return el;
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function formatTime(ts) {
  if (!ts) return "-";
  return new Date(ts).toLocaleTimeString("ko-KR", { hour12: false });
}

function formatDateTime(ts) {
  if (!ts) return "-";
  return new Date(ts).toLocaleString("ko-KR", { hour12: false });
}

function getRequestedLogPath() {
  const inputValue = qs("fetch-path")?.value?.trim();
  if (!inputValue) return viewerState.fetchPath || "putty_log/putty.log";
  return inputValue.replace(/\\/g, "/");
}

function buildDirectLogUrl(path) {
  const normalized = (path || "").replace(/\\/g, "/").replace(/^\/+/, "");
  if (/^(https?:)?\/\//i.test(normalized)) return normalized;
  if (normalized.startsWith("./") || normalized.startsWith("../")) return normalized;
  return `../../${normalized}`;
}

function buildBridgeApiUrl(endpoint, path) {
  const normalized = (path || "putty_log/putty.log").replace(/\\/g, "/").replace(/^\/+/, "");
  return `/api/${endpoint}?path=${encodeURIComponent(normalized)}`;
}

function parseMaybeNumber(value) {
  if (value == null || value === "") return value;
  const normalized = String(value).replace(/deg$/i, "");
  const num = Number(normalized);
  return Number.isFinite(num) ? num : value;
}

function parseKeyValuePairs(text, delimiter = ",") {
  return text.split(delimiter).reduce((acc, token) => {
    const trimmed = token.trim();
    if (!trimmed) return acc;
    const eqIndex = trimmed.indexOf("=");
    if (eqIndex === -1) return acc;
    const key = trimmed.slice(0, eqIndex).trim();
    const value = trimmed.slice(eqIndex + 1).trim();
    acc[key] = value;
    return acc;
  }, {});
}

function parseSpacePairs(text) {
  const result = {};
  const regex = /([A-Za-z_]+)=([^\s]+)/g;
  let match = regex.exec(text);
  while (match) {
    result[match[1]] = match[2];
    match = regex.exec(text);
  }
  return result;
}

function buildEventBase(lineNo, raw, type, subtype, version) {
  return {
    key: `${version}:${lineNo}:${raw}`,
    lineNo,
    raw,
    type,
    subtype,
    level: "info",
    title: raw.length > 120 ? `${raw.slice(0, 117)}...` : raw,
    summary: "",
    interpretation: "",
    data: {}
  };
}

function mapObjectValues(object, mapper) {
  return Object.fromEntries(Object.entries(object).map(([key, value]) => [key, mapper(value)]));
}

function formatSignedNumber(value) {
  if (value == null || value === "" || Number.isNaN(Number(value))) return "-";
  const num = Number(value);
  return `${num >= 0 ? "+" : ""}${num.toFixed(Math.abs(num) >= 100 ? 0 : 2)}`;
}

function parseCsvRow(line, lineNo, version) {
  const parts = line.split(",");
  if (parts[0] === "CSV_HEADER") {
    const event = buildEventBase(lineNo, line, "csv", "header", version);
    event.title = "CSV header";
    event.summary = "CSV 컬럼 헤더";
    event.interpretation = "이후 CSV row가 어떤 컬럼 순서를 따르는지 정의한다.";
    event.data.columns = parts.slice(1);
    return event;
  }

  const event = buildEventBase(lineNo, line, "csv", "row", version);
  const values = parts.slice(1);
  const [
    ms,
    mode,
    targetDeg,
    currentDeg,
    errorDeg,
    output,
    dir,
    encCnt,
    encRaw,
    reqHz,
    appliedHz,
    outActive,
    revGuard,
    cmdId,
    cmdState,
    cmdResult
  ] = values;

  event.data = {
    ms: parseMaybeNumber(ms),
    mode: parseMaybeNumber(mode),
    targetDeg: parseMaybeNumber(targetDeg),
    currentDeg: parseMaybeNumber(currentDeg),
    errorDeg: parseMaybeNumber(errorDeg),
    output: parseMaybeNumber(output),
    dir: parseMaybeNumber(dir),
    encCnt: parseMaybeNumber(encCnt),
    encRaw: parseMaybeNumber(encRaw),
    reqHz: parseMaybeNumber(reqHz),
    appliedHz: parseMaybeNumber(appliedHz),
    outActive: parseMaybeNumber(outActive),
    revGuard: parseMaybeNumber(revGuard),
    cmdId: parseMaybeNumber(cmdId),
    cmdState,
    cmdResult
  };

  const err = Number(event.data.errorDeg || 0);
  const req = Number(event.data.reqHz || 0);
  const applied = Number(event.data.appliedHz || 0);
  event.title = `CSV @ ${event.data.ms ?? "?"} ms`;
  event.summary = `target ${formatSignedNumber(event.data.targetDeg)} / current ${formatSignedNumber(event.data.currentDeg)} / error ${formatSignedNumber(event.data.errorDeg)}`;

  if (Math.abs(err) >= 30 && Math.abs(Number(event.data.output || 0)) >= 9000) {
    event.level = "warn";
    event.interpretation = "오차가 큰 상태에서 출력이 포화 근처라 actuator 또는 feedback 체인을 같이 봐야 한다.";
  } else if (req && applied && req !== applied) {
    event.level = "warn";
    event.interpretation = "requested Hz와 applied Hz가 다르다. clamp 또는 guard 동작 여부를 확인할 필요가 있다.";
  } else {
    event.level = "ok";
    event.interpretation = "주기 샘플 한 줄이다. target/current/error/output/encoder 상태를 함께 보는 기본 프레임으로 쓰면 된다.";
  }

  return event;
}

function parseLatency(line, lineNo, version) {
  if (line.startsWith("LATENCY_BATCH_BEGIN")) {
    const event = buildEventBase(lineNo, line, "latency", "batch_begin", version);
    const data = parseKeyValuePairs(line.slice("LATENCY_BATCH_BEGIN,".length));
    event.data = mapObjectValues(data, parseMaybeNumber);
    event.title = `Latency batch #${event.data.seq ?? "?"} begin`;
    event.summary = `samples ${event.data.samples ?? "?"} / deadline_miss ${event.data.deadline_miss ?? 0}`;
    if (Number(event.data.deadline_miss || 0) > 0) {
      event.level = "warn";
      event.interpretation = `이 배치에서 deadline miss가 ${event.data.deadline_miss}회 발생했다. UART logging 또는 제어 부하를 함께 봐야 한다.`;
    } else {
      event.level = "ok";
      event.interpretation = "현재 배치 시작 시점 기준 deadline miss는 보고되지 않았다.";
    }
    return event;
  }

  if (line.startsWith("LATENCY_STAGE")) {
    const event = buildEventBase(lineNo, line, "latency", "stage", version);
    const data = parseKeyValuePairs(line.slice("LATENCY_STAGE,".length));
    event.data = mapObjectValues(data, parseMaybeNumber);
    event.title = `Latency ${event.data.name ?? "stage"} @ seq ${event.data.seq ?? "?"}`;
    event.summary = `avg ${event.data.avg_us ?? "?"} us / p99 ${event.data.p99_us ?? "?"} us / max ${event.data.max_us ?? "?"} us`;
    const maxUs = Number(event.data.max_us || 0);
    if (maxUs >= 1000) {
      event.level = "error";
      event.interpretation = "단일 stage 최대 시간이 1 ms budget에 닿거나 넘는다. 제어 루프 deadline에 직접 영향이 있을 수 있다.";
    } else if (maxUs >= 100) {
      event.level = "warn";
      event.interpretation = "stage max_us가 평소보다 큰 편이다. batch 전후 로그량과 blocking 구간을 확인하는 편이 좋다.";
    } else {
      event.level = "ok";
      event.interpretation = "현재 stage 시간은 1 ms loop budget 대비 충분히 낮다.";
    }
    return event;
  }

  const event = buildEventBase(lineNo, line, "latency", "batch_end", version);
  const data = parseKeyValuePairs(line.slice("LATENCY_BATCH_END,".length));
  event.data = mapObjectValues(data, parseMaybeNumber);
  event.title = `Latency batch #${event.data.seq ?? "?"} end`;
  event.summary = "latency batch 종료";
  event.level = "ok";
  event.interpretation = "한 개의 latency batch가 끝났다.";
  return event;
}

function parseKeyboard(line, lineNo, version) {
  const bracketMatch = line.match(/^\[KB\]\[(target|snapshot)\]\s+(.*)$/);
  if (bracketMatch) {
    const event = buildEventBase(lineNo, line, "keyboard", bracketMatch[1], version);
    const data = mapObjectValues(parseSpacePairs(bracketMatch[2]), parseMaybeNumber);
    event.data = data;
    event.title = `[KB][${bracketMatch[1]}] ${formatSignedNumber(data.T)} target`;
    event.summary = `current ${formatSignedNumber(data.C)} / error ${formatSignedNumber(data.E)} / output ${data.O ?? "?"}`;

    const absError = Math.abs(Number(data.E || 0));
    const absOutput = Math.abs(Number(data.O || 0));
    if (bracketMatch[1] === "snapshot" && absError >= 20 && absOutput >= 9000) {
      event.level = "warn";
      event.interpretation = "snapshot 기준 오차가 크고 출력이 포화에 가깝다. motion이 안 따라오면 feedback 또는 drive chain 이슈 가능성이 있다.";
    } else {
      event.level = bracketMatch[1] === "target" ? "info" : "ok";
      event.interpretation = bracketMatch[1] === "target"
        ? "keyboard에서 목표각을 바꾼 직후의 상태다."
        : "bench snapshot이다. target/current/error/output을 한 줄에서 확인할 수 있다.";
    }
    return event;
  }

  const event = buildEventBase(lineNo, line, "keyboard", "action", version);
  event.title = line.replace(/^\[KB\]\s*/, "[KB] ");
  event.summary = "keyboard console 메시지";

  if (/emergency stop/i.test(line)) {
    event.level = "error";
    event.interpretation = "keyboard에서 emergency stop이 요청됐다.";
  } else if (/unknown key/i.test(line) || /invalid target/i.test(line)) {
    event.level = "warn";
    event.interpretation = "입력 해석에 실패한 keyboard 이벤트다.";
  } else if (/control enabled/i.test(line)) {
    event.level = "ok";
    event.interpretation = "keyboard로 제어를 활성화했다.";
  } else if (/control disabled/i.test(line)) {
    event.level = "warn";
    event.interpretation = "keyboard로 제어를 비활성화했다.";
  } else if (/target steer/i.test(line)) {
    const steer = line.match(/steer=([-\d.]+)/);
    const motor = line.match(/motor=([-\d.]+)/);
    const ret = line.match(/ret=([-\d]+)/);
    event.data = {
      steerDeg: parseMaybeNumber(steer?.[1]),
      motorDeg: parseMaybeNumber(motor?.[1]),
      ret: parseMaybeNumber(ret?.[1])
    };
    event.summary = `steer ${formatSignedNumber(event.data.steerDeg)} / motor ${formatSignedNumber(event.data.motorDeg)}`;
    event.interpretation = Number(event.data.ret) === 0
      ? "keyboard 목표 설정 호출이 정상 리턴한 것으로 보인다."
      : "keyboard 목표 설정 호출이 에러 코드를 반환했다.";
    event.level = Number(event.data.ret) === 0 ? "info" : "warn";
  } else {
    event.level = "info";
    event.interpretation = "keyboard bench용 일반 안내 또는 상태 메시지다.";
  }

  return event;
}

function parseCommand(line, lineNo, version) {
  const prefixEnd = line.indexOf(",");
  const name = prefixEnd === -1 ? line : line.slice(0, prefixEnd);
  const payload = prefixEnd === -1 ? {} : mapObjectValues(parseKeyValuePairs(line.slice(prefixEnd + 1)), parseMaybeNumber);
  const event = buildEventBase(lineNo, line, "command", name.replace("CMD_", "").toLowerCase(), version);
  event.data = payload;
  event.title = name;

  switch (name) {
    case "CMD_START":
      event.summary = `id ${payload.id ?? "?"} / target ${formatSignedNumber(payload.target_deg)}`;
      event.level = "info";
      event.interpretation = "새 명령 lifecycle이 시작됐다.";
      break;
    case "CMD_REACHED":
      event.summary = `id ${payload.id ?? "?"} / final error ${formatSignedNumber(payload.final_error_deg)}`;
      event.level = "ok";
      event.interpretation = "명령이 settle 조건 안에 들어와 도달 처리됐다.";
      break;
    case "CMD_TIMEOUT":
      event.summary = `id ${payload.id ?? "?"} / elapsed ${payload.elapsed_ms ?? "?"} ms / error ${formatSignedNumber(payload.error_deg)}`;
      event.level = "error";
      event.interpretation = "명령이 제한 시간 안에 목표에 도달하지 못했다.";
      break;
    case "CMD_ABORT":
      event.summary = `id ${payload.id ?? "?"} / reason ${payload.reason ?? "-"}`;
      event.level = "warn";
      event.interpretation = "명령이 중간에 중단됐다. disable, replacement, estop 같은 원인을 확인해야 한다.";
      break;
    case "CMD_FAULT":
      event.summary = `id ${payload.id ?? "?"} / reason ${payload.reason ?? "-"}`;
      event.level = "error";
      event.interpretation = "fault reason 때문에 lifecycle이 종료됐다.";
      break;
    default:
      event.summary = "command lifecycle event";
      event.level = "info";
      event.interpretation = "명령 관련 이벤트다.";
      break;
  }

  return event;
}

function parseEncoder(line, lineNo, version) {
  const event = buildEventBase(lineNo, line, "encoder", "encdbg", version);
  const payload = line.replace(/^\[ENCDBG\]\s*/, "");
  event.data = mapObjectValues(parseSpacePairs(payload), parseMaybeNumber);
  event.title = `[ENCDBG] cnt ${event.data.cnt ?? "?"} / delta ${event.data.delta ?? "?"}`;
  event.summary = `A ${event.data.A ?? "?"} / B ${event.data.B ?? "?"} / prev ${event.data.prev ?? "?"}`;

  if (Number(event.data.delta || 0) === 0) {
    event.level = "warn";
    event.interpretation = "이 샘플에서는 TIM2 delta가 0이다. motion 중 계속 반복되면 feedback 정체를 의심할 수 있다.";
  } else {
    event.level = "ok";
    event.interpretation = "실제 TIM2 encoder delta가 관찰됐다.";
  }
  return event;
}

function parseSystem(line, lineNo, version) {
  const event = buildEventBase(lineNo, line, "system", "system", version);
  event.title = line;
  event.summary = "boot / system / 기타 메시지";
  event.level = line.startsWith("=~=~=~=~=~=~=~=~=~=~=~= PuTTY log") ? "info" : "ok";
  if (line.startsWith("[BOOT]")) {
    event.interpretation = "부팅 직후 runtime 상태를 알려주는 메시지다.";
  } else if (line.startsWith("[VENC]")) {
    event.interpretation = "virtual encoder 표시 정책 안내 메시지다.";
  } else if (line.startsWith("=~=~=~=~=~=~=~=~=~=~=~= PuTTY log")) {
    event.interpretation = "PuTTY 세션 로그 헤더다.";
  } else {
    event.interpretation = "분류되지 않은 system/other 메시지다.";
  }
  return event;
}

function parseLine(line, lineNo, version) {
  if (!line) return null;
  if (line.startsWith("CSV")) return parseCsvRow(line, lineNo, version);
  if (line.startsWith("LATENCY_BATCH_BEGIN") || line.startsWith("LATENCY_STAGE") || line.startsWith("LATENCY_BATCH_END")) {
    return parseLatency(line, lineNo, version);
  }
  if (line.startsWith("[KB]")) return parseKeyboard(line, lineNo, version);
  if (line.startsWith("CMD_")) return parseCommand(line, lineNo, version);
  if (line.startsWith("[ENCDBG]")) return parseEncoder(line, lineNo, version);
  return parseSystem(line, lineNo, version);
}

function parseLogText(text) {
  const normalizedText = text.replace(/\r/g, "");
  const rawLines = normalizedText.split("\n");
  const counts = buildEmptyCounts();
  const events = [];

  rawLines.forEach((rawLine, index) => {
    const line = rawLine.trim();
    if (!line) return;
    const event = parseLine(line, index + 1, viewerState.sourceVersion);
    if (!event) return;
    if (counts[event.type] != null) counts[event.type] += 1;
    events.push(event);
  });

  return {
    rawText: normalizedText,
    rawLines,
    events,
    counts,
    insights: [],
    totalLines: rawLines.filter((line) => line.trim() !== "").length,
    lastUpdatedAt: Date.now()
  };
}

function makeInsight(level, title, detail) {
  return { level, title, detail };
}

function buildFeedbackInsight(samples) {
  if (samples.length < 4) return null;

  const normalized = samples
    .map((event) => {
      if (event.type === "csv") {
        return {
          current: Number(event.data.currentDeg || 0),
          error: Number(event.data.errorDeg || 0),
          output: Number(event.data.output || 0)
        };
      }
      return {
        current: Number(event.data.C || 0),
        error: Number(event.data.E || 0),
        output: Number(event.data.O || 0)
      };
    })
    .slice(-8);

  const currentValues = normalized.map((item) => item.current);
  const errorValues = normalized.map((item) => Math.abs(item.error));
  const outputValues = normalized.map((item) => Math.abs(item.output));
  const currentSpan = Math.max(...currentValues) - Math.min(...currentValues);
  const largeError = Math.max(...errorValues);
  const largeOutput = Math.max(...outputValues);

  if (currentSpan <= 0.5 && largeError >= 15 && largeOutput >= 5000) {
    return makeInsight(
      "warn",
      "최근 상태 샘플에서 current 값이 거의 안 움직이는데 error와 output은 크다.",
      "controller는 명령을 내고 있지만 feedback 변화가 작다. drive accept 여부, real encoder truth, virtual feedback 설정을 같이 확인하는 편이 좋다."
    );
  }

  return null;
}

function buildInsights(events, counts) {
  const insights = [];
  const recentEvents = events.slice(-120);
  const failureEvent = [...recentEvents].reverse().find((event) => event.type === "command" && (event.level === "error" || event.level === "warn"));
  const deadlineMissSum = recentEvents
    .filter((event) => event.type === "latency" && event.subtype === "batch_begin")
    .reduce((sum, event) => sum + Number(event.data.deadline_miss || 0), 0);
  const worstStage = recentEvents
    .filter((event) => event.type === "latency" && event.subtype === "stage")
    .reduce((worst, event) => {
      const maxUs = Number(event.data.max_us || 0);
      if (!worst || maxUs > Number(worst.data.max_us || 0)) return event;
      return worst;
    }, null);
  const stateSamples = recentEvents.filter((event) => {
    if (event.type === "csv" && event.subtype === "row") return true;
    if (event.type === "keyboard" && (event.subtype === "target" || event.subtype === "snapshot")) return true;
    return false;
  });

  if (counts.command === 0 && counts.csv === 0 && counts.keyboard === 0) {
    insights.push(makeInsight("info", "아직 제어 상태 기록이 많지 않다.", "PuTTY가 막 시작됐거나 latency 중심 로그만 들어오는 중일 수 있다."));
  }

  if (deadlineMissSum > 0) {
    insights.push(makeInsight("warn", `최근 latency batch에서 deadline miss ${deadlineMissSum}회가 보였다.`, "interactive log 출력이 많은 순간과 겹치는지 함께 보는 편이 좋다."));
  } else if (counts.latency > 0) {
    insights.push(makeInsight("ok", "최근 latency batch에서는 deadline miss가 보고되지 않았다.", "현재 들어온 배치 기준으로는 1 ms 주기 여유가 있는 편으로 해석할 수 있다."));
  }

  if (worstStage) {
    insights.push(makeInsight(
      Number(worstStage.data.max_us || 0) >= 100 ? "warn" : "ok",
      `가장 큰 latency stage는 ${worstStage.data.name} ${worstStage.data.max_us} us다.`,
      "1 ms budget과 비교해 stage별 여유를 빠르게 확인하는 기준점으로 쓰면 된다."
    ));
  }

  if (failureEvent) {
    insights.push(makeInsight(
      failureEvent.level === "error" ? "error" : "warn",
      `최근 command 이슈: ${failureEvent.title}`,
      failureEvent.interpretation
    ));
  }

  const feedbackInsight = buildFeedbackInsight(stateSamples);
  if (feedbackInsight) insights.push(feedbackInsight);

  const encoderRecent = recentEvents.filter((event) => event.type === "encoder").slice(-10);
  if (encoderRecent.length >= 5 && encoderRecent.every((event) => Number(event.data.delta || 0) === 0)) {
    insights.push(makeInsight(
      "warn",
      "최근 ENCDBG 샘플 여러 개에서 delta가 계속 0이다.",
      "명령은 바뀌는데 encoder delta가 계속 0이면 실제 입력 체인이나 배선 상태를 다시 봐야 한다."
    ));
  }

  if (!insights.length) {
    insights.push(makeInsight("info", "해석 가능한 핵심 패턴이 아직 충분하지 않다.", "조금 더 로그가 쌓이면 deadline miss, command outcome, feedback 정체를 자동으로 요약한다."));
  }

  return insights.slice(0, 6);
}

function getFilteredEvents() {
  if (viewerState.filterType === "all") return viewerState.parsed.events;
  return viewerState.parsed.events.filter((event) => event.type === viewerState.filterType);
}

function eventTypeLabel(type) {
  return TYPE_DEFS.find((item) => item.key === type)?.label ?? type;
}

function pillClass(level) {
  if (level === "error") return "event-type-pill error";
  if (level === "warn") return "event-type-pill warn";
  if (level === "ok") return "event-type-pill ok";
  return "event-type-pill";
}

function ensureSelectedEventVisible() {
  const filtered = getFilteredEvents();
  if (!filtered.length) {
    viewerState.selectedKey = "";
    return;
  }
  const stillExists = filtered.some((event) => event.key === viewerState.selectedKey);
  if (!stillExists) viewerState.selectedKey = filtered.at(-1).key;
}

function renderSessionStatus() {
  const host = qs("session-metrics");
  const stamp = qs("session-stamp");
  host.innerHTML = "";

  const metrics = [
    { label: "Source", value: viewerState.sourceName || "not connected", note: viewerState.sourceMode },
    { label: "Mode", value: viewerState.liveEnabled ? "live polling" : "manual", note: `${viewerState.pollIntervalMs / 1000}s interval` },
    { label: "Events", value: String(viewerState.parsed.events.length), note: `${viewerState.parsed.totalLines} non-empty lines` },
    { label: "Updated", value: formatTime(viewerState.parsed.lastUpdatedAt), note: formatDateTime(viewerState.parsed.lastUpdatedAt) }
  ];

  metrics.forEach((metric) => {
    const card = makeEl("article", "viewer-metric-card");
    card.append(makeEl("span", "metric-label", metric.label));
    card.append(makeEl("strong", "", metric.value));
    card.append(makeEl("p", "caption", metric.note));
    host.append(card);
  });

  stamp.textContent = viewerState.liveEnabled ? "live" : "snapshot";
}

function renderInsights() {
  const host = qs("insight-list");
  const badge = qs("insight-badge");
  host.innerHTML = "";
  badge.textContent = `${viewerState.parsed.insights.length} insights`;

  if (!viewerState.parsed.insights.length) {
    const card = makeEl("article", "insight-card info");
    card.innerHTML = "<strong>로그를 연결하면 자동 해석이 나타난다.</strong><p class=\"caption\">deadline miss, command timeout, feedback 정체, 최근 target 변화 같은 힌트를 여기에 보여준다.</p>";
    host.append(card);
    return;
  }

  viewerState.parsed.insights.forEach((insight) => {
    const card = makeEl("article", `insight-card ${insight.level}`);
    card.innerHTML = `<strong>${escapeHtml(insight.title)}</strong><p class="caption">${escapeHtml(insight.detail)}</p>`;
    host.append(card);
  });
}

function renderFilters() {
  const host = qs("filter-chips");
  host.innerHTML = "";
  TYPE_DEFS.forEach((typeDef) => {
    const button = makeEl("button", `filter-chip ${viewerState.filterType === typeDef.key ? "active" : ""}`, `${typeDef.label}`);
    button.type = "button";
    button.addEventListener("click", () => {
      viewerState.filterType = typeDef.key;
      ensureSelectedEventVisible();
      renderEventViews();
    });
    host.append(button);
  });
}

function renderTypeCards() {
  const host = qs("type-card-grid");
  host.innerHTML = "";

  TYPE_DEFS.filter((item) => item.key !== "all").forEach((typeDef) => {
    const events = viewerState.parsed.events.filter((event) => event.type === typeDef.key);
    const latest = events.at(-1);
    const card = makeEl("article", `type-card ${viewerState.filterType === typeDef.key ? "active" : ""}`);
    card.innerHTML = `
      <div class="type-card-head">
        <strong>${escapeHtml(typeDef.label)}</strong>
        <span class="type-count">${events.length}</span>
      </div>
      <p class="caption">${escapeHtml(typeDef.note)}</p>
      <p>${escapeHtml(latest ? latest.title : "아직 없음")}</p>
      <p class="caption">${escapeHtml(latest ? latest.interpretation : "이 타입의 로그가 아직 들어오지 않았다.")}</p>
    `;
    card.addEventListener("click", () => {
      viewerState.filterType = typeDef.key;
      ensureSelectedEventVisible();
      renderEventViews();
    });
    host.append(card);
  });
}

function renderTimeline() {
  const host = qs("timeline-body");
  const caption = qs("timeline-caption");
  const filtered = getFilteredEvents();
  const visibleEvents = filtered.slice(-250);

  host.innerHTML = "";
  caption.textContent = `${visibleEvents.length} visible / ${filtered.length} filtered`;

  if (!visibleEvents.length) {
    host.innerHTML = `<tr><td colspan="4" class="empty-cell">현재 필터에 해당하는 이벤트가 없다.</td></tr>`;
    return;
  }

  visibleEvents.forEach((event) => {
    const row = document.createElement("tr");
    if (event.key === viewerState.selectedKey) row.classList.add("active");
    row.innerHTML = `
      <td>${event.lineNo}</td>
      <td><span class="${pillClass(event.level)}">${escapeHtml(eventTypeLabel(event.type))}</span></td>
      <td>${escapeHtml(event.title)}</td>
      <td>${escapeHtml(event.interpretation)}</td>
    `;
    row.addEventListener("click", () => {
      viewerState.selectedKey = event.key;
      renderDetail();
      renderTimeline();
    });
    host.append(row);
  });
}

function renderDetail() {
  const host = qs("detail-panel");
  const badge = qs("detail-type-badge");
  const event = viewerState.parsed.events.find((item) => item.key === viewerState.selectedKey);

  if (!event) {
    badge.textContent = "none";
    host.innerHTML = `<p class="caption">왼쪽 타임라인에서 항목을 선택하면 파싱 결과와 원본 로그가 표시된다.</p>`;
    return;
  }

  badge.textContent = `${eventTypeLabel(event.type)} / ${event.subtype}`;
  const detailRows = Object.entries(event.data || {}).map(([key, value]) => `
    <div class="detail-row">
      <div class="detail-key">${escapeHtml(key)}</div>
      <div class="mono">${escapeHtml(String(value))}</div>
    </div>
  `).join("");

  host.innerHTML = `
    <article class="insight-card ${event.level}">
      <strong>${escapeHtml(event.title)}</strong>
      <p>${escapeHtml(event.summary || "요약 없음")}</p>
      <p class="caption">${escapeHtml(event.interpretation || "해석 없음")}</p>
    </article>
    <div class="detail-list">
      <div class="detail-row">
        <div class="detail-key">line</div>
        <div class="mono">${event.lineNo}</div>
      </div>
      <div class="detail-row">
        <div class="detail-key">type</div>
        <div class="mono">${escapeHtml(event.type)} / ${escapeHtml(event.subtype)}</div>
      </div>
      ${detailRows || `
      <div class="detail-row">
        <div class="detail-key">parsed</div>
        <div class="mono">-</div>
      </div>`}
    </div>
    <pre class="detail-raw">${escapeHtml(event.raw)}</pre>
  `;
}

function renderEventViews() {
  renderFilters();
  renderTypeCards();
  renderTimeline();
  renderDetail();
}

function renderRecordingStatus() {
  const status = qs("recording-status");
  const list = qs("recording-list");
  const saveButton = qs("save-recording");
  const active = viewerState.recording.active;

  if (active) {
    status.textContent = `recording 중 · ${viewerState.recording.events.length} events captured`;
    status.className = "status-banner viewer-status-banner";
  } else {
    status.textContent = viewerState.recordings.length
      ? `recording 꺼짐 · 저장 대기 세션 ${viewerState.recordings.length}개`
      : "recording이 꺼져 있다.";
    status.className = "status-banner viewer-status-banner neutral";
  }

  saveButton.disabled = !viewerState.recordings.length && !viewerState.recording.events.length;
  list.innerHTML = "";

  if (active) {
    const activeCard = makeEl("article", "recording-card");
    activeCard.innerHTML = `
      <strong>현재 recording</strong>
      <div class="recording-meta">
        <span>source: ${escapeHtml(viewerState.recording.sourceName || "-")}</span>
        <span>started: ${escapeHtml(viewerState.recording.startedIso || "-")}</span>
        <span>captured: ${viewerState.recording.events.length} events</span>
      </div>
    `;
    list.append(activeCard);
  }

  viewerState.recordings.slice().reverse().forEach((recording, index) => {
    const card = makeEl("article", "recording-card");
    card.innerHTML = `
      <strong>Saved Session ${viewerState.recordings.length - index}</strong>
      <div class="recording-meta">
        <span>source: ${escapeHtml(recording.sourceName)}</span>
        <span>started: ${escapeHtml(recording.startedIso)}</span>
        <span>stopped: ${escapeHtml(recording.stoppedIso)}</span>
        <span>events: ${recording.events.length}</span>
      </div>
    `;
    list.append(card);
  });
}

function setSourceStatus(message, tone = "neutral") {
  const host = qs("source-status");
  host.textContent = message;
  host.className = tone === "warning"
    ? "status-banner viewer-status-banner"
    : "status-banner viewer-status-banner neutral";
}

async function readFromFetch() {
  const url = buildDirectLogUrl(viewerState.fetchPath);
  const response = await fetch(`${url}${url.includes("?") ? "&" : "?"}ts=${Date.now()}`, { cache: "no-store" });
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return response.text();
}

async function readFromBridgeSnapshot() {
  const response = await fetch(buildBridgeApiUrl("snapshot", viewerState.fetchPath), { cache: "no-store" });
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return response.json();
}

async function readFromFileHandle() {
  if (!viewerState.fileHandle) throw new Error("file handle not connected");
  const file = await viewerState.fileHandle.getFile();
  return file.text();
}

function appendLiveLine(line) {
  const nextText = viewerState.previousText
    ? `${viewerState.previousText}${viewerState.previousText.endsWith("\n") ? "" : "\n"}${line}`
    : line;
  updateFromText(nextText, viewerState.sourceName);
}

function closeEventSource() {
  if (viewerState.eventSource) {
    viewerState.eventSource.close();
    viewerState.eventSource = null;
  }
}

function openBridgeEventSource() {
  closeEventSource();
  const eventSource = new EventSource(`${buildBridgeApiUrl("live", viewerState.fetchPath)}&tail=1`);
  viewerState.eventSource = eventSource;

  eventSource.addEventListener("line", (event) => {
    try {
      const payload = JSON.parse(event.data);
      appendLiveLine(payload.line ?? "");
      setSourceStatus(`Bridge live 수신 중: ${viewerState.fetchPath}`);
    } catch (error) {
      setSourceStatus(`Bridge line parse 실패: ${error.message}`, "warning");
    }
  });

  eventSource.addEventListener("reset", () => {
    readFromBridgeSnapshot()
      .then((payload) => updateFromText(payload.text || "", payload.path || viewerState.fetchPath))
      .then(() => setSourceStatus("로그가 회전되거나 초기화되어 bridge snapshot을 다시 읽었다."))
      .catch((error) => setSourceStatus(`Bridge reset 처리 실패: ${error.message}`, "warning"));
  });

  eventSource.addEventListener("error", () => {
    if (viewerState.sourceMode === "bridge" && viewerState.liveEnabled) {
      setSourceStatus("Bridge 연결이 잠시 끊겼다. 자동 재연결을 기다리는 중이다.", "warning");
    }
  });
}

async function updateFromText(text, sourceName) {
  if (viewerState.previousText && text !== viewerState.previousText && !text.startsWith(viewerState.previousText)) {
    viewerState.sourceVersion += 1;
    viewerState.observedKeys = new Set();
    setSourceStatus("로그 파일이 회전되었거나 중간부터 다시 시작된 것으로 보여 source version을 새로 잡았다.", "warning");
  }

  viewerState.previousText = text;
  viewerState.sourceName = sourceName;
  viewerState.parsed = parseLogText(text);
  viewerState.parsed.insights = buildInsights(viewerState.parsed.events, viewerState.parsed.counts);
  captureNewRecordingEvents(viewerState.parsed.events);
  ensureSelectedEventVisible();
  renderSessionStatus();
  renderInsights();
  renderEventViews();
  renderRecordingStatus();
}

function captureNewRecordingEvents(events) {
  const newEvents = [];
  events.forEach((event) => {
    if (viewerState.observedKeys.has(event.key)) return;
    viewerState.observedKeys.add(event.key);
    newEvents.push(event);
  });

  if (viewerState.recording.active && newEvents.length) {
    const cloned = newEvents.map((event) => ({
      ...event,
      data: { ...event.data }
    }));
    viewerState.recording.events.push(...cloned);
  }
}

async function refreshNow() {
  try {
    let text = "";
    if (viewerState.sourceMode === "bridge") {
      const payload = await readFromBridgeSnapshot();
      await updateFromText(payload.text || "", payload.path || viewerState.fetchPath);
      setSourceStatus(`Bridge snapshot을 다시 읽었다: ${payload.path || viewerState.fetchPath}`);
      return;
    }

    if (viewerState.sourceMode === "fetch") {
      text = await readFromFetch();
      await updateFromText(text, viewerState.fetchPath);
      setSourceStatus(`파일 polling으로 다시 읽었다: ${viewerState.fetchPath}`);
      return;
    }

    if (viewerState.sourceMode === "handle") {
      text = await readFromFileHandle();
      await updateFromText(text, viewerState.fileHandle.name || "local file");
      setSourceStatus(`로컬 파일 권한으로 다시 읽었다: ${viewerState.fileHandle.name || "local file"}`);
      return;
    }

    setSourceStatus("현재는 auto refresh할 소스가 없다. fetch 또는 file handle 연결을 먼저 해 달라.", "warning");
  } catch (error) {
    setSourceStatus(`로그를 읽는 중 오류가 발생했다: ${error.message}`, "warning");
    stopLivePolling();
  }
}

function stopLivePolling() {
  if (viewerState.liveTimerId) {
    clearInterval(viewerState.liveTimerId);
    viewerState.liveTimerId = 0;
  }
  closeEventSource();
  viewerState.liveEnabled = false;
  qs("toggle-live").textContent = "Auto refresh OFF";
  renderSessionStatus();
}

function startBridgeLive() {
  stopLivePolling();
  viewerState.liveEnabled = true;
  qs("toggle-live").textContent = "Auto refresh ON";
  openBridgeEventSource();
  renderSessionStatus();
}

function startLivePolling() {
  stopLivePolling();
  viewerState.liveEnabled = true;
  qs("toggle-live").textContent = "Auto refresh ON";
  viewerState.liveTimerId = window.setInterval(() => {
    refreshNow();
  }, viewerState.pollIntervalMs);
  refreshNow();
  renderSessionStatus();
}

function toggleLivePolling() {
  if (viewerState.liveEnabled) {
    stopLivePolling();
    setSourceStatus("Auto refresh를 중지했다.");
  } else {
    if (viewerState.sourceMode === "bridge") {
      startBridgeLive();
      setSourceStatus("Bridge live를 다시 시작했다.");
      return;
    }
    if (viewerState.sourceMode !== "fetch" && viewerState.sourceMode !== "handle") {
      setSourceStatus("live update는 bridge, fetch, file handle 소스를 먼저 연결해야 한다.", "warning");
      return;
    }
    startLivePolling();
    setSourceStatus("Auto refresh를 시작했다.");
  }
}

async function connectBridgeSource() {
  stopLivePolling();
  viewerState.fetchPath = getRequestedLogPath();
  viewerState.sourceMode = "bridge";
  viewerState.sourceName = viewerState.fetchPath;
  const payload = await readFromBridgeSnapshot();
  await updateFromText(payload.text || "", payload.path || viewerState.fetchPath);
  startBridgeLive();
  setSourceStatus(`Bridge live 연결 완료: ${payload.path || viewerState.fetchPath}`);
}

async function connectFetchSource() {
  stopLivePolling();
  viewerState.fetchPath = getRequestedLogPath();
  viewerState.sourceMode = "fetch";
  viewerState.sourceName = viewerState.fetchPath;
  await refreshNow();
}

async function connectFileHandleSource() {
  stopLivePolling();
  if (!window.showOpenFilePicker) {
    setSourceStatus("이 브라우저 또는 현재 열기 방식에서는 파일 권한 연결 API를 지원하지 않는다. localhost로 열거나 snapshot 업로드를 사용해 달라.", "warning");
    return;
  }

  const [handle] = await window.showOpenFilePicker({
    multiple: false,
    types: [
      {
        description: "PuTTY log",
        accept: {
          "text/plain": [".log", ".txt"]
        }
      }
    ]
  });

  viewerState.fileHandle = handle;
  viewerState.sourceMode = "handle";
  await refreshNow();
}

async function handleUpload(event) {
  const file = event.target.files?.[0];
  if (!file) return;
  const text = await file.text();
  viewerState.sourceMode = "upload";
  viewerState.sourceName = file.name;
  stopLivePolling();
  await updateFromText(text, file.name);
  setSourceStatus(`snapshot 업로드를 읽었다: ${file.name}`);
}

function toggleRecording() {
  const button = qs("toggle-recording");

  if (viewerState.recording.active) {
    viewerState.recordings.push({
      sourceName: viewerState.recording.sourceName || viewerState.sourceName || "unknown",
      startedAt: viewerState.recording.startedAt,
      startedIso: viewerState.recording.startedIso,
      stoppedAt: Date.now(),
      stoppedIso: formatDateTime(Date.now()),
      events: viewerState.recording.events.slice()
    });
    viewerState.recording = {
      active: false,
      startedAt: 0,
      startedIso: "",
      sourceName: "",
      events: []
    };
    button.textContent = "Recording 시작";
  } else {
    viewerState.recording = {
      active: true,
      startedAt: Date.now(),
      startedIso: formatDateTime(Date.now()),
      sourceName: viewerState.sourceName || "unknown",
      events: []
    };
    button.textContent = "Recording 종료";
  }

  renderRecordingStatus();
}

async function saveRecording() {
  const activeEvents = viewerState.recording.active ? viewerState.recording.events.slice() : [];
  const sessions = viewerState.recordings.slice();

  if (activeEvents.length) {
    sessions.push({
      sourceName: viewerState.recording.sourceName || viewerState.sourceName || "unknown",
      startedAt: viewerState.recording.startedAt,
      startedIso: viewerState.recording.startedIso,
      stoppedAt: Date.now(),
      stoppedIso: formatDateTime(Date.now()),
      events: activeEvents
    });
  }

  if (!sessions.length) {
    setSourceStatus("저장할 recording 세션이 없다.", "warning");
    return;
  }

  const payload = {
    savedAt: new Date().toISOString(),
    sourceMode: viewerState.sourceMode,
    sourceName: viewerState.sourceName,
    pollIntervalMs: viewerState.pollIntervalMs,
    sessionCount: sessions.length,
    sessions
  };

  const text = JSON.stringify(payload, null, 2);
  const filename = `putty_recording_${new Date().toISOString().replace(/[:.]/g, "-")}.json`;

  if (window.showSaveFilePicker) {
    try {
      const handle = await window.showSaveFilePicker({
        suggestedName: filename,
        types: [
          {
            description: "JSON recording",
            accept: { "application/json": [".json"] }
          }
        ]
      });
      const writable = await handle.createWritable();
      await writable.write(text);
      await writable.close();
      setSourceStatus(`recording 세션을 저장했다: ${filename}`);
      return;
    } catch (error) {
      if (error.name !== "AbortError") {
        setSourceStatus(`Save picker 저장에 실패했다. 다운로드 방식으로 전환한다: ${error.message}`, "warning");
      }
    }
  }

  const blob = new Blob([text], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
  setSourceStatus(`recording 세션을 다운로드로 저장했다: ${filename}`);
}

function clearRecordings() {
  viewerState.recordings = [];
  if (!viewerState.recording.active) {
    viewerState.recording.events = [];
  }
  renderRecordingStatus();
}

function bindEvents() {
  qs("connect-stream").addEventListener("click", () => {
    connectBridgeSource().catch((error) => {
      setSourceStatus(`Bridge live 연결 실패: ${error.message}`, "warning");
    });
  });

  qs("connect-fetch").addEventListener("click", () => {
    connectFetchSource().catch((error) => {
      setSourceStatus(`파일 polling 연결 실패: ${error.message}`, "warning");
    });
  });

  qs("connect-file-handle").addEventListener("click", () => {
    connectFileHandleSource().catch((error) => {
      if (error.name === "AbortError") return;
      setSourceStatus(`파일 권한 연결 실패: ${error.message}`, "warning");
    });
  });

  qs("upload-log").addEventListener("change", (event) => {
    handleUpload(event).catch((error) => {
      setSourceStatus(`snapshot 업로드 실패: ${error.message}`, "warning");
    });
  });

  qs("toggle-live").addEventListener("click", toggleLivePolling);
  qs("refresh-now").addEventListener("click", () => {
    refreshNow();
  });
  qs("toggle-recording").addEventListener("click", toggleRecording);
  qs("save-recording").addEventListener("click", () => {
    saveRecording().catch((error) => {
      setSourceStatus(`recording 저장 실패: ${error.message}`, "warning");
    });
  });
  qs("clear-recordings").addEventListener("click", clearRecordings);
  qs("poll-interval").addEventListener("change", (event) => {
    viewerState.pollIntervalMs = Number(event.target.value);
    if (viewerState.liveEnabled) {
      if (viewerState.sourceMode === "bridge") {
        startBridgeLive();
      } else {
        startLivePolling();
      }
    }
    renderSessionStatus();
  });
  qs("fetch-path").addEventListener("change", (event) => {
    viewerState.fetchPath = event.target.value.trim() || "putty_log/putty.log";
  });
}

function init() {
  bindEvents();
  renderSessionStatus();
  renderInsights();
  renderFilters();
  renderTypeCards();
  renderTimeline();
  renderDetail();
  renderRecordingStatus();

  const params = new URLSearchParams(window.location.search);
  const requestedPath = params.get("path");
  if (requestedPath) {
    qs("fetch-path").value = requestedPath;
    viewerState.fetchPath = requestedPath;
  }

  if (params.get("autoconnect") === "bridge") {
    connectBridgeSource().catch((error) => {
      setSourceStatus(`자동 bridge 연결 실패: ${error.message}`, "warning");
    });
  }
}

document.addEventListener("DOMContentLoaded", init);
