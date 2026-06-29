const state = {
  registerImageWidth: 1,
  registerImageHeight: 1,
  scaleMinPoint: null,
  scaleMaxPoint: null,
  detectTimer: null,
  detecting: false,
  fpsTimestamps: [],
};

const el = {
  globalStatus: document.getElementById("globalStatus"),
  registerImage: document.getElementById("registerImage"),
  registerEmpty: document.getElementById("registerEmpty"),
  registerMeta: document.getElementById("registerMeta"),
  registerResult: document.getElementById("registerResult"),
  detectImage: document.getElementById("detectImage"),
  detectEmpty: document.getElementById("detectEmpty"),
  detectResult: document.getElementById("detectResult"),
  detectStatusCard: document.getElementById("detectStatusCard"),
  detectStatusValue: document.getElementById("detectStatusValue"),
  detectMessageValue: document.getElementById("detectMessageValue"),
  detectNeedleValue: document.getElementById("detectNeedleValue"),
  detectFpsValue: document.getElementById("detectFpsValue"),
  detectMarkerValue: document.getElementById("detectMarkerValue"),
  detectAngleValue: document.getElementById("detectAngleValue"),
  detectCircleValue: document.getElementById("detectCircleValue"),
  detectNeedleTipValue: document.getElementById("detectNeedleTipValue"),
  detectScaleValue: document.getElementById("detectScaleValue"),
  detectConfigValue: document.getElementById("detectConfigValue"),
  configSettingsList: document.getElementById("configSettingsList"),
  configRawJson: document.getElementById("configRawJson"),
  configSelect: document.getElementById("configSelectInput"),
  selectedConfig: document.getElementById("selectedConfigInput"),
  reloadConfigButton: document.getElementById("reloadConfigButton"),
  dictionary: document.getElementById("dictionaryInput"),
  markerId: document.getElementById("markerIdInput"),
  maxWidth: document.getElementById("maxWidthInput"),
  cameraIndex: document.getElementById("cameraIndexInput"),
  detectCameraIndex: document.getElementById("detectCameraIndexInput"),
  clickMode: document.getElementById("clickModeInput"),
  centerX: document.getElementById("centerXInput"),
  centerY: document.getElementById("centerYInput"),
  radius: document.getElementById("radiusInput"),
  scaleMinValue: document.getElementById("scaleMinValueInput"),
  scaleMaxValue: document.getElementById("scaleMaxValueInput"),
  scaleDirection: document.getElementById("scaleDirectionInput"),
  drawText: document.getElementById("drawTextInput"),
  detectInterval: document.getElementById("detectIntervalInput"),
  captureRegisterButton: document.getElementById("captureRegisterButton"),
  uploadRegisterInput: document.getElementById("uploadRegisterInput"),
  detectMarkerButton: document.getElementById("detectMarkerButton"),
  autoCircleButton: document.getElementById("autoCircleButton"),
  previewNeedleButton: document.getElementById("previewNeedleButton"),
  autoScaleButton: document.getElementById("autoScaleButton"),
  clearScaleButton: document.getElementById("clearScaleButton"),
  saveRegistrationButton: document.getElementById("saveRegistrationButton"),
  startDetectButton: document.getElementById("startDetectButton"),
  stopDetectButton: document.getElementById("stopDetectButton"),
};

function setStatus(message, kind = "") {
  el.globalStatus.textContent = message;
  el.globalStatus.classList.remove("ok", "error");
  if (kind) {
    el.globalStatus.classList.add(kind);
  }
}

function numberValue(input, fallback = 0) {
  const value = Number(input.value);
  return Number.isFinite(value) ? value : fallback;
}

function setText(element, value) {
  if (element) {
    element.textContent = value;
  }
}

function formatNumber(value, digits = 2) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) {
    return "-";
  }
  return Number(value).toFixed(digits);
}

function formatPoint(point) {
  if (!Array.isArray(point) || point.length < 2) {
    return "-";
  }
  return `(${Math.round(point[0])}, ${Math.round(point[1])})`;
}

function formatCircle(circle) {
  if (!circle) {
    return "-";
  }
  return `(${circle.x}, ${circle.y}) / r=${circle.radius}`;
}

function formatNeedleValue(value, hasNeedle) {
  if (!hasNeedle) {
    return "-";
  }
  if (value === null || value === undefined) {
    return "未登録";
  }
  return formatNumber(value, 6);
}

function formatFraction(value) {
  if (value === null || value === undefined) {
    return "未登録";
  }
  return `${formatNumber(Number(value) * 100, 2)}%`;
}

function resetFps() {
  state.fpsTimestamps = [];
  setText(el.detectFpsValue, "-");
}

function updateFps() {
  const now = performance.now();
  state.fpsTimestamps.push(now);
  if (state.fpsTimestamps.length > 12) {
    state.fpsTimestamps.shift();
  }
  if (state.fpsTimestamps.length < 2) {
    setText(el.detectFpsValue, "-");
    return;
  }
  const elapsed = state.fpsTimestamps[state.fpsTimestamps.length - 1] - state.fpsTimestamps[0];
  const fps = elapsed > 0 ? (state.fpsTimestamps.length - 1) * 1000 / elapsed : 0;
  setText(el.detectFpsValue, `${fps.toFixed(1)} fps`);
}

function currentSettings() {
  return {
    dictionary: el.dictionary.value,
    marker_id: numberValue(el.markerId, 0),
    max_width: numberValue(el.maxWidth, 1280),
    camera_index: numberValue(el.cameraIndex, 0),
    circle: {
      x: numberValue(el.centerX, 0),
      y: numberValue(el.centerY, 0),
      radius: Math.max(1, numberValue(el.radius, 1)),
    },
    scale_min_value: numberValue(el.scaleMinValue, 0.0),
    scale_max_value: numberValue(el.scaleMaxValue, 0.1),
    scale_direction: el.scaleDirection.value,
    scale_min_point: state.scaleMinPoint,
    scale_max_point: state.scaleMaxPoint,
    draw_text: el.drawText.checked,
  };
}

async function postJson(url, payload = {}) {
  const response = await fetch(url, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload),
  });
  const data = await response.json();
  if (!response.ok || !data.ok) {
    throw new Error(data.error || `API error: ${response.status}`);
  }
  return data;
}

function applyRegisterResponse(data) {
  if (data.image) {
    el.registerImage.src = data.image;
    el.registerImage.classList.add("loaded");
    el.registerEmpty.classList.add("hidden");
  }
  if (data.image_width && data.image_height) {
    state.registerImageWidth = data.image_width;
    state.registerImageHeight = data.image_height;
  }
  if (data.circle) {
    el.centerX.value = data.circle.x;
    el.centerY.value = data.circle.y;
    el.radius.value = data.circle.radius;
  }
  if (data.dictionary) {
    el.dictionary.value = data.dictionary;
  }
  if (data.marker_id !== undefined && data.marker_id !== null) {
    el.markerId.value = data.marker_id;
  }
  if (data.max_width) {
    el.maxWidth.value = data.max_width;
  }
  if (data.scale_min_value !== undefined && data.scale_min_value !== null) {
    el.scaleMinValue.value = data.scale_min_value;
  }
  if (data.scale_max_value !== undefined && data.scale_max_value !== null) {
    el.scaleMaxValue.value = data.scale_max_value;
  }
  if (data.scale_direction) {
    el.scaleDirection.value = data.scale_direction;
  }
  if (data.config_files) {
    applyConfigFiles(data.config_files, data.config_name);
  }
  if (data.config_settings !== undefined) {
    applyConfigSettings(data.config_settings);
  }
  if (data.config_name) {
    el.selectedConfig.value = data.config_name;
  }
  state.scaleMinPoint = data.scale_min_point || null;
  state.scaleMaxPoint = data.scale_max_point || null;

  const meta = [
    data.image_name ? `image: ${data.image_name}` : "",
    data.source_width ? `source: ${data.source_width} x ${data.source_height}` : "",
    data.image_width ? `work: ${data.image_width} x ${data.image_height}` : "",
    data.work_scale ? `scale=${Number(data.work_scale).toFixed(4)}` : "",
    data.config_name ? `config: ${data.config_name}` : "",
  ].filter(Boolean);
  el.registerMeta.textContent = meta.join(" / ") || "-";

  const result = {
    message: data.message || "",
    circle: data.circle || null,
    scale_min_point: state.scaleMinPoint,
    scale_max_point: state.scaleMaxPoint,
    marker: data.marker || null,
    needle_detection: data.needle_detection || undefined,
    registration: data.registration || undefined,
  };
  el.registerResult.textContent = JSON.stringify(result, null, 2);
}

function applyDetectResponse(data) {
  if (data.image) {
    el.detectImage.src = data.image;
    el.detectImage.classList.add("loaded");
    el.detectEmpty.classList.add("hidden");
    updateFps();
  }
  if (data.config_settings !== undefined) {
    applyConfigSettings(data.config_settings);
  }
  const needleValue = data.needle_value === null || data.needle_value === undefined
    ? "未登録"
    : Number(data.needle_value).toFixed(6);
  const fraction = data.scale_fraction === null || data.scale_fraction === undefined
    ? "未登録"
    : `${(Number(data.scale_fraction) * 100).toFixed(2)}%`;
  const markerText = data.marker_detected
    ? `検出${data.marker && data.marker.marker_id !== undefined ? ` id=${data.marker.marker_id}` : ""}`
    : "未検出";
  const statusText = data.detection_ok === false
    ? (data.marker_detected === false ? "AR未検出 / 針検出なし" : "検出失敗")
    : "検出OK";
  el.detectStatusCard.classList.toggle("error", data.detection_ok === false);
  el.detectStatusCard.classList.toggle("ok", data.detection_ok !== false);
  setText(el.detectStatusValue, statusText);
  setText(el.detectMessageValue, data.message || "-");
  setText(el.detectNeedleValue, formatNeedleValue(data.needle_value, Boolean(data.needle)));
  setText(el.detectMarkerValue, markerText);
  setText(
    el.detectAngleValue,
    data.needle
      ? `${formatNumber(data.needle.image_angle_deg, 1)} deg / ${formatNumber(data.needle.cartesian_angle_deg, 1)} deg`
      : "-"
  );
  setText(el.detectCircleValue, formatCircle(data.circle));
  setText(el.detectNeedleTipValue, data.needle ? formatPoint(data.needle.tip) : "-");
  setText(el.detectScaleValue, data.needle ? fraction : "-");
  setText(el.detectConfigValue, data.config_name || el.selectedConfig.value || "-");

  el.detectResult.textContent = JSON.stringify({
    detection_ok: data.detection_ok !== false,
    message: data.message || "",
    config_name: data.config_name || el.selectedConfig.value || "",
    marker_detected: data.marker_detected,
    marker_error: data.marker_error || "",
    circle: data.circle,
    needle: data.needle,
    needle_value: needleValue,
    scale_fraction: fraction,
    scale_min_point: data.scale_min_point,
    scale_max_point: data.scale_max_point,
  }, null, 2);
}

function applyConfigSettings(settings) {
  if (!el.configSettingsList || !el.configRawJson) {
    return;
  }
  el.configSettingsList.innerHTML = "";
  if (!settings) {
    el.configSettingsList.innerHTML = "<div><dt>状態</dt><dd>未読込</dd></div>";
    el.configRawJson.textContent = "-";
    return;
  }

  const rows = [
    ["JSON", settings.config_name || "-"],
    ["辞書", settings.dictionary || "-"],
    ["マーカーID", settings.marker_id ?? "-"],
    ["処理最大幅", settings.max_width ?? "-"],
    ["登録画像", settings.registration_image || "-"],
    ["登録円", formatCircle(settings.registration_circle)],
    [
      "登録処理サイズ",
      settings.registration_work_size
        ? `${settings.registration_work_size.width || "-"} x ${settings.registration_work_size.height || "-"}`
        : "-",
    ],
    ["最小値", settings.scale_min_value ?? "未登録"],
    ["最大値", settings.scale_max_value ?? "未登録"],
    ["目盛り方向", settings.scale_direction || "-"],
    ["最小目盛り点", formatPoint(settings.scale_min_point)],
    ["最大目盛り点", formatPoint(settings.scale_max_point)],
  ];

  for (const [label, value] of rows) {
    const row = document.createElement("div");
    const dt = document.createElement("dt");
    const dd = document.createElement("dd");
    dt.textContent = label;
    dd.textContent = value;
    row.appendChild(dt);
    row.appendChild(dd);
    el.configSettingsList.appendChild(row);
  }
  el.configRawJson.textContent = JSON.stringify(settings.raw || settings, null, 2);
}

function applyConfigFiles(files, selectedName) {
  if (!el.configSelect) {
    return;
  }
  const currentValue = selectedName || el.configSelect.value;
  el.configSelect.innerHTML = "";
  for (const file of files || []) {
    const option = document.createElement("option");
    option.value = file.name;
    option.textContent = file.name;
    if (file.selected || file.name === currentValue) {
      option.selected = true;
    }
    el.configSelect.appendChild(option);
  }
  el.selectedConfig.value = el.configSelect.value || currentValue || "-";
}

async function loadConfigList() {
  const response = await fetch("/api/config/list");
  const data = await response.json();
  if (!response.ok || !data.ok) {
    throw new Error(data.error || `API error: ${response.status}`);
  }
  applyConfigFiles(data.config_files, data.selected_config);
  if (data.config_settings !== undefined) {
    applyConfigSettings(data.config_settings);
  }
  return data;
}

async function selectConfig(configName) {
  const data = await postJson("/api/config/select", {config_name: configName});
  applyRegisterResponse(data);
  setStatus(data.message || "JSONを選択しました。", "ok");
}

function imagePointFromEvent(event) {
  const rect = el.registerImage.getBoundingClientRect();
  if (rect.width <= 0 || rect.height <= 0) {
    return null;
  }
  const x = Math.round((event.clientX - rect.left) * state.registerImageWidth / rect.width);
  const y = Math.round((event.clientY - rect.top) * state.registerImageHeight / rect.height);
  return {
    x: Math.max(0, Math.min(state.registerImageWidth - 1, x)),
    y: Math.max(0, Math.min(state.registerImageHeight - 1, y)),
  };
}

async function refreshPreview() {
  const data = await postJson("/api/register/preview", {settings: currentSettings()});
  applyRegisterResponse(data);
  setStatus(data.message || "プレビュー更新", "ok");
}

function setScalePointOnCircle(point) {
  const circle = currentSettings().circle;
  const dx = point.x - circle.x;
  const dy = point.y - circle.y;
  const distance = Math.hypot(dx, dy);
  if (distance <= 1e-9) {
    return {x: Math.round(circle.x + circle.radius), y: Math.round(circle.y)};
  }
  return {
    x: Math.round(circle.x + dx * circle.radius / distance),
    y: Math.round(circle.y + dy * circle.radius / distance),
  };
}

async function handleRegisterClick(event) {
  if (!el.registerImage.classList.contains("loaded")) {
    return;
  }
  const point = imagePointFromEvent(event);
  if (!point) {
    return;
  }
  if (el.clickMode.value === "scale_min") {
    state.scaleMinPoint = setScalePointOnCircle(point);
  } else if (el.clickMode.value === "scale_max") {
    state.scaleMaxPoint = setScalePointOnCircle(point);
  } else {
    el.centerX.value = point.x;
    el.centerY.value = point.y;
  }
  try {
    await refreshPreview();
  } catch (error) {
    setStatus(error.message, "error");
  }
}

async function captureRegisterImage() {
  try {
    setStatus("カメラから登録画像を取得中...");
    const data = await postJson("/api/register/capture", {settings: currentSettings()});
    applyRegisterResponse(data);
    setStatus(data.message, "ok");
  } catch (error) {
    setStatus(error.message, "error");
  }
}

async function uploadRegisterImage(file) {
  if (!file) {
    return;
  }
  const formData = new FormData();
  formData.append("image", file);
  formData.append("settings", JSON.stringify(currentSettings()));
  try {
    setStatus("アップロード画像を読み込み中...");
    const response = await fetch("/api/register/upload", {
      method: "POST",
      body: formData,
    });
    const data = await response.json();
    if (!response.ok || !data.ok) {
      throw new Error(data.error || `API error: ${response.status}`);
    }
    applyRegisterResponse(data);
    setStatus(data.message, "ok");
  } catch (error) {
    setStatus(error.message, "error");
  }
}

async function runRegisterAction(url, workingMessage) {
  try {
    setStatus(workingMessage);
    const data = await postJson(url, {settings: currentSettings()});
    applyRegisterResponse(data);
    setStatus(data.message, "ok");
  } catch (error) {
    setStatus(error.message, "error");
  }
}

async function previewNeedleDetection() {
  await runRegisterAction("/api/register/detect-needle", "登録画像で針検出中...");
}

async function detectOnce() {
  const payload = {
    settings: {
      camera_index: numberValue(el.detectCameraIndex, 0),
      draw_text: el.drawText.checked,
    },
  };
  const data = await postJson("/api/detect/latest", payload);
  applyDetectResponse(data);
  setStatus(data.message || "検出更新", data.detection_ok === false ? "error" : "ok");
}

async function detectLoop() {
  if (!state.detecting) {
    return;
  }
  try {
    await detectOnce();
  } catch (error) {
    el.detectResult.textContent = error.message;
    setStatus(error.message, "error");
  } finally {
    if (state.detecting) {
      const interval = Math.max(200, numberValue(el.detectInterval, 500));
      state.detectTimer = window.setTimeout(detectLoop, interval);
    }
  }
}

function startDetection() {
  if (state.detecting) {
    return;
  }
  state.detecting = true;
  el.startDetectButton.disabled = true;
  el.stopDetectButton.disabled = false;
  resetFps();
  setStatus("リアルタイム検出を開始しました。", "ok");
  detectLoop();
}

function stopDetection() {
  state.detecting = false;
  if (state.detectTimer) {
    window.clearTimeout(state.detectTimer);
    state.detectTimer = null;
  }
  el.startDetectButton.disabled = false;
  el.stopDetectButton.disabled = true;
  resetFps();
  setStatus("リアルタイム検出を停止しました。");
}

function setupTabs() {
  for (const button of document.querySelectorAll(".tab-button")) {
    button.addEventListener("click", () => {
      document.querySelectorAll(".tab-button").forEach((item) => item.classList.remove("active"));
      document.querySelectorAll(".tab-panel").forEach((item) => item.classList.remove("active"));
      button.classList.add("active");
      document.getElementById(`tab-${button.dataset.tab}`).classList.add("active");
    });
  }
}

function setupEvents() {
  el.captureRegisterButton.addEventListener("click", captureRegisterImage);
  el.uploadRegisterInput.addEventListener("change", () => uploadRegisterImage(el.uploadRegisterInput.files[0]));
  el.detectMarkerButton.addEventListener("click", () => runRegisterAction("/api/register/marker", "ARマーカー検出中..."));
  el.autoCircleButton.addEventListener("click", () => runRegisterAction("/api/register/auto-circle", "メーター円を自動検出中..."));
  el.previewNeedleButton.addEventListener("click", previewNeedleDetection);
  el.autoScaleButton.addEventListener("click", () => runRegisterAction("/api/register/auto-scale", "目盛りを自動設定中..."));
  el.saveRegistrationButton.addEventListener("click", () => runRegisterAction("/api/register/save", "登録保存中..."));
  el.clearScaleButton.addEventListener("click", async () => {
    state.scaleMinPoint = null;
    state.scaleMaxPoint = null;
    await refreshPreview();
  });
  el.registerImage.addEventListener("click", handleRegisterClick);
  el.startDetectButton.addEventListener("click", startDetection);
  el.stopDetectButton.addEventListener("click", stopDetection);
  el.reloadConfigButton.addEventListener("click", async () => {
    try {
      await loadConfigList();
      setStatus("JSON一覧を更新しました。", "ok");
    } catch (error) {
      setStatus(error.message, "error");
    }
  });
  el.configSelect.addEventListener("change", async () => {
    try {
      await selectConfig(el.configSelect.value);
    } catch (error) {
      setStatus(error.message, "error");
    }
  });

  for (const input of [el.centerX, el.centerY, el.radius, el.scaleMinValue, el.scaleMaxValue, el.scaleDirection, el.drawText]) {
    input.addEventListener("change", () => refreshPreview().catch((error) => setStatus(error.message, "error")));
  }
  el.maxWidth.addEventListener("change", () => runRegisterAction("/api/register/reprocess", "処理画像サイズを更新中..."));
  el.detectCameraIndex.value = el.cameraIndex.value;
  el.cameraIndex.addEventListener("change", () => {
    el.detectCameraIndex.value = el.cameraIndex.value;
  });
}

async function initialize() {
  setupTabs();
  setupEvents();
  try {
    await loadConfigList();
    const response = await fetch("/api/register/state");
    const data = await response.json();
    if (!response.ok || !data.ok) {
      throw new Error(data.error || `API error: ${response.status}`);
    }
    applyRegisterResponse(data);
    setStatus("準備完了", "ok");
  } catch (error) {
    setStatus(error.message, "error");
  }
}

initialize();
