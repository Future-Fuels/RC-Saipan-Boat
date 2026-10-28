"use strict";

const $ = (id) => document.getElementById(id);
const samples = [];
let state = null;
let streamOnline = false;
let polling = false;
let commanding = false;
let lastRecordedPacket = 0;

// Little-endian ESP32 structs from shared.h: 16-byte header, 40-byte control,
// 136-byte sensor packet, then 40 bytes of settings echoed by RX.
function readControl(view, offset) {
  const control = {};
  for (const name of [
    "j1x", "j1y", "j2x", "j2y", "speed",
    "servoLock", "motorLock", "speedLimit", "stopped",
  ]) {
    control[name] = view.getInt32(offset, true);
    offset += 4;
  }
  control.gpsResetId = view.getUint32(offset, true);
  return control;
}

function decodePacket(buffer) {
  const view = new DataView(buffer);
  if (buffer.byteLength !== 232 || view.getUint32(0, true) !== 1) {
    throw new Error("Monitor version mismatch. Reload after flashing both boards.");
  }

  const uptimeMs = view.getUint32(4, true);
  const packetCount = view.getUint32(8, true);
  const telemetryAgeMs = view.getUint32(12, true);
  const control = readControl(view, 16);
  const applied = readControl(view, 192);
  const sensor = {};
  let offset = 56;

  for (const name of [
    "accX", "accY", "accZ", "gyroX", "gyroY", "gyroZ", "resG",
    "linearAccX", "linearAccY", "linearAccZ", "quatW", "quatX", "quatY", "quatZ",
  ]) {
    sensor[name] = view.getFloat32(offset, true);
    offset += 4;
  }
  for (const name of ["latitude", "longitude"]) {
    sensor[name] = view.getFloat64(offset, true);
    offset += 8;
  }
  for (const name of [
    "altitude", "gpsSpeed", "gpsCourse", "east", "north",
    "eastVelocity", "northVelocity",
  ]) {
    sensor[name] = view.getFloat32(offset, true);
    offset += 4;
  }
  for (const name of [
    "gpsAgeMs", "satellites", "motorSpeed", "servoAngle", "controlPackets", "controlAgeMs",
  ]) {
    sensor[name] = name === "motorSpeed"
      ? view.getInt32(offset, true)
      : view.getUint32(offset, true);
    offset += 4;
  }
  for (const name of [
    "active", "imuAvailable", "gpsFix", "gpsSpeedValid", "gpsCourseValid",
    "gpsAltitudeValid", "filterInitialized",
  ]) {
    sensor[name] = Boolean(view.getUint8(offset++));
  }
  for (const name of [
    "systemCalibration", "gyroCalibration", "accelCalibration", "magCalibration",
  ]) {
    sensor[name] = view.getUint8(offset++);
  }

  const telemetryFresh = packetCount > 0 && telemetryAgeMs < 500;
  const connected = telemetryFresh && sensor.controlAgeMs < 500;
  return {
    connected,
    uptimeMs,
    packetCount,
    lastReceivedAt: Date.now(),
    telemetry: {
      j1x: control.j1x,
      j1y: control.j1y,
      j2x: control.j2x,
      j2y: control.j2y,
      speed: control.speed,
      ...sensor,
      servoLocked: applied.servoLock !== -1,
      motorLocked: applied.motorLock !== -256,
      servoLock: applied.servoLock,
      motorLock: applied.motorLock,
      speedLimit: applied.speedLimit,
      stopped: Boolean(applied.stopped),
      gpsResetId: applied.gpsResetId,
      connected,
      telemetryFresh,
      telemetryAgeMs,
    },
  };
}

async function api(path, command) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 2000);
  try {
    const response = await fetch(path, {
      cache: "no-store",
      signal: controller.signal,
      ...(command === undefined ? {} : {
        method: "POST",
        headers: { "Content-Type": "text/plain" },
        body: command,
      }),
    });
    if (!response.ok) throw new Error(await response.text());
    return command === undefined
      ? await response.arrayBuffer()
      : await response.text();
  } finally {
    clearTimeout(timeout);
  }
}

function showError(message = "") {
  $("error").textContent = message;
  $("error").hidden = !message;
}

function drawJoystick(id, label, x, y) {
  const canvas = $(id);
  const context = canvas.getContext("2d");
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.font = "12px monospace";
  context.fillText(label, 12, 14);
  context.strokeStyle = "#aaa";
  context.strokeRect(12.5, 24.5, 120, 120);
  context.beginPath();
  context.moveTo(72.5, 24.5);
  context.lineTo(72.5, 144.5);
  context.moveTo(12.5, 84.5);
  context.lineTo(132.5, 84.5);
  context.stroke();
  const valid = Number.isFinite(x) && Number.isFinite(y);
  if (valid) {
    const clamp = (value) => Math.max(-255, Math.min(255, value));
    context.beginPath();
    context.arc(
      72.5 + (clamp(x) / 255) * 55,
      84.5 - (clamp(y) / 255) * 55,
      4, 0, Math.PI * 2,
    );
    context.fill();
  }
  const values = valid ? `X ${x}  Y ${y}` : "No data";
  context.fillText(values, 12, 158);
  canvas.setAttribute("aria-label", `${label}: ${values}`);
}

function drawJoysticks() {
  const fresh = streamOnline && state && Date.now() - state.lastReceivedAt < 1500;
  const packet = fresh ? state.telemetry : {};
  drawJoystick("joystick1", "J1 · Motor", packet.j1x, packet.j1y);
  drawJoystick("joystick2", "J2 · Servo", packet.j2x, packet.j2y);
}

function renderControls() {
  $("export").disabled = samples.length === 0;
  $("connection").textContent = streamOnline
    ? `${state.connected ? "RX connected" : "TX online · Waiting for RX"} · ${samples.length} samples recorded`
    : "TX unavailable · Retrying…";
  for (const element of document.querySelectorAll("[data-control], [data-command]")) {
    element.disabled = !streamOnline || !state?.connected || commanding;
  }
}

function acceptState(update) {
  if (state && update.uptimeMs < state.uptimeMs) lastRecordedPacket = 0;
  state = update;
  if (update.telemetry.telemetryFresh && update.packetCount !== lastRecordedPacket) {
    samples.push({ recordedAt: new Date().toISOString(), ...update.telemetry });
    lastRecordedPacket = update.packetCount;
  }
  $("packet").textContent = JSON.stringify(update.telemetry, null, 2);
}

function toCsv(packets) {
  if (!packets.length) return "";
  const columns = [...new Set(packets.flatMap((packet) => Object.keys(packet)))];
  const cell = (value) => {
    if (value === null || value === undefined) return "";
    const text = typeof value === "object" ? JSON.stringify(value) : String(value);
    return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
  };
  return [
    columns.map(cell).join(","),
    ...packets.map((packet) => columns.map((column) => cell(packet[column])).join(",")),
  ].join("\r\n") + "\r\n";
}

async function poll() {
  if (polling) return;
  polling = true;
  try {
    acceptState(decodePacket(await api("/api/telemetry")));
    streamOnline = true;
    showError();
  } catch (error) {
    streamOnline = false;
    showError(`Can't read TX: ${error.message}`);
  } finally {
    polling = false;
    renderControls();
    drawJoysticks();
    setTimeout(poll, streamOnline ? 50 : 1000);
  }
}

async function sendCommand(command) {
  if (commanding) return;
  commanding = true;
  $("command-result").hidden = true;
  renderControls();
  try {
    await api("/api/command", command);
  } catch (error) {
    $("command-result").textContent = `Command failed: ${error.message}`;
    $("command-result").hidden = false;
  } finally {
    commanding = false;
    renderControls();
  }
}

for (const command of ["servo", "motor", "limit"]) {
  $(`${command}-form`).addEventListener("submit", (event) => {
    event.preventDefault();
    sendCommand(`${command} ${$(command).value}`);
  });
}
for (const button of document.querySelectorAll("[data-command]")) {
  button.addEventListener("click", () => sendCommand(button.dataset.command));
}
$("export").addEventListener("click", () => {
  const url = URL.createObjectURL(
    new Blob([toCsv(samples)], { type: "text/csv;charset=utf-8" }),
  );
  const link = document.createElement("a");
  link.href = url;
  link.download = `rcboat-${new Date().toISOString().replaceAll(":", "-")}.csv`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

drawJoysticks();
poll();
