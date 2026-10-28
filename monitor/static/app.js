"use strict";

const $ = (id) => document.getElementById(id);
const controls = document.querySelectorAll("[data-control], [data-command]");
let state = null;
let streamOnline = false;
let busy = false;
const samples = [];
const seenSamples = new Set();

function showError(message = "") {
  $("error").textContent = message.toLowerCase();
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
      4,
      0,
      Math.PI * 2,
    );
    context.fill();
  }
  const values = valid ? `x ${x}  y ${y}` : "no data";
  context.fillText(values, 12, 158);
  canvas.setAttribute("aria-label", `${label}: ${values}`);
}

function drawJoysticks() {
  const fresh =
    streamOnline &&
    state?.connected &&
    Date.now() - state.lastReceivedAt < 1500;
  const packet = fresh ? state.telemetry || {} : {};
  drawJoystick("joystick1", "j1", packet.j1x, packet.j1y);
  drawJoystick("joystick2", "j2", packet.j2x, packet.j2y);
}

function renderControls() {
  $("connect").disabled = busy || !streamOnline;
  $("export").disabled = samples.length === 0;
  for (const element of controls) {
    element.disabled = busy || !streamOnline || !state?.connected;
  }
}

function acceptState(update) {
  state = update;
  if (update.telemetry) {
    const key = JSON.stringify([
      update.port,
      update.packetCount,
      update.lastReceivedAt,
    ]);
    if (!seenSamples.has(key)) {
      seenSamples.add(key);
      samples.push(update.telemetry);
    }
  }
  $("packet").textContent = update.telemetry
    ? JSON.stringify(update.telemetry, null, 2)
    : "no telemetry received.";
  showError(update.error || "");
  renderControls();
  drawJoysticks();
}

function toCsv(packets) {
  if (!packets.length) return "";
  const columns = [
    ...new Set(packets.flatMap((packet) => Object.keys(packet))),
  ];
  const cell = (value) => {
    if (value === null || value === undefined) return "";
    const text =
      typeof value === "object" ? JSON.stringify(value) : String(value);
    return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
  };
  return (
    [
      columns.map(cell).join(","),
      ...packets.map((packet) =>
        columns.map((column) => cell(packet[column])).join(","),
      ),
    ].join("\r\n") + "\r\n"
  );
}

async function request(path, body = {}) {
  if (busy) return;
  busy = true;
  showError();
  $("command-result").hidden = true;
  renderControls();
  try {
    const response = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "request failed");
    if (path === "/api/connect") {
      acceptState(data);
    } else {
      $("command-result").textContent = `tx accepted: ${data.command}`;
      $("command-result").hidden = false;
    }
  } catch (error) {
    showError(error.message);
  } finally {
    busy = false;
    renderControls();
  }
}

$("connect").addEventListener("click", () => request("/api/connect"));

for (const command of ["servo", "motor", "limit"]) {
  $(`${command}-form`).addEventListener("submit", (event) => {
    event.preventDefault();
    request("/api/command", { command: `${command} ${$(command).value}` });
  });
}
for (const button of document.querySelectorAll("[data-command]")) {
  button.addEventListener("click", () =>
    request("/api/command", { command: button.dataset.command }),
  );
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

const events = new EventSource("/api/events");
events.onopen = () => {
  streamOnline = true;
  showError(state?.error || "");
  renderControls();
};
events.onerror = () => {
  streamOnline = false;
  showError("monitor server unavailable. reconnecting…");
  renderControls();
  drawJoysticks();
};
events.addEventListener("update", (event) => {
  streamOnline = true;
  acceptState(JSON.parse(event.data));
});
renderControls();
drawJoysticks();
setInterval(drawJoysticks, 250);
