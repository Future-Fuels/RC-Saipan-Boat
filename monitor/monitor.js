import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const WS_URL = 'ws://localhost:8765';
const HISTORY_LENGTH = 100;
const JOY_MIN = -255;
const JOY_MAX = 255;
const GRAPH_COLORS = ['#3e3e3e', '#ff3333', '#33cc33', '#3333ff', '#ff9900'];

const els = {
  connectBtn: document.getElementById('connectBtn'),
  calibrateBtn: document.getElementById('calibrateBtn'),
  calibrateStatus: document.getElementById('calibrateStatus'),
  recordBtn: document.getElementById('recordBtn'),
  captureBtn: document.getElementById('captureBtn'),
  exportBtn: document.getElementById('exportBtn'),
  sampleRate: document.getElementById('sampleRate'),
  captureDuration: document.getElementById('captureDuration'),
  recordStatus: document.getElementById('recordStatus'),
  status: document.getElementById('status'),
  pktInfo: document.getElementById('pktInfo'),
  controls: document.getElementById('controls'),
  resG: document.getElementById('resG'),
  attitude: document.getElementById('attitude'),
  tiltPeak: document.getElementById('tiltPeak'),
  gyroVals: document.getElementById('gyroVals'),
  accGraph: document.getElementById('accGraph'),
  gyroGraph: document.getElementById('gyroGraph'),
  tiltGraph: document.getElementById('tiltGraph'),
  boatContainer: document.getElementById('boatContainer'),
};

const accHist = { x: [], y: [], z: [] };
const gyroHist = { x: [], y: [], z: [] };
const tiltHist = { pitch: [], roll: [] };

const recordedSamples = [];

let peakPitch = 0;
let peakRoll = 0;
let ws = null;
let isRecording = false;
let lastSampleTime = 0;
let captureTimer = null;
let countdownTimer = null;

let boatGroup = null;

function fmt(value, digits = 1) {
  return Number(value).toFixed(digits);
}

function pushHistory(series, value) {
  series.push(value);
  if (series.length > HISTORY_LENGTH) {
    series.shift();
  }
}

function pitchRollFromAccel(ax, ay, az) {
  const roll = (Math.atan2(ay, az) * 180) / Math.PI;
  const pitch = (Math.atan2(-ax, Math.hypot(ay, az)) * 180) / Math.PI;
  return { pitch, roll };
}

function padGyro(axis, value) {
  const formatted = fmt(value, 1);
  const sign = formatted.startsWith('-') ? '' : ' ';
  return `${axis}: ${sign}${formatted.padStart(7, ' ')} °/s`;
}

function drawJoystick(canvasId, x, y) {
  const canvas = document.getElementById(canvasId);
  const ctx = canvas.getContext('2d');
  const { width: w, height: h } = canvas;
  const cx = w / 2;
  const cy = h / 2;
  const pad = 12;
  const inner = Math.min(w, h) - pad * 2;

  ctx.clearRect(0, 0, w, h);
  ctx.strokeRect(pad, pad, inner, inner);
  ctx.beginPath();
  ctx.moveTo(cx, pad);
  ctx.lineTo(cx, h - pad);
  ctx.moveTo(pad, cy);
  ctx.lineTo(w - pad, cy);
  ctx.stroke();

  const nx = (x - JOY_MIN) / (JOY_MAX - JOY_MIN);
  const ny = (y - JOY_MIN) / (JOY_MAX - JOY_MIN);
  const px = pad + nx * inner;
  const py = pad + (1 - ny) * inner;

  ctx.beginPath();
  ctx.arc(px, py, 8, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillText(`${canvasId}(${x}, ${y})`, pad, h - 3);
}

function drawGraph(canvas, series, yMin, yMax) {
  const ctx = canvas.getContext('2d');
  const { width: w, height: h } = canvas;
  const keys = Object.keys(series);
  const len = Math.max(...keys.map((key) => series[key].length));

  if (len < 2) {
    return;
  }

  ctx.clearRect(0, 0, w, h);
  const range = yMax - yMin || 1;

  ctx.beginPath();
  ctx.moveTo(0, h - ((0 - yMin) / range) * h);
  ctx.lineTo(w, h - ((0 - yMin) / range) * h);
  ctx.stroke();

  keys.forEach((key, index) => {
    const values = series[key];
    ctx.strokeStyle = GRAPH_COLORS[index % GRAPH_COLORS.length];
    ctx.beginPath();

    for (let i = 0; i < values.length; i += 1) {
      const x = (i / (HISTORY_LENGTH - 1)) * w;
      const y = h - ((values[i] - yMin) / range) * h;
      if (i === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    }

    ctx.stroke();
  });
}

function render(packet) {
  els.controls.textContent =
    `button: ${packet.button}\n` +
    `speed: ${packet.speed} / 255\n` +
    `calibrate: ${packet.calibrate}\n` +
    `bt: ${packet.connected ? 'connected' : 'waiting'}`;

  drawJoystick('j1', packet.j1x, packet.j1y);
  drawJoystick('j2', packet.j2x, packet.j2y);

  pushHistory(accHist.x, packet.accX);
  pushHistory(accHist.y, packet.accY);
  pushHistory(accHist.z, packet.accZ);
  pushHistory(gyroHist.x, packet.gyroX);
  pushHistory(gyroHist.y, packet.gyroY);
  pushHistory(gyroHist.z, packet.gyroZ);

  drawGraph(els.accGraph, accHist, -2, 2);
  drawGraph(els.gyroGraph, gyroHist, -250, 250);

  els.resG.textContent = `resultant g: ${fmt(packet.resG, 3)}`;

  const tilt = pitchRollFromAccel(packet.accX, packet.accY, packet.accZ);
  pushHistory(tiltHist.pitch, tilt.pitch);
  pushHistory(tiltHist.roll, tilt.roll);

  peakPitch = Math.max(peakPitch, Math.abs(tilt.pitch));
  peakRoll = Math.max(peakRoll, Math.abs(tilt.roll));

  drawGraph(els.tiltGraph, tiltHist, -45, 45);

  els.attitude.textContent =
    `pitch: ${fmt(tilt.pitch, 1)} °\nroll: ${fmt(tilt.roll, 1)} °`;
  els.tiltPeak.textContent =
    `peak pitch: ${fmt(peakPitch, 1)} °\npeak roll: ${fmt(peakRoll, 1)} °`;
  els.gyroVals.textContent =
    `${padGyro('x', packet.gyroX)}\n` +
    `${padGyro('y', packet.gyroY)}\n` +
    `${padGyro('z', packet.gyroZ)}`;

  if (packet.calibrate) {
    els.calibrateStatus.textContent = 'active';
  } else if (els.calibrateStatus.textContent === 'active') {
    els.calibrateStatus.textContent = 'done';
  }

  if (boatGroup) {
    boatGroup.rotation.x = tilt.pitch * (Math.PI / 180);
    boatGroup.rotation.z = tilt.roll * (Math.PI / 180);
  }
}

function initBoatScene() {
  const { clientWidth: width, clientHeight: height } = els.boatContainer;

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 1000);
  camera.position.set(5, 4, 6);
  camera.lookAt(0, 0, 0);

  const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
  renderer.setSize(width, height);
  els.boatContainer.appendChild(renderer.domElement);

  scene.add(new THREE.AmbientLight(0xffffff, 0.8));

  const dirLight = new THREE.DirectionalLight(0xffffff, 1);
  dirLight.position.set(10, 20, 10);
  scene.add(dirLight);

  new GLTFLoader().load(
    'boat.gltf',
    (gltf) => {
      boatGroup = gltf.scene;
      boatGroup.scale.set(10, 10, 10);
      scene.add(boatGroup);
    },
    undefined,
    (error) => {
      console.error('failed to load boat model:', error);
    },
  );

  function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
  }

  animate();
}

function sampleRateMs() {
  const rate = Number.parseInt(els.sampleRate.value, 10) || 10;
  return 1000 / rate;
}

function captureDurationMs() {
  const seconds = Number.parseInt(els.captureDuration.value, 10) || 10;
  return Math.min(300, Math.max(1, seconds)) * 1000;
}

function maybeRecordSample(packet) {
  if (!isRecording) {
    return;
  }

  const now = Date.now();
  if (now - lastSampleTime < sampleRateMs()) {
    return;
  }

  recordedSamples.push({ ...packet, timestamp: now });
  lastSampleTime = now;
}

function clearCaptureTimers() {
  if (captureTimer !== null) {
    clearTimeout(captureTimer);
    captureTimer = null;
  }
  if (countdownTimer !== null) {
    clearInterval(countdownTimer);
    countdownTimer = null;
  }
  els.captureBtn.disabled = false;
  els.captureDuration.disabled = false;
}

function startRecording() {
  isRecording = true;
  recordedSamples.length = 0;
  lastSampleTime = 0;
  els.recordBtn.textContent = 'stop';
  els.recordStatus.textContent = 'recording';
}

function stopRecording() {
  isRecording = false;
  clearCaptureTimers();
  els.recordBtn.textContent = 'record';
  els.recordStatus.textContent = '';
}

function toggleRecording() {
  if (isRecording) {
    stopRecording();
    return;
  }

  startRecording();
}

function updateCaptureCountdown(endsAt) {
  const remaining = Math.max(0, Math.ceil((endsAt - Date.now()) / 1000));
  els.recordStatus.textContent = `capturing… ${remaining}s`;
}

function finishTimedCapture() {
  clearCaptureTimers();
  isRecording = false;
  els.recordBtn.textContent = 'record';

  const count = recordedSamples.length;
  if (count === 0) {
    els.recordStatus.textContent = 'no data captured';
    return;
  }

  exportCsv();
  els.recordStatus.textContent = `exported ${count} samples`;
}

function startTimedCapture() {
  if (ws?.readyState !== WebSocket.OPEN) {
    alert('connect first');
    return;
  }

  if (isRecording) {
    stopRecording();
  }

  startRecording();
  els.captureBtn.disabled = true;
  els.captureDuration.disabled = true;

  const durationMs = captureDurationMs();
  const endsAt = Date.now() + durationMs;
  updateCaptureCountdown(endsAt);
  countdownTimer = setInterval(() => updateCaptureCountdown(endsAt), 200);

  captureTimer = setTimeout(finishTimedCapture, durationMs);
}

function exportCsv() {
  if (recordedSamples.length === 0) {
    alert('no data');
    return;
  }

  const headers = Object.keys(recordedSamples[0]);
  const rows = [
    headers.join(','),
    ...recordedSamples.map((sample) =>
      headers
        .map((header) => {
          const value = sample[header] ?? '';
          return `"${value}"`;
        })
        .join(','),
    ),
  ];

  const blob = new Blob([rows.join('\n')], { type: 'text/csv' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = `rcboat_data_${Date.now()}.csv`;
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function connect() {
  if (ws) {
    ws.close();
    ws = null;
  }

  ws = new WebSocket(WS_URL);
  els.status.textContent = 'connecting...';

  ws.onopen = () => {
    els.status.textContent = 'connected';
    els.connectBtn.textContent = 'disconnect';
  };

  ws.onmessage = (event) => {
    els.pktInfo.textContent = `${event.data.length} chars`;

    try {
      const packet = JSON.parse(event.data);
      maybeRecordSample(packet);
      render(packet);
    } catch (error) {
      els.status.textContent = 'parse error';
      console.error(error, event.data);
    }
  };

  ws.onclose = () => {
    els.status.textContent = 'disconnected';
    els.connectBtn.textContent = 'connect';
    ws = null;

    if (isRecording) {
      stopRecording();
    } else {
      clearCaptureTimers();
    }
  };

  ws.onerror = () => {
    els.status.textContent = 'error';
  };
}

function toggleConnection() {
  if (ws?.readyState === WebSocket.OPEN) {
    ws.close();
    return;
  }

  connect();
}

function sendCalibrate() {
  if (ws?.readyState !== WebSocket.OPEN) {
    alert('connect first');
    return;
  }

  ws.send(JSON.stringify({ cmd: 'calibrate' }));
  els.calibrateStatus.textContent = 'sent';
}

function init() {
  initBoatScene();

  els.connectBtn.addEventListener('click', toggleConnection);
  els.calibrateBtn.addEventListener('click', sendCalibrate);
  els.recordBtn.addEventListener('click', toggleRecording);
  els.captureBtn.addEventListener('click', startTimedCapture);
  els.exportBtn.addEventListener('click', exportCsv);
}

init();
