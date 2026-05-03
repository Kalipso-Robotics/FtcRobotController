#!/usr/bin/env python3
"""Real-time ADB Logcat Grapher for FTC Robot Telemetry.

Streams logs from `adb logcat`, parses key-value pairs from tagged log lines,
and graphs multiple data series in real time in your browser.
Includes a Field View mode for visualizing robot position on a 12ft x 12ft field.

Usage:
    python3 logcat_grapher.py

No external dependencies required — uses only Python standard library.
Opens http://localhost:8765 in your default browser.
"""

import json
import os
import re
import subprocess
import sys
import threading
import time
import webbrowser
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from queue import Queue, Empty

# ── State ────────────────────────────────────────────────────────────

series_lock = threading.Lock()
series_map = {}  # id -> {tag, key, color, pattern}
series_counter = 0
data_queues = []  # list of Queue for SSE clients
logcat_process = None
streaming = False
streaming_lock = threading.Lock()

# Position source state
position_lock = threading.Lock()
position_source = None  # {tag, x_pat, y_pat, theta_pat}

LOG_RE = re.compile(
    r'^(\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}\.\d{3})\s+\d+\s+\d+\s+[VDIWEF]\s+(\S+?)\s*:\s*(.*)')

POS_RE_X = re.compile(r'x\s*[:=]\s*(-?[\d.]+(?:[eE][+-]?\d+)?)')
POS_RE_Y = re.compile(r'(?<![a-zA-Z])y\s*[:=]\s*(-?[\d.]+(?:[eE][+-]?\d+)?)')
POS_RE_THETA = re.compile(r'theta\s*[:=]\s*(-?[\d.]+(?:[eE][+-]?\d+)?)')


def clean_key(key):
    """Strip trailing delimiters/spaces from key input."""
    return key.strip().rstrip(':').rstrip('=').rstrip()


def make_pattern(key):
    cleaned = clean_key(key)
    return re.compile(re.escape(cleaned) + r'\s*[:=]\s*(-?[\d.]+(?:[eE][+-]?\d+)?)')


# ── Logcat Streaming ─────────────────────────────────────────────────

def start_logcat():
    global logcat_process, streaming
    with streaming_lock:
        if streaming:
            return False
        try:
            logcat_process = subprocess.Popen(
                ["adb", "logcat", "-v", "threadtime"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1)
        except FileNotFoundError:
            return False
        streaming = True

    thread = threading.Thread(target=_read_logcat, daemon=True)
    thread.start()
    return True


def stop_logcat():
    global logcat_process, streaming
    with streaming_lock:
        streaming = False
        if logcat_process:
            try:
                logcat_process.terminate()
            except Exception:
                pass
            logcat_process = None


def _read_logcat():
    global streaming
    proc = logcat_process
    if not proc or not proc.stdout:
        return
    try:
        for line in proc.stdout:
            if not streaming:
                break
            _process_line(line.rstrip('\n'))
    except Exception:
        pass
    with streaming_lock:
        streaming = False


def _parse_timestamp(timestamp_str):
    try:
        now = datetime.now()
        ts = datetime.strptime(timestamp_str, "%m-%d %H:%M:%S.%f")
        ts = ts.replace(year=now.year)
        return ts.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
    except ValueError:
        return None


def _process_line(line):
    m = LOG_RE.match(line)
    if not m:
        return

    timestamp_str, tag, message = m.group(1), m.group(2), m.group(3)

    # Check regular series
    with series_lock:
        items = list(series_map.items())

    for sid, s in items:
        if tag != s['tag']:
            continue
        vm = s['pattern'].search(message)
        if not vm:
            continue
        try:
            value = float(vm.group(1))
        except ValueError:
            continue
        ts_iso = _parse_timestamp(timestamp_str)
        if not ts_iso:
            continue

        event = json.dumps({
            "type": "data",
            "series_id": sid,
            "time": ts_iso,
            "value": value,
            "log": line
        })
        _broadcast(event)

    # Check position source
    with position_lock:
        pos_src = position_source

    if pos_src and tag == pos_src['tag']:
        xm = POS_RE_X.search(message)
        ym = POS_RE_Y.search(message)
        tm = POS_RE_THETA.search(message)
        if xm and ym and tm:
            try:
                x = float(xm.group(1))
                y = float(ym.group(1))
                theta = float(tm.group(1))
            except ValueError:
                return
            ts_iso = _parse_timestamp(timestamp_str)
            if not ts_iso:
                return
            event = json.dumps({
                "type": "position",
                "time": ts_iso,
                "x": x,
                "y": y,
                "theta": theta,
                "log": line
            })
            _broadcast(event)


def _broadcast(event_data):
    for q in list(data_queues):
        try:
            q.put_nowait(event_data)
        except Exception:
            pass


# ── HTML UI ──────────────────────────────────────────────────────────

HTML_PAGE = r'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Logcat Grapher</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.7/dist/chart.umd.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns@3.0.0/dist/chartjs-adapter-date-fns.bundle.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/hammerjs@2.0.8/hammer.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/chartjs-plugin-zoom@2.0.1/dist/chartjs-plugin-zoom.min.js"></script>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
       background: #1a1a2e; color: #e0e0e0; height: 100vh; display: flex; flex-direction: column; }

.toolbar { background: #16213e; padding: 10px 16px; display: flex; align-items: center;
           gap: 12px; flex-wrap: wrap; border-bottom: 1px solid #0f3460; }
.toolbar label { font-size: 13px; color: #a0a0c0; }
.toolbar input[type="text"], .toolbar input[type="number"] {
  background: #1a1a2e; border: 1px solid #0f3460; color: #e0e0e0;
  padding: 6px 10px; border-radius: 4px; font-size: 13px; }
.toolbar input[type="number"] { width: 60px; }
.toolbar input[type="color"] { width: 36px; height: 30px; border: none; cursor: pointer;
                                background: none; border-radius: 4px; }
.btn { padding: 6px 16px; border: none; border-radius: 4px; cursor: pointer; font-size: 13px;
       font-weight: 600; transition: opacity 0.15s; }
.btn:hover { opacity: 0.85; }
.btn-add { background: #0f3460; color: #e0e0e0; }
.btn-start { background: #4CAF50; color: white; }
.btn-stop { background: #f44336; color: white; }
.btn-clear { background: #555; color: white; }
.btn-small { padding: 4px 10px; font-size: 11px; }

.series-bar { background: #16213e; padding: 6px 16px; display: flex; gap: 12px;
              flex-wrap: wrap; align-items: center; min-height: 36px;
              border-bottom: 1px solid #0f3460; }
.series-chip { display: inline-flex; align-items: center; gap: 6px; background: #1a1a2e;
               padding: 4px 10px; border-radius: 14px; font-size: 12px; }
.series-chip .swatch { width: 12px; height: 12px; border-radius: 50%; display: inline-block; }
.series-chip .remove { cursor: pointer; color: #f44336; font-weight: bold; margin-left: 4px; }
.series-chip .remove:hover { color: #ff6666; }

.main { flex: 1; display: flex; overflow: hidden; }
.log-panel { width: 50%; display: flex; flex-direction: column; border-right: 1px solid #0f3460; }
.log-header { padding: 8px 12px; background: #16213e; font-size: 13px; font-weight: 600;
              display: flex; justify-content: space-between; align-items: center; }
.log-header label { font-size: 12px; color: #a0a0c0; cursor: pointer; }
.log-content { flex: 1; overflow-y: auto; padding: 8px; font-family: 'Courier New', monospace;
               font-size: 11px; line-height: 1.5; background: #0d0d1a; }
.log-line { white-space: pre; }

.graph-panel { width: 50%; display: flex; flex-direction: column; position: relative; }
.graph-header { padding: 8px 12px; background: #16213e; font-size: 13px; font-weight: 600;
                display: flex; justify-content: space-between; align-items: center; gap: 10px; }
.mode-tabs { display: flex; gap: 2px; }
.mode-tab { padding: 4px 12px; border-radius: 4px; cursor: pointer; font-size: 12px;
            background: #1a1a2e; color: #888; border: 1px solid #0f3460; transition: all 0.15s; }
.mode-tab.active { background: #0f3460; color: #e0e0e0; }
.mode-tab:hover { color: #ccc; }

.graph-container { flex: 1; padding: 8px; position: relative; }
.graph-container canvas { width: 100% !important; height: 100% !important; }

.field-container { flex: 1; display: flex; flex-direction: column; display: none; }
.field-settings { padding: 8px 12px; background: #16213e; display: flex; align-items: center;
                  gap: 12px; flex-wrap: wrap; border-bottom: 1px solid #0f3460; font-size: 12px; }
.field-settings label { color: #a0a0c0; }
.field-settings input[type="text"], .field-settings input[type="number"] {
  background: #1a1a2e; border: 1px solid #0f3460; color: #e0e0e0;
  padding: 4px 8px; border-radius: 4px; font-size: 12px; }
.field-settings input[type="number"] { width: 55px; }
.field-settings input[type="color"] { width: 28px; height: 24px; border: none; cursor: pointer; background: none; }
.field-canvas-wrap { flex: 1; padding: 8px; display: flex; align-items: center; justify-content: center; position: relative; }
.field-canvas-wrap canvas { background: #111; }
.field-readout { position: absolute; bottom: 16px; left: 50%; transform: translateX(-50%);
                 background: rgba(22,33,62,0.9); border: 1px solid #0f3460; border-radius: 6px;
                 padding: 6px 14px; font-size: 13px; font-family: 'Courier New', monospace; }

.crosshair-tooltip { position: absolute; background: rgba(22, 33, 62, 0.95);
                     border: 1px solid #0f3460; border-radius: 6px; padding: 8px 12px;
                     font-size: 12px; pointer-events: none; z-index: 10; display: none; }
.crosshair-tooltip .tt-row { display: flex; align-items: center; gap: 6px; padding: 2px 0; }
.crosshair-tooltip .tt-swatch { width: 10px; height: 10px; border-radius: 50%; }

.no-series { color: #555; font-style: italic; font-size: 12px; }
.sep { width: 1px; height: 20px; background: #0f3460; }
</style>
</head>
<body>

<div class="toolbar">
  <label>Tag:</label>
  <input type="text" id="tagInput" placeholder="KLog_PushAllBalls" style="width:200px">
  <label>Key:</label>
  <input type="text" id="keyInput" placeholder="x" style="width:150px">
  <label>Color:</label>
  <input type="color" id="colorInput" value="#ff4444">
  <button class="btn btn-add" onclick="addSeries()">Add Series</button>
  <div class="sep"></div>
  <label>Window:</label>
  <input type="number" id="timeWindow" value="0" min="0" step="5" title="Rolling time window in seconds (0 = show all)">
  <label style="font-size:11px;color:#666">s</label>
  <div style="flex:1"></div>
  <button class="btn btn-small btn-add" onclick="resetZoom()" title="Double-click graph also resets">Reset Zoom</button>
  <button class="btn btn-clear" onclick="clearData()">Clear</button>
  <button class="btn btn-start" id="streamBtn" onclick="toggleStream()">Start</button>
</div>

<div class="series-bar" id="seriesBar">
  <span class="no-series">No series added</span>
</div>

<div class="main">
  <div class="log-panel">
    <div class="log-header">
      <span>Matching Logs</span>
      <label><input type="checkbox" id="autoScroll" checked> Auto-scroll</label>
    </div>
    <div class="log-content" id="logContent"></div>
  </div>
  <div class="graph-panel">
    <div class="graph-header">
      <div class="mode-tabs">
        <div class="mode-tab active" onclick="setMode('timeseries')">Time Series</div>
        <div class="mode-tab" onclick="setMode('field')">Field View</div>
      </div>
      <span id="graphTitle" style="color:#888;font-size:11px"></span>
    </div>
    <!-- Time Series view -->
    <div class="graph-container" id="graphContainer">
      <canvas id="chart"></canvas>
    </div>
    <div class="crosshair-tooltip" id="tooltip"></div>
    <!-- Field View -->
    <div class="field-container" id="fieldContainer">
      <div class="field-settings">
        <label>Position Tag:</label>
        <input type="text" id="posTag" placeholder="KLog_Odometry_IMU_Position" style="width:220px">
        <button class="btn btn-small btn-add" onclick="setPositionSource()">Set</button>
        <div class="sep"></div>
        <label>Zero X:</label>
        <input type="number" id="zeroX" value="0" step="10"> <label style="color:#666">mm</label>
        <label>Zero Y:</label>
        <input type="number" id="zeroY" value="0" step="10"> <label style="color:#666">mm</label>
        <div class="sep"></div>
        <label>Trail:</label>
        <input type="number" id="trailBuffer" value="30" min="1" step="5" title="Trail duration in seconds">
        <label style="color:#666">s</label>
        <label>Color:</label>
        <input type="color" id="trailColor" value="#00ff88">
      </div>
      <div class="field-canvas-wrap">
        <canvas id="fieldCanvas"></canvas>
        <div class="field-readout" id="fieldReadout">X: -- Y: -- &theta;: --</div>
      </div>
    </div>
  </div>
</div>

<script>
// ── Shared State ────────────────────────────────────────────────
const seriesList = {};
let seriesCounter = 0;
let isStreaming = false;
let eventSource = null;
let chart = null;
let currentMode = 'timeseries';
const MAX_LOG_LINES = 3000;

// Position data
const positionData = []; // [{time, x, y, theta}]  x/y in mm, theta in rad
const MAX_POS_POINTS = 10000;
let posSourceActive = false;

// ── Chart Setup (Time Series) ───────────────────────────────────
function initChart() {
  const ctx = document.getElementById('chart').getContext('2d');
  chart = new Chart(ctx, {
    type: 'line',
    data: { datasets: [] },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: { mode: 'nearest', axis: 'x', intersect: false },
      scales: {
        x: {
          type: 'time',
          time: { tooltipFormat: 'HH:mm:ss.SSS', displayFormats: { second: 'HH:mm:ss', millisecond: 'HH:mm:ss.SSS' } },
          ticks: { color: '#aaa', font: { size: 10 } },
          grid: { color: 'rgba(255,255,255,0.05)' },
          title: { display: true, text: 'Time', color: '#aaa' }
        },
        y: {
          ticks: { color: '#aaa', font: { size: 10 } },
          grid: { color: 'rgba(255,255,255,0.05)' },
          title: { display: true, text: 'Value', color: '#aaa' }
        }
      },
      plugins: {
        legend: { labels: { color: '#ddd', font: { size: 11 } } },
        tooltip: { enabled: false },
        zoom: {
          pan: { enabled: true, mode: 'x', modifierKey: null },
          zoom: {
            wheel: { enabled: true },
            pinch: { enabled: true },
            mode: 'x'
          }
        }
      },
      onClick: (evt) => { showCrosshair(evt); }
    }
  });

  // Double-click to reset zoom
  document.getElementById('chart').addEventListener('dblclick', () => { resetZoom(); });
}

function resetZoom() {
  if (chart) chart.resetZoom();
}

// ── Crosshair ───────────────────────────────────────────────────
let crosshairPlugin = {
  id: 'crosshair',
  _x: null,
  afterDraw(chart) {
    if (this._x === null) return;
    const ctx = chart.ctx;
    const yAxis = chart.scales.y;
    ctx.save();
    ctx.beginPath();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = 'rgba(255,255,255,0.5)';
    ctx.lineWidth = 1;
    ctx.moveTo(this._x, yAxis.top);
    ctx.lineTo(this._x, yAxis.bottom);
    ctx.stroke();
    ctx.restore();
  }
};
Chart.register(crosshairPlugin);

function showCrosshair(evt) {
  const points = chart.getElementsAtEventForMode(evt, 'nearest', { axis: 'x', intersect: false }, false);
  if (!points.length) { hideCrosshair(); return; }

  const xPixel = points[0].element.x;
  crosshairPlugin._x = xPixel;

  const ds = chart.data.datasets[points[0].datasetIndex];
  const idx = points[0].index;
  const clickTime = ds.data[idx].x;

  const tooltip = document.getElementById('tooltip');
  let html = '<div style="font-weight:600;margin-bottom:4px;">' + new Date(clickTime).toLocaleTimeString('en-US', {hour12:false, hour:'2-digit',minute:'2-digit',second:'2-digit',fractionalSecondDigits:3}) + '</div>';

  chart.data.datasets.forEach((dataset) => {
    let best = null, bestDiff = Infinity;
    for (let i = 0; i < dataset.data.length; i++) {
      const diff = Math.abs(new Date(dataset.data[i].x) - new Date(clickTime));
      if (diff < bestDiff) { bestDiff = diff; best = dataset.data[i]; }
    }
    if (best && bestDiff < 10000) {
      html += '<div class="tt-row"><span class="tt-swatch" style="background:' + dataset.borderColor + '"></span>'
            + '<span>' + dataset.label + ': <b>' + best.y.toFixed(4) + '</b></span></div>';
    }
  });

  tooltip.innerHTML = html;
  tooltip.style.display = 'block';

  const container = document.getElementById('graphContainer');
  const rect = container.getBoundingClientRect();
  let left = evt.native.clientX - rect.left + 15;
  let top = evt.native.clientY - rect.top - 10;
  if (left + 200 > rect.width) left = left - 230;
  if (top < 0) top = 10;
  tooltip.style.left = left + 'px';
  tooltip.style.top = top + 'px';

  chart.update('none');
}

function hideCrosshair() {
  crosshairPlugin._x = null;
  document.getElementById('tooltip').style.display = 'none';
  chart.update('none');
}

// ── Mode Toggle ─────────────────────────────────────────────────
function setMode(mode) {
  currentMode = mode;
  document.querySelectorAll('.mode-tab').forEach(t => t.classList.remove('active'));
  if (mode === 'timeseries') {
    document.querySelector('.mode-tab:nth-child(1)').classList.add('active');
    document.getElementById('graphContainer').style.display = '';
    document.getElementById('tooltip').style.display = 'none';
    document.getElementById('fieldContainer').style.display = 'none';
  } else {
    document.querySelector('.mode-tab:nth-child(2)').classList.add('active');
    document.getElementById('graphContainer').style.display = 'none';
    document.getElementById('tooltip').style.display = 'none';
    document.getElementById('fieldContainer').style.display = 'flex';
    resizeFieldCanvas();
    drawField();
  }
}

// ── Series Management ───────────────────────────────────────────
function addSeries() {
  const tag = document.getElementById('tagInput').value.trim();
  const key = document.getElementById('keyInput').value.trim();
  const color = document.getElementById('colorInput').value;
  if (!tag || !key) return;

  seriesCounter++;
  const sid = seriesCounter;
  seriesList[sid] = { tag, key, color, data: [] };

  chart.data.datasets.push({
    label: tag + ' : ' + key,
    borderColor: color,
    backgroundColor: color + '33',
    borderWidth: 1.5,
    pointRadius: 0,
    data: [],
    _sid: sid
  });
  chart.update('none');

  fetch('/api/series', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id: sid, tag, key, color })
  });

  document.getElementById('tagInput').value = '';
  document.getElementById('keyInput').value = '';
  refreshSeriesBar();
}

function removeSeries(sid) {
  delete seriesList[sid];
  const idx = chart.data.datasets.findIndex(d => d._sid === sid);
  if (idx >= 0) chart.data.datasets.splice(idx, 1);
  chart.update('none');

  fetch('/api/series/' + sid, { method: 'DELETE' });
  refreshSeriesBar();
}

function refreshSeriesBar() {
  const bar = document.getElementById('seriesBar');
  const ids = Object.keys(seriesList);
  if (!ids.length) { bar.innerHTML = '<span class="no-series">No series added</span>'; return; }

  bar.innerHTML = ids.map(sid => {
    const s = seriesList[sid];
    return '<span class="series-chip"><span class="swatch" style="background:' + s.color + '"></span>'
      + s.tag + ' : ' + s.key
      + '<span class="remove" onclick="removeSeries(' + sid + ')">&#10005;</span></span>';
  }).join('');
}

// ── Position Source ─────────────────────────────────────────────
function setPositionSource() {
  const tag = document.getElementById('posTag').value.trim();
  if (!tag) return;
  posSourceActive = true;
  fetch('/api/position-source', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ tag })
  });
}

// ── Streaming ───────────────────────────────────────────────────
function toggleStream() {
  if (isStreaming) stopStream(); else startStream();
}

function startStream() {
  const hasSeries = Object.keys(seriesList).length > 0;
  const hasPos = posSourceActive;
  if (!hasSeries && !hasPos) return;
  fetch('/api/start', { method: 'POST' }).then(r => {
    if (!r.ok) return;
    isStreaming = true;
    document.getElementById('streamBtn').textContent = 'Stop';
    document.getElementById('streamBtn').className = 'btn btn-stop';
    connectSSE();
  });
}

function stopStream() {
  fetch('/api/stop', { method: 'POST' });
  isStreaming = false;
  document.getElementById('streamBtn').textContent = 'Start';
  document.getElementById('streamBtn').className = 'btn btn-start';
  if (eventSource) { eventSource.close(); eventSource = null; }
}

function connectSSE() {
  if (eventSource) eventSource.close();
  eventSource = new EventSource('/api/events');
  eventSource.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    if (msg.type === 'data') handleData(msg);
    else if (msg.type === 'position') handlePosition(msg);
  };
  eventSource.onerror = () => {
    if (!isStreaming) return;
    setTimeout(() => { if (isStreaming) connectSSE(); }, 1000);
  };
}

let pendingUpdate = false;
function handleData(msg) {
  const sid = msg.series_id;
  if (!seriesList[sid]) return;

  const point = { x: new Date(msg.time), y: msg.value };
  seriesList[sid].data.push(point);

  const ds = chart.data.datasets.find(d => d._sid === sid);
  if (ds) ds.data.push(point);

  appendLog(msg.log, seriesList[sid].color);

  if (!pendingUpdate) {
    pendingUpdate = true;
    requestAnimationFrame(() => {
      applyTimeWindow();
      chart.update('none');
      pendingUpdate = false;
    });
  }
}

function handlePosition(msg) {
  const pt = { time: new Date(msg.time), x: msg.x, y: msg.y, theta: msg.theta };
  positionData.push(pt);
  while (positionData.length > MAX_POS_POINTS) positionData.shift();

  appendLog(msg.log, document.getElementById('trailColor').value);

  if (currentMode === 'field' && !pendingFieldDraw) {
    pendingFieldDraw = true;
    requestAnimationFrame(() => { drawField(); pendingFieldDraw = false; });
  }
}
let pendingFieldDraw = false;

// ── Time Window ─────────────────────────────────────────────────
function applyTimeWindow() {
  const windowSec = parseInt(document.getElementById('timeWindow').value) || 0;
  if (windowSec <= 0) return; // show all

  const cutoff = new Date(Date.now() - windowSec * 1000);
  chart.data.datasets.forEach(ds => {
    // Find first index >= cutoff
    let startIdx = 0;
    while (startIdx < ds.data.length && ds.data[startIdx].x < cutoff) startIdx++;
    if (startIdx > 0) ds.data.splice(0, startIdx);
  });
}

// ── Log Panel ───────────────────────────────────────────────────
function appendLog(text, color) {
  const el = document.getElementById('logContent');
  const line = document.createElement('div');
  line.className = 'log-line';
  line.style.color = color;
  line.textContent = text;
  el.appendChild(line);

  while (el.children.length > MAX_LOG_LINES) el.removeChild(el.firstChild);

  if (document.getElementById('autoScroll').checked) {
    el.scrollTop = el.scrollHeight;
  }
}

function clearData() {
  Object.values(seriesList).forEach(s => s.data = []);
  chart.data.datasets.forEach(ds => ds.data = []);
  chart.update('none');
  positionData.length = 0;
  document.getElementById('logContent').innerHTML = '';
  hideCrosshair();
  drawField();
  fetch('/api/clear', { method: 'POST' });
}

// ── Field View ──────────────────────────────────────────────────
const FIELD_SIZE_MM = 3657.6; // 12ft in mm

function resizeFieldCanvas() {
  const wrap = document.querySelector('.field-canvas-wrap');
  const canvas = document.getElementById('fieldCanvas');
  const size = Math.min(wrap.clientWidth - 16, wrap.clientHeight - 50);
  canvas.width = size;
  canvas.height = size;
}

function drawField() {
  const canvas = document.getElementById('fieldCanvas');
  if (!canvas || canvas.width === 0) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width;
  const H = canvas.height;
  const pad = 30;
  const fieldPx = Math.min(W, H) - 2 * pad;
  const scale = fieldPx / FIELD_SIZE_MM;

  const zeroX = parseFloat(document.getElementById('zeroX').value) || 0;
  const zeroY = parseFloat(document.getElementById('zeroY').value) || 0;
  const trailSec = parseFloat(document.getElementById('trailBuffer').value) || 30;
  const trailColor = document.getElementById('trailColor').value;

  // Clear
  ctx.fillStyle = '#111';
  ctx.fillRect(0, 0, W, H);

  // Field boundary
  ctx.strokeStyle = '#444';
  ctx.lineWidth = 2;
  ctx.strokeRect(pad, pad, fieldPx, fieldPx);

  // Grid (1ft = 12in intervals)
  ctx.strokeStyle = '#222';
  ctx.lineWidth = 0.5;
  for (let i = 1; i < 12; i++) {
    const p = pad + i * (fieldPx / 12);
    ctx.beginPath(); ctx.moveTo(p, pad); ctx.lineTo(p, pad + fieldPx); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(pad, p); ctx.lineTo(pad + fieldPx, p); ctx.stroke();
  }

  // Axis labels
  // Horizontal axis = Y (mm), Vertical axis = X (mm)
  // X increases upward, Y increases to the right
  ctx.fillStyle = '#555';
  ctx.font = '10px sans-serif';
  ctx.textAlign = 'center';
  const GRID_STEPS = 12;
  const STEP_MM = FIELD_SIZE_MM / GRID_STEPS;
  for (let i = 0; i <= GRID_STEPS; i++) {
    const label = Math.round(i * STEP_MM).toString();
    const p = pad + i * (fieldPx / GRID_STEPS);
    // Bottom axis: Y labels (0 left, 3658 right)
    ctx.fillText(label, p, pad + fieldPx + 14);
    // Left axis: X labels (0 at bottom, 3658 at top)
    ctx.save();
    ctx.translate(pad - 8, pad + fieldPx - i * (fieldPx / GRID_STEPS));
    ctx.rotate(-Math.PI / 2);
    ctx.fillText(label, 0, 0);
    ctx.restore();
  }
  // Axis titles
  ctx.fillStyle = '#666';
  ctx.font = '11px sans-serif';
  ctx.fillText('Y (mm)', pad + fieldPx / 2, pad + fieldPx + 26);
  ctx.save();
  ctx.translate(pad - 22, pad + fieldPx / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.fillText('X (mm)', 0, 0);
  ctx.restore();

  // Helper: field coords (mm) to canvas pixels
  // Horizontal = Y, Vertical = X (X increases upward)
  function toCanvas(xMm, yMm) {
    return [pad + yMm * scale, pad + fieldPx - xMm * scale];
  }

  // Filter to trail buffer
  const now = positionData.length > 0 ? positionData[positionData.length - 1].time : new Date();
  const cutoff = new Date(now.getTime() - trailSec * 1000);
  const trail = positionData.filter(p => p.time >= cutoff);

  if (trail.length === 0) {
    document.getElementById('fieldReadout').innerHTML = 'X: -- &nbsp; Y: -- &nbsp; &theta;: --';
    return;
  }

  // Draw trail
  ctx.strokeStyle = trailColor;
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  ctx.beginPath();
  for (let i = 0; i < trail.length; i++) {
    const xMm = trail[i].x + zeroX;
    const yMm = trail[i].y + zeroY;
    const [cx, cy] = toCanvas(xMm, yMm);
    if (i === 0) ctx.moveTo(cx, cy);
    else ctx.lineTo(cx, cy);
  }
  ctx.stroke();

  // Add dots at intervals along trail
  if (trail.length > 2) {
    const dotInterval = Math.max(1, Math.floor(trail.length / 50));
    ctx.fillStyle = trailColor;
    for (let i = 0; i < trail.length; i += dotInterval) {
      const xMm = trail[i].x + zeroX;
      const yMm = trail[i].y + zeroY;
      const [cx, cy] = toCanvas(xMm, yMm);
      ctx.beginPath();
      ctx.arc(cx, cy, 2, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Draw robot (triangle pointing in theta direction)
  const cur = trail[trail.length - 1];
  const curXMm = cur.x + zeroX;
  const curYMm = cur.y + zeroY;
  const [rx, ry] = toCanvas(curXMm, curYMm);
  const robotSize = 8;
  // theta from odometry is in radians, CCW positive
  // Display heading: negate so turning right (CW) = positive
  // On canvas with swapped axes: robot points up (negative Y canvas) at theta=0
  // Canvas rotation: -theta rotates CW on screen (which matches right turn = positive)
  const drawTheta = -(cur.theta - Math.PI / 2);

  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(drawTheta);
  ctx.fillStyle = '#ff4444';
  ctx.beginPath();
  ctx.moveTo(robotSize, 0);
  ctx.lineTo(-robotSize * 0.6, -robotSize * 0.5);
  ctx.lineTo(-robotSize * 0.6, robotSize * 0.5);
  ctx.closePath();
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.restore();

  // Readout: negate theta so CW (right turn) = positive degrees
  const thetaDeg = (-cur.theta * 180 / Math.PI).toFixed(1);
  document.getElementById('fieldReadout').innerHTML =
    'X: ' + curXMm.toFixed(1) + ' mm &nbsp;&nbsp; Y: ' + curYMm.toFixed(1) + ' mm &nbsp;&nbsp; &theta;: ' + thetaDeg + '&deg;';
}

// ── Init ────────────────────────────────────────────────────────
window.addEventListener('load', () => {
  initChart();
  new ResizeObserver(() => {
    if (chart) chart.resize();
    if (currentMode === 'field') { resizeFieldCanvas(); drawField(); }
  }).observe(document.querySelector('.graph-panel'));
});
</script>
</body>
</html>
'''


# ── HTTP Server ──────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())

        elif self.path == '/api/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()

            q = Queue()
            data_queues.append(q)
            try:
                while True:
                    try:
                        event = q.get(timeout=30)
                        self.wfile.write(f"data: {event}\n\n".encode())
                        self.wfile.flush()
                    except Empty:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                if q in data_queues:
                    data_queues.remove(q)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/api/series':
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            sid = body['id']
            with series_lock:
                series_map[sid] = {
                    'tag': body['tag'],
                    'key': body['key'],
                    'color': body['color'],
                    'pattern': make_pattern(body['key'])
                }
            self.send_response(200)
            self.end_headers()

        elif self.path == '/api/position-source':
            global position_source
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            with position_lock:
                position_source = {'tag': body['tag']}
            self.send_response(200)
            self.end_headers()

        elif self.path == '/api/start':
            ok = start_logcat()
            self.send_response(200 if ok or streaming else 500)
            self.end_headers()

        elif self.path == '/api/stop':
            stop_logcat()
            self.send_response(200)
            self.end_headers()

        elif self.path == '/api/clear':
            self.send_response(200)
            self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()

    def do_DELETE(self):
        if self.path.startswith('/api/series/'):
            try:
                sid = int(self.path.split('/')[-1])
                with series_lock:
                    series_map.pop(sid, None)
            except ValueError:
                pass
            self.send_response(200)
            self.end_headers()

        elif self.path == '/api/position-source':
            global position_source
            with position_lock:
                position_source = None
            self.send_response(200)
            self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()


def main():
    port = 8765
    server = HTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True

    print(f"Logcat Grapher running at http://localhost:{port}")
    print("Press Ctrl+C to stop.\n")

    webbrowser.open(f"http://localhost:{port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        stop_logcat()
        server.shutdown()


if __name__ == "__main__":
    main()
