"""Generate a self-contained HTML preview from the optimizer CSV outputs."""

from __future__ import annotations

import csv
import json
from pathlib import Path


def _load_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def _float_rows(rows: list[dict[str, str]], keys: tuple[str, ...]) -> list[list[float]]:
    return [[float(row[key]) for key in keys] for row in rows]


def compute_metrics(
    sim_rows: list[dict[str, str]], *, base_y: float, h_high: float
) -> dict:
    times = [float(row["time_s"]) for row in sim_rows]
    x = [float(row["x_mm"]) for row in sim_rows]
    y = [float(row["y_mm"]) for row in sim_rows]
    hip_vel = [float(row["hip_vel"]) for row in sim_rows]
    knee_vel = [float(row["knee_vel"]) for row in sim_rows]
    hip_accel = [float(row["hip_accel"]) for row in sim_rows]
    knee_accel = [float(row["knee_accel"]) for row in sim_rows]
    support = [int(float(row["support"])) for row in sim_rows]

    dt = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    dx = [x[i + 1] - x[i] for i in range(len(x) - 1)]
    lift = [v - base_y for v in y]

    support_blocks = sum(
        1 for i in range(1, len(support)) if support[i] and not support[i - 1]
    )
    if support and support[0] and support[-1]:
        support_blocks = max(0, support_blocks - 1)
    support_time = sum(dt[i] for i in range(len(dt)) if support[i])
    d_leg = sum(abs(v) for v in dx)
    d_sup = sum(abs(dx[i]) for i in range(len(dx)) if support[i])
    d_high = sum(
        abs(dx[i])
        for i in range(len(dx))
        if lift[i] >= h_high or lift[i + 1] >= h_high
    )
    d_high += d_sup
    d_total = d_leg + d_sup

    return {
        "cycle_s": times[-1] - times[0] if times else 0.0,
        "support_blocks": support_blocks,
        "support_time_s": support_time,
        "r_actual_mm": d_sup,
        "d_total_mm": d_total,
        "d_high_mm": d_high,
        "high_ratio": d_high / d_total if d_total > 1e-9 else 0.0,
        "max_hip_vel": max(abs(v) for v in hip_vel),
        "max_knee_vel": max(abs(v) for v in knee_vel),
        "max_hip_accel": max(abs(v) for v in hip_accel),
        "max_knee_accel": max(abs(v) for v in knee_accel),
    }


def write_preview(
    target_csv: Path,
    sim_csv: Path,
    preview_path: Path,
    *,
    l1: float = 125.0,
    l2: float = 100.0,
    base_y: float = 54.0,
    h_high: float = 25.0,
    eps_down: float = 3.0,
) -> Path:
    target_rows = _load_csv(target_csv)
    sim_rows = _load_csv(sim_csv)
    targets = _float_rows(
        target_rows, ("phase", "x_mm", "y_mm", "hip_deg", "knee_deg")
    )
    sim = _float_rows(
        sim_rows,
        (
            "time_s",
            "hip_deg",
            "knee_deg",
            "hip_vel",
            "knee_vel",
            "hip_accel",
            "knee_accel",
            "x_mm",
            "y_mm",
            "support",
        ),
    )
    metrics = compute_metrics(sim_rows, base_y=base_y, h_high=h_high)
    data = {
        "targets": targets,
        "sim": sim,
        "params": {"l1": l1, "l2": l2, "base_y": base_y, "h_high": h_high, "eps_down": eps_down},
        "metrics": metrics,
    }
    preview_path.parent.mkdir(parents=True, exist_ok=True)
    preview_path.write_text(
        _TEMPLATE.replace("__DATA__", json.dumps(data, ensure_ascii=False)),
        encoding="utf-8",
    )
    return preview_path


_TEMPLATE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Gait Optimizer Preview</title>
<style>
* { box-sizing: border-box; }
body { margin: 0; background: #eef2f6; color: #17202a; font-family: "Segoe UI", "Microsoft YaHei", sans-serif; }
.wrap { max-width: 1180px; margin: 0 auto; padding: 22px; }
.header { display: flex; justify-content: space-between; align-items: baseline; flex-wrap: wrap; gap: 10px; margin-bottom: 16px; }
h1 { font-size: 22px; margin: 0; }
.status { color: #52606d; font-size: 13px; }
.metrics { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; margin-bottom: 16px; }
.metric { background: #fff; border: 1px solid #d7dee8; border-radius: 8px; padding: 10px 12px; }
.metric .label { font-size: 12px; color: #6b7787; }
.metric .value { font-size: 20px; font-weight: 650; margin-top: 4px; }
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
@media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
.panel { background: #fff; border: 1px solid #d7dee8; border-radius: 8px; padding: 14px; }
.panel-title { font-weight: 650; font-size: 14px; margin-bottom: 10px; }
canvas { width: 100%; height: 280px; display: block; background: #fbfcfe; border: 1px solid #dfe5ee; border-radius: 6px; }
.controls { display: flex; align-items: center; gap: 10px; margin-top: 10px; }
button { border: 1px solid #c7cfdb; background: #fff; border-radius: 6px; padding: 7px 12px; font-size: 13px; cursor: pointer; }
input[type=range] { flex: 1; }
.legend { display: flex; flex-wrap: wrap; gap: 14px; margin-top: 8px; font-size: 12px; color: #52606d; }
.dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; margin-right: 4px; }
.swatch { display: inline-block; width: 12px; height: 12px; border-radius: 3px; margin-right: 4px; vertical-align: -2px; }
</style>
</head>
<body>
<div class="wrap">
  <div class="header">
    <h1>Gait Optimizer Preview</h1>
    <div class="status" id="status"></div>
  </div>
  <div class="metrics" id="metrics"></div>
  <div class="grid">
    <div class="panel">
      <div class="panel-title">Leg & Foot Trajectory</div>
      <canvas id="trajectory"></canvas>
      <div class="controls">
        <button id="play">Play</button>
        <input id="scrub" type="range" min="0" max="1000" value="0">
      </div>
      <div class="legend">
        <span><span class="dot" style="background:#0e7c7b"></span>actual foot path</span>
        <span><span class="dot" style="background:#d97706"></span>target waypoint</span>
      </div>
    </div>
    <div class="panel">
      <div class="panel-title">Joint Angles</div>
      <canvas id="angles"></canvas>
      <div class="legend">
        <span><span class="dot" style="background:#0e7c7b"></span>hip</span>
        <span><span class="dot" style="background:#d97706"></span>knee</span>
      </div>
    </div>
    <div class="panel">
      <div class="panel-title">Foot Height</div>
      <canvas id="height"></canvas>
      <div class="legend">
        <span><span class="swatch" style="background:#dbe7ff"></span>support</span>
        <span><span class="dot" style="background:#dc2626"></span>h_high</span>
      </div>
    </div>
    <div class="panel">
      <div class="panel-title">Joint Velocity</div>
      <canvas id="velocity"></canvas>
      <div class="legend">
        <span><span class="dot" style="background:#0e7c7b"></span>hip vel</span>
        <span><span class="dot" style="background:#d97706"></span>knee vel</span>
      </div>
    </div>
  </div>
</div>
<script>
const DATA = __DATA__;
const sim = DATA.sim;
const targets = DATA.targets;
const p = DATA.params;
const m = DATA.metrics;

function setupCanvas(id) {
  const canvas = document.getElementById(id);
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { canvas, ctx, w: rect.width, h: rect.height };
}

function lineChart(id, series, options = {}) {
  const { canvas, ctx, w, h } = setupCanvas(id);
  ctx.clearRect(0, 0, w, h);
  const padL = 48, padR = 12, padT = 12, padB = 24;
  const plotW = w - padL - padR, plotH = h - padT - padB;
  let minX = 0, maxX = sim[sim.length - 1][0] || 1;
  let minY = Infinity, maxY = -Infinity;
  for (const s of series) {
    for (const row of sim) {
      minY = Math.min(minY, row[s]);
      maxY = Math.max(maxY, row[s]);
    }
  }
  if (options.fixedMin !== undefined) minY = options.fixedMin;
  if (options.fixedMax !== undefined) maxY = options.fixedMax;
  const pad = (maxY - minY) * 0.12 || 1;
  minY -= pad; maxY += pad;
  const tx = (t) => padL + ((t - minX) / (maxX - minX)) * plotW;
  const ty = (v) => padT + (1 - (v - minY) / (maxY - minY)) * plotH;
  ctx.strokeStyle = "#d5dce6";
  ctx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const y = padT + plotH * i / 4;
    ctx.moveTo(padL, y);
    ctx.lineTo(w - padR, y);
  }
  ctx.stroke();
  ctx.fillStyle = "#667085";
  ctx.font = "11px sans-serif";
  for (let i = 0; i <= 4; i++) {
    const y = padT + plotH * i / 4;
    const value = maxY - (maxY - minY) * i / 4;
    ctx.fillText(value.toFixed(0), 4, y + 4);
  }
  series.forEach((col, idx) => {
    const color = idx === 0 ? "#0e7c7b" : "#d97706";
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    sim.forEach((row, i) => {
      const x = tx(row[0]);
      const y = ty(row[col]);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
}

function drawTrajectory() {
  const { canvas, ctx, w, h } = setupCanvas("trajectory");
  ctx.clearRect(0, 0, w, h);
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const row of sim) {
    minX = Math.min(minX, row[7]);
    maxX = Math.max(maxX, row[7]);
    minY = Math.min(minY, row[8]);
    maxY = Math.max(maxY, row[8]);
  }
  minY = Math.min(minY, p.base_y - 20);
  maxY = p.l1 + p.l2 + 20;
  const pad = 20;
  const sx = (w - 2 * pad) / (maxX - minX || 1);
  const sy = (h - 2 * pad) / (maxY - minY || 1);
  const scale = Math.min(sx, sy);
  const px = (x) => pad + (x - minX) * scale;
  const py = (y) => h - pad - (y - minY) * scale;

  ctx.strokeStyle = "#b7c1ce";
  ctx.setLineDash([5, 5]);
  ctx.beginPath();
  ctx.moveTo(0, py(p.base_y));
  ctx.lineTo(w, py(p.base_y));
  ctx.stroke();
  ctx.setLineDash([]);

  ctx.strokeStyle = "#0e7c7b";
  ctx.lineWidth = 2;
  ctx.beginPath();
  sim.forEach((row, i) => {
    const x = px(row[7]), y = py(row[8]);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  ctx.fillStyle = "#d97706";
  targets.forEach((row) => {
    ctx.beginPath();
    ctx.arc(px(row[1]), py(row[2]), 4, 0, Math.PI * 2);
    ctx.fill();
  });

  const idx = Math.max(0, Math.min(sim.length - 1, Math.round(scrub.value / 1000 * (sim.length - 1))));
  const row = sim[idx];
  const hipRad = row[1] * Math.PI / 180;
  const kneeRad = (row[1] + row[2]) * Math.PI / 180;
  const hipX = px(0), hipY = py(p.l1 + p.l2);
  const kneeX = px(p.l1 * Math.sin(hipRad));
  const kneeY = py(p.l1 + p.l2 - p.l1 * Math.cos(hipRad));
  const footX = px(row[7]), footY = py(row[8]);
  ctx.strokeStyle = "#18202b";
  ctx.lineWidth = 4;
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(hipX, hipY);
  ctx.lineTo(kneeX, kneeY);
  ctx.lineTo(footX, footY);
  ctx.stroke();
  ctx.fillStyle = "#0e7c7b";
  ctx.beginPath();
  ctx.arc(footX, footY, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#18202b";
  ctx.beginPath();
  ctx.arc(kneeX, kneeY, 4, 0, Math.PI * 2);
  ctx.fill();
}

function drawHeight() {
  const { canvas, ctx, w, h } = setupCanvas("height");
  ctx.clearRect(0, 0, w, h);
  const padL = 48, padR = 12, padT = 12, padB = 24;
  const plotW = w - padL - padR, plotH = h - padT - padB;
  const maxT = sim[sim.length - 1][0] || 1;
  let maxLift = 0;
  for (const row of sim) maxLift = Math.max(maxLift, row[8] - p.base_y);
  maxLift = Math.max(maxLift, p.h_high * 1.3);
  const tx = (t) => padL + t / maxT * plotW;
  const ty = (v) => padT + (1 - v / maxLift) * plotH;
  ctx.fillStyle = "#dbe7ff";
  sim.forEach((row, i) => {
    if (i === 0 || row[9] < 0.5) return;
    const x0 = tx(sim[i - 1][0]);
    const x1 = tx(row[0]);
    ctx.fillRect(x0, padT, x1 - x0, plotH);
  });
  ctx.strokeStyle = "#dc2626";
  ctx.setLineDash([6, 5]);
  ctx.beginPath();
  ctx.moveTo(padL, ty(p.h_high));
  ctx.lineTo(w - padR, ty(p.h_high));
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.strokeStyle = "#0e7c7b";
  ctx.lineWidth = 2;
  ctx.beginPath();
  sim.forEach((row, i) => {
    const x = tx(row[0]);
    const y = ty(row[8] - p.base_y);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function renderMetrics() {
  const container = document.getElementById("metrics");
  const items = [
    ["Support time", m.support_time_s.toFixed(3) + " s"],
    ["Support blocks", m.support_blocks],
    ["r_actual", m.r_actual_mm.toFixed(1) + " mm"],
    ["High ratio", m.high_ratio.toFixed(3)]
  ];
  container.innerHTML = items.map(([label, value]) =>
    `<div class="metric"><div class="label">${label}</div><div class="value">${value}</div></div>`
  ).join("");
  document.getElementById("status").textContent =
    `cycle ${m.cycle_s.toFixed(3)} s | max vel ${Math.max(m.max_hip_vel, m.max_knee_vel).toFixed(0)} deg/s | max accel ${Math.max(m.max_hip_accel, m.max_knee_accel).toFixed(0)} deg/s²`;
}

const scrub = document.getElementById("scrub");
const playBtn = document.getElementById("play");
let playing = false;
let timer = null;

function render() {
  drawTrajectory();
  lineChart("angles", [1, 2]);
  drawHeight();
  lineChart("velocity", [3, 4]);
}

function step() {
  const max = sim.length - 1;
  let idx = Math.round(scrub.value / 1000 * max);
  idx = idx >= max ? 0 : idx + 1;
  scrub.value = Math.round(idx / max * 1000);
  drawTrajectory();
}

playBtn.addEventListener("click", () => {
  playing = !playing;
  playBtn.textContent = playing ? "Pause" : "Play";
  if (playing) timer = setInterval(step, 60);
  else if (timer) clearInterval(timer);
});
scrub.addEventListener("input", render);
renderMetrics();
render();
window.addEventListener("resize", render);
</script>
</body>
</html>
"""
