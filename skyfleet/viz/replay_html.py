"""生成自包含的 HTML 回放页面。

"自包含"是硬要求：所有数据内嵌、脚本与样式内联、不引用任何外部资源。
答辩现场可能没有网络，甚至可能是离线机器——外链会让页面直接白屏。
"""
from __future__ import annotations

import json
from pathlib import Path

from skyfleet.observe.replay import build_timeline

_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SkyFleet 集群通信仿真回放</title>
<style>
  :root { --ink:#1b2631; --muted:#5d6d7e; --line:#d5dbdb; --bg:#fbfcfc; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--ink);
         font-family:"Noto Sans CJK SC","Noto Sans SC",system-ui,-apple-system,
                     "Segoe UI",Roboto,sans-serif; }
  header { padding:14px 20px; border-bottom:1px solid var(--line);
           display:flex; align-items:baseline; gap:14px; flex-wrap:wrap; }
  h1 { font-size:17px; margin:0; font-weight:600; }
  .meta { color:var(--muted); font-size:12px; }
  main { display:grid; grid-template-columns:minmax(320px,1fr) 320px;
         gap:18px; padding:18px 20px 28px; align-items:start; }
  @media (max-width:900px){ main { grid-template-columns:1fr; } }
  .panel { background:#fff; border:1px solid var(--line); border-radius:8px;
           padding:14px; }
  canvas { width:100%; height:auto; display:block; border-radius:6px;
           background:#fff; }
  .controls { display:flex; align-items:center; gap:10px; margin-top:12px;
              flex-wrap:wrap; }
  button { font:inherit; font-size:13px; padding:6px 14px; border-radius:6px;
           border:1px solid var(--line); background:#fff; cursor:pointer; }
  button:hover { background:#f4f6f7; }
  input[type=range] { flex:1; min-width:140px; }
  .clock { font-variant-numeric:tabular-nums; color:var(--muted);
           font-size:13px; min-width:72px; }
  table { width:100%; border-collapse:collapse; font-size:12.5px; }
  th,td { text-align:left; padding:4px 6px; border-bottom:1px solid #eef1f2; }
  th { color:var(--muted); font-weight:500; }
  .tag { display:inline-block; padding:1px 6px; border-radius:4px;
         font-size:11px; color:#fff; }
  .ONLINE{background:#1e8449}.DEGRADED{background:#b9770e}
  .SUSPECT{background:#ca6f1e}.FAILED{background:#c0392b}
  .LEFT{background:#566573}.REJOINING{background:#2471a3}
  .legend { display:flex; gap:12px; font-size:12px; color:var(--muted);
            margin-top:8px; flex-wrap:wrap; }
  .swatch { display:inline-block; width:10px; height:10px; border-radius:2px;
            margin-right:4px; }
  ol.ticker { list-style:none; margin:0; padding:0; max-height:220px;
              overflow:auto; font-size:12px; }
  ol.ticker li { padding:3px 0; border-bottom:1px solid #f2f4f5;
                 font-variant-numeric:tabular-nums; }
  h2 { font-size:13px; margin:0 0 8px; color:var(--muted); font-weight:600;
       text-transform:uppercase; letter-spacing:.04em; }
</style>
</head>
<body>
<header>
  <h1>SkyFleet 集群通信仿真回放</h1>
  <span class="meta" id="world-meta"></span>
  <span class="meta">数据全部内嵌，无外部依赖</span>
</header>
<main>
  <section class="panel">
    <canvas id="stage" width="720" height="720"></canvas>
    <div class="controls">
      <button id="play">播放</button>
      <button id="reset">回到起点</button>
      <label class="clock" id="clock">0.0 s</label>
      <input type="range" id="scrub" min="0" max="1000" value="0">
      <label class="clock" style="min-width:auto">速度
        <select id="speed">
          <option value="1">1x</option>
          <option value="4" selected>4x</option>
          <option value="16">16x</option>
          <option value="64">64x</option>
        </select>
      </label>
    </div>
    <div class="legend">
      <span><i class="swatch" style="background:#c0392b"></i>主节点</span>
      <span><i class="swatch" style="background:#2471a3"></i>从节点</span>
      <span><i class="swatch" style="background:#1e8449"></i>中继</span>
      <span><i class="swatch" style="background:#b9770e"></i>取餐点→投递点</span>
      <span>灰度为楼高</span>
    </div>
  </section>
  <aside>
    <div class="panel">
      <h2>节点状态</h2>
      <table><thead><tr><th>节点</th><th>角色</th><th>状态</th></tr></thead>
      <tbody id="node-rows"></tbody></table>
    </div>
    <div class="panel" style="margin-top:14px">
      <h2>任务进度</h2>
      <table><thead><tr><th>任务</th><th>承担</th><th>状态</th></tr></thead>
      <tbody id="task-rows"></tbody></table>
    </div>
    <div class="panel" style="margin-top:14px">
      <h2>事件流</h2>
      <ol class="ticker" id="ticker"></ol>
    </div>
  </aside>
</main>
<script>
const DATA = __DATA__;

const STATUS_ORDER = ["REJOINING","LEFT","FAILED","SUSPECT","DEGRADED","ONLINE"];
const ROLE_COLOUR = { LEADER:"#c0392b", SLAVE:"#2471a3", RELAY:"#1e8449" };
const STATE_COLOUR = { CREATED:"#7f8c8d", ASSIGNED:"#b9770e",
  REASSIGNED:"#8e44ad", PICKED:"#2471a3", DELIVERED:"#1e8449",
  FAILED:"#c0392b", EXPIRED:"#566573" };

const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");
const scrub = document.getElementById("scrub");
const clock = document.getElementById("clock");
const ticker = document.getElementById("ticker");
const nodeRows = document.getElementById("node-rows");
const taskRows = document.getElementById("task-rows");
const playBtn = document.getElementById("play");
const speedSel = document.getElementById("speed");

const DURATION = DATA.duration || 1;
const AREA = DATA.area || 1;
let current = 0, playing = false, lastFrame = 0;

document.getElementById("world-meta").textContent =
  `区域 ${AREA} m × ${AREA} m · 时长 ${DURATION} s · 种子 ${DATA.seed} · ` +
  `${Object.keys(DATA.nodes).length} 节点 · ${Object.keys(DATA.tasks).length} 任务`;

// 世界坐标 -> 画布像素（y 轴翻转，符合"北向上"的直觉）
const MARGIN = 28;
const scale = (canvas.width - 2 * MARGIN) / AREA;
function sx(x) { return MARGIN + x * scale; }
function sy(y) { return canvas.height - MARGIN - y * scale; }

const heights = DATA.buildings.map(b => b[4]);
const loH = Math.min(...heights, 0), hiH = Math.max(...heights, 1);

function statusAt(history, t) {
  let value = "ONLINE";
  for (const [ts, st] of history) { if (ts <= t) value = st; else break; }
  return value;
}

function sampleAt(samples, t) {
  if (!samples.length) return null;
  let prev = samples[0], next = null;
  for (let i = 0; i < samples.length; i++) {
    if (samples[i][0] <= t) prev = samples[i];
    else { next = samples[i]; break; }
  }
  if (!next) return [prev[1], prev[2], prev[3]];
  const span = next[0] - prev[0];
  const k = span > 0 ? (t - prev[0]) / span : 0;
  return [prev[1] + (next[1] - prev[1]) * k,
          prev[2] + (next[2] - prev[2]) * k,
          prev[3] + (next[3] - prev[3]) * k];
}

function drawWorld() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = "#eef1f2";
  ctx.lineWidth = 1;
  for (let g = 0; g <= AREA; g += 100) {
    ctx.beginPath(); ctx.moveTo(sx(g), sy(0)); ctx.lineTo(sx(g), sy(AREA));
    ctx.stroke();
    ctx.beginPath(); ctx.moveTo(sx(0), sy(g)); ctx.lineTo(sx(AREA), sy(g));
    ctx.stroke();
  }

  for (const [x, y, w, h, height] of DATA.buildings) {
    const shade = Math.round(255 - 140 * (height - loH) / ((hiH - loH) || 1));
    ctx.fillStyle = `rgb(${shade},${shade},${shade})`;
    ctx.fillRect(sx(x), sy(y + h), w * scale, h * scale);
    ctx.strokeStyle = "#7f8c8d";
    ctx.strokeRect(sx(x), sy(y + h), w * scale, h * scale);
  }

  for (const task of Object.values(DATA.tasks)) {
    ctx.strokeStyle = "rgba(185,119,14,0.45)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(sx(task.origin[0]), sy(task.origin[1]));
    ctx.lineTo(sx(task.destination[0]), sy(task.destination[1]));
    ctx.stroke();
    ctx.fillStyle = "#b9770e";
    ctx.fillRect(sx(task.origin[0]) - 2, sy(task.origin[1]) - 2, 4, 4);
    ctx.fillStyle = STATE_COLOUR[task.state] || "#1e8449";
    ctx.fillRect(sx(task.destination[0]) - 2.5, sy(task.destination[1]) - 2.5, 5, 5);
  }
}

function drawNodes(t) {
  for (const [id, node] of Object.entries(DATA.nodes)) {
    const status = statusAt(node.status, t);
    if (status === "LEFT" || status === "FAILED") continue;
    const pos = sampleAt(node.samples, t);
    if (!pos) continue;
    const colour = ROLE_COLOUR[node.role] || "#555";
    ctx.beginPath();
    ctx.arc(sx(pos[0]), sy(pos[1]), node.role === "LEADER" ? 6 : 4.5,
            0, Math.PI * 2);
    ctx.fillStyle = colour;
    ctx.globalAlpha = status === "DEGRADED" || status === "SUSPECT" ? 0.45 : 1;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.fillStyle = "#1b2631";
    ctx.font = "11px sans-serif";
    ctx.fillText(id, sx(pos[0]) + 7, sy(pos[1]) - 6);
  }
}

function renderPanels(t) {
  let rows = "";
  for (const id of Object.keys(DATA.nodes).sort()) {
    const node = DATA.nodes[id];
    const status = statusAt(node.status, t);
    rows += `<tr><td>${id}</td><td>${node.role}</td>` +
            `<td><span class="tag ${status}">${status}</span></td></tr>`;
  }
  nodeRows.innerHTML = rows;

  rows = "";
  for (const [id, task] of Object.entries(DATA.tasks).sort()) {
    let state = task.state;
    if (task.created > t) state = "—";
    else if (task.delivered !== null && task.delivered <= t) state = "DELIVERED";
    else if (task.picked !== null && task.picked <= t) state = "PICKED";
    else if (task.assigned !== null && task.assigned <= t) state = "ASSIGNED";
    else state = "CREATED";
    const colour = STATE_COLOUR[state] || "#7f8c8d";
    rows += `<tr><td>${id}</td><td>${task.assignee || "—"}</td>` +
            `<td><span class="tag" style="background:${colour}">${state}</span></td></tr>`;
  }
  taskRows.innerHTML = rows;

  const recent = DATA.events.filter(e => e[0] <= t).slice(-14).reverse();
  ticker.innerHTML = recent.map(e =>
    `<li>${e[0].toFixed(2)}s · ${e[1]} · ${e[2] || ""} ${e[3] || ""}</li>`
  ).join("");
}

function render(t) {
  current = Math.min(Math.max(t, 0), DURATION);
  drawWorld();
  drawNodes(current);
  renderPanels(current);
  clock.textContent = current.toFixed(1) + " s";
  scrub.value = String(Math.round(current / DURATION * 1000));
}

function frame(now) {
  if (!playing) return;
  const dt = lastFrame ? (now - lastFrame) / 1000 : 0;
  lastFrame = now;
  const next = current + dt * Number(speedSel.value);
  if (next >= DURATION) { render(DURATION); playing = false;
                          playBtn.textContent = "播放"; return; }
  render(next);
  requestAnimationFrame(frame);
}

playBtn.addEventListener("click", () => {
  playing = !playing;
  playBtn.textContent = playing ? "暂停" : "播放";
  if (playing) {
    if (current >= DURATION) current = 0;
    lastFrame = 0;
    requestAnimationFrame(frame);
  }
});
document.getElementById("reset").addEventListener("click", () => {
  playing = false; playBtn.textContent = "播放"; render(0);
});
scrub.addEventListener("input", () => {
  playing = false; playBtn.textContent = "播放";
  render(Number(scrub.value) / 1000 * DURATION);
});

render(0);
</script>
</body>
</html>
"""


def render_replay(log_path: str | Path, out_path: str | Path) -> str:
    """把日志渲染成单文件 HTML 回放，返回写出的路径。"""
    timeline = build_timeline(log_path)
    payload = json.dumps(timeline, ensure_ascii=False,
                         separators=(",", ":")).replace("</", "<\\/")

    target = Path(out_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(_TEMPLATE.replace("__DATA__", payload),
                      encoding="utf-8")
    return str(target)
