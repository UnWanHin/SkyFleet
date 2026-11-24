"""Self-contained WebGL2 3D replay from a SkyFleet event log."""
from __future__ import annotations

import json
from pathlib import Path

from skyfleet.observe.log import read_log
from skyfleet.observe.replay import build_timeline


def build_3d_timeline(log_path: str | Path) -> dict:
    data = build_timeline(log_path)
    events = list(read_log(log_path))
    # Correlate each RX to the most recent eligible TX of this hop, rather
    # than coloring every retry of the same message as successfully received.
    pending: dict[tuple[str, str], list[int]] = {}
    radio: list[dict] = []
    for event in events:
        kind = event["kind"]
        if kind == "MSG_TX":
            src, dst = event.get("node_id"), event.get("next_hop")
            if src not in data["nodes"] or dst not in data["nodes"]:
                continue
            entry = {"t": event["t"], "src": src, "next_hop": dst,
                     "received": False}
            pending.setdefault((event.get("msg_id"), dst), []).append(len(radio))
            radio.append(entry)
        elif kind in ("MSG_RX", "MSG_FORWARD"):
            key = (event.get("msg_id"), event.get("node_id"))
            candidates = pending.get(key, [])
            if candidates:
                index = candidates[-1]
                if radio[index]["t"] <= event["t"]:
                    radio[index]["received"] = True
                    candidates.clear()
    data["radio"] = radio
    return data


def render_replay_3d(log_path: str | Path, out_path: str | Path) -> str:
    data = build_3d_timeline(log_path)
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    payload = payload.replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")
    target = Path(out_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(_HTML.replace("__DATA__", payload), encoding="utf-8")
    return str(target)


_HTML = r'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SkyFleet 3D 城市通信回放</title><style>
body{margin:0;background:#101b2b;color:#e8f1fb;font:14px system-ui}header{padding:15px 20px;background:#182a41}h1{margin:0 0 4px;font-size:19px}
small{color:#a9bfce}main{display:grid;grid-template-columns:minmax(0,1fr) 280px;gap:12px;padding:12px}section,aside{background:#182a41;border-radius:9px;overflow:hidden}
#stage{width:100%;height:min(74vh,820px);min-height:420px;display:block;touch-action:none}.controls{padding:12px;display:flex;align-items:center;gap:9px;flex-wrap:wrap}
input[type=range]{flex:1;min-width:160px}button,select{background:#28435d;border:1px solid #50687e;border-radius:5px;color:white;padding:5px 9px;cursor:pointer}
#readout{padding:14px;line-height:1.8;white-space:pre-line;overflow-wrap:anywhere}.legend{padding:8px 14px;color:#b7c9d5}
@media(max-width:800px){main{grid-template-columns:1fr}#stage{height:55vh}}
</style></head><body><header><h1>SkyFleet · 三维城市通信回放</h1>
<small>WebGL2 离线单文件 · 拖动旋转 / 滚轮缩放 · 从日志重建飞行，不是实时 MuJoCo 画面</small></header>
<main><section><canvas id="stage"></canvas><div class="controls"><button id="play">播放</button><button id="reset">重置</button>
<input id="scrub" type="range" min="0" max="1000" value="0"><span id="clock">0.00 s</span><select id="speed"><option value="1">1×</option><option value="4" selected>4×</option><option value="16">16×</option></select></div>
<div class="legend">红=主节点　青=无人机　绿=中继　彩色=分组　橙=收到的封包　红线=发送未收到　灰蓝=建筑</div></section><aside id="readout"></aside></main>
<script>
const DATA=__DATA__,canvas=document.getElementById("stage"),gl=canvas.getContext("webgl2",{antialias:true});
const info=document.getElementById("readout"),clock=document.getElementById("clock"),scrub=document.getElementById("scrub");
if(!gl){info.textContent="需要 WebGL2 浏览器与驱动。";throw Error("WebGL2 unavailable")}
const vs=`#version 300 es
in vec3 aPos;in vec3 aColor;out vec3 vColor;uniform mat4 uView;
void main(){gl_Position=uView*vec4(aPos,1.0);vColor=aColor;}`;
const fs=`#version 300 es
precision mediump float;in vec3 vColor;out vec4 pixel;
void main(){pixel=vec4(vColor,1.0);}`;
function compile(type,code){let s=gl.createShader(type);gl.shaderSource(s,code);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s}
const program=gl.createProgram();gl.attachShader(program,compile(gl.VERTEX_SHADER,vs));gl.attachShader(program,compile(gl.FRAGMENT_SHADER,fs));gl.linkProgram(program);
if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));gl.useProgram(program);
const buffer=gl.createBuffer(),locP=gl.getAttribLocation(program,"aPos"),locC=gl.getAttribLocation(program,"aColor"),locV=gl.getUniformLocation(program,"uView");
gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.enableVertexAttribArray(locP);gl.vertexAttribPointer(locP,3,gl.FLOAT,false,24,0);
gl.enableVertexAttribArray(locC);gl.vertexAttribPointer(locC,3,gl.FLOAT,false,24,12);gl.enable(gl.DEPTH_TEST);
let t=0,playing=false,last=0,yaw=-.65,pitch=.68,zoom=1,drag=null;
const duration=Math.max(DATA.duration||1,1),nodes=DATA.nodes,terrain=DATA.terrain||{};
const nearby=DATA.buildings.length?Object.keys(nodes).filter(id=>{
 const first=nodes[id].samples[0];if(!first)return false;
 return DATA.buildings.some(([x,y,w,d])=>{
  const dx=Math.max(x-first[1],0,first[1]-x-w),dy=Math.max(y-first[2],0,first[2]-y-d);
  return Math.hypot(dx,dy)<Math.max(150,DATA.area*1.5)
 })}):Object.keys(nodes);
const visibleIds=nearby.length?nearby:Object.keys(nodes);
function sample(samples,time){if(!samples||!samples.length)return null;let i=0;while(i+1<samples.length&&samples[i+1][0]<=time)i++;
const a=samples[i],b=samples[i+1];if(!b)return a.slice(1);const k=Math.max(0,Math.min(1,(time-a[0])/(b[0]-a[0]||1)));
return [1,2,3].map(j=>a[j]+(b[j]-a[j])*k)}
function color(id){const g=DATA.groups&&DATA.groups[id];if(g&&g!=="main"){let h=0;for(const ch of g)h=(h*31+ch.charCodeAt(0))%360;const s=.72,v=.95,c=v*s,x=c*(1-Math.abs((h/60)%2-1)),m=v-c;let q=h<60?[c,x,0]:h<120?[x,c,0]:h<180?[0,c,x]:h<240?[0,x,c]:h<300?[x,0,c]:[c,0,x];return q.map(a=>a+m)}return id===DATA.leader_id?[.98,.27,.29]:nodes[id].role==="RELAY"?[.29,.89,.55]:[.26,.8,1]}
function push(a,p,c){a.push(...p,...c)}
function cube(a,x,y,z,w,d,h,c){let p=[[x,y,z],[x+w,y,z],[x+w,y+d,z],[x,y+d,z],[x,y,z+h],[x+w,y,z+h],[x+w,y+d,z+h],[x,y+d,z+h]];
for(const [face,shade] of [[[0,3,2,1],.36],[[4,5,6,7],1],[[0,1,5,4],.65],[[1,2,6,5],.75],[[2,3,7,6],.53],[[3,0,4,7],.62]]){
const q=face.map(i=>p[i]),tone=c.map(v=>v*shade);for(const i of [0,1,2,0,2,3])push(a,q[i],tone)}}
function line(a,p,q,c){push(a,p,c);push(a,q,c)}
function terrainHeight(x,y){if(!terrain.hill)return 0;
 const [hx,hy,peak,radius]=terrain.hill,d=Math.hypot(x-hx,y-hy)/radius;
 return peak*Math.max(0,1-d)**2}
function tile(a,x,y,w,d,z,c){const p=[[x,y,z],[x+w,y,z],[x+w,y+d,z],[x,y+d,z]];
 for(const i of [0,1,2,0,2,3])push(a,p[i],c)}
function ground(a){if(!terrain.bounds)return;
 const [minX,minY,maxX,maxY]=terrain.bounds,step=10;
 for(let x=minX;x<maxX;x+=step)for(let y=minY;y<maxY;y+=step){
  const x1=Math.min(maxX,x+step),y1=Math.min(maxY,y+step);
  const corners=[[x,y],[x1,y],[x1,y1],[x,y1]].map(([px,py])=>[px,py,terrainHeight(px,py)-1]);
  const color=terrainHeight((x+x1)/2,(y+y1)/2)>1?[.22,.38,.31]:[.19,.28,.35];
  for(const k of [0,1,2,0,2,3])push(a,corners[k],color)
 }
 for(const road of terrain.roads||[])tile(a,...road,.12,[.19,.21,.26]);
 if(terrain.park)tile(a,...terrain.park,.13,[.17,.44,.25])}
// Camera: metric world with z-up, perspective and orbit controls.
function viewMatrix(){let bounds=[];for(const id of visibleIds)for(const s of nodes[id].samples)bounds.push(s.slice(1));
for(const [x,y,w,d,h] of DATA.buildings)bounds.push([x,y,0],[x+w,y+d,h]);
if(terrain.bounds){const [x0,y0,x1,y1]=terrain.bounds;bounds.push([x0,y0,0],[x1,y1,terrain.hill?terrain.hill[2]:0])}
let lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];for(const p of bounds)for(let j=0;j<3;j++){lo[j]=Math.min(lo[j],p[j]);hi[j]=Math.max(hi[j],p[j])}
if(!bounds.length){lo=[0,0,0];hi=[100,100,100]}let center=lo.map((v,j)=>(v+hi[j])/2),span=Math.max(...hi.map((v,j)=>v-lo[j]),40);
let cp=Math.cos(pitch),sp=Math.sin(pitch),cy=Math.cos(yaw),sy=Math.sin(yaw),dir=[cp*cy,cp*sy,sp],eye=center.map((v,j)=>v+dir[j]*span*2.3*zoom);
let fwd=center.map((v,j)=>v-eye[j]),norm=Math.hypot(...fwd);fwd=fwd.map(v=>v/norm);let right=[-fwd[1],fwd[0],0];norm=Math.hypot(...right);right=right.map(v=>v/norm);
let up=[-right[1]*fwd[2],right[0]*fwd[2],right[1]*fwd[0]-right[0]*fwd[1]];
let f=1/Math.tan(Math.PI/6),aspect=canvas.width/canvas.height,near=.1,far=span*30*zoom+100;
let v=[right[0],up[0],-fwd[0],0,right[1],up[1],-fwd[1],0,right[2],up[2],-fwd[2],0,
-right.reduce((a,b,j)=>a+b*eye[j],0),-up.reduce((a,b,j)=>a+b*eye[j],0),fwd.reduce((a,b,j)=>a+b*eye[j],0),1];
let proj=[f/aspect,0,0,0,0,f,0,0,0,0,(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0],out=new Float32Array(16);
for(let col=0;col<4;col++)for(let row=0;row<4;row++)for(let k=0;k<4;k++)out[col*4+row]+=proj[k*4+row]*v[col*4+k];return out}
function render(){let width=Math.max(1,Math.floor(canvas.clientWidth*devicePixelRatio)),height=Math.max(1,Math.floor(canvas.clientHeight*devicePixelRatio));
if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height}gl.viewport(0,0,width,height);gl.clearColor(.055,.12,.2,1);
gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.uniformMatrix4fv(locV,false,viewMatrix());let triangles=[],lines=[];
ground(triangles);
for(const [x,y,w,d,h,base=0] of DATA.buildings)cube(triangles,x,y,base,w,d,h-base,[.39,.56,.68]);
for(let g=-100;g<=Math.max(DATA.area,150)+100;g+=50){line(lines,[g,-100,0],[g,Math.max(DATA.area,150)+100,0],[.18,.35,.47]);line(lines,[-100,g,0],[Math.max(DATA.area,150)+100,g,0],[.18,.35,.47])}
for(const id of visibleIds){const n=nodes[id],p=sample(n.samples,t);if(!p)continue;let c=color(id),s=id===DATA.leader_id?3.5:2.8;
cube(triangles,p[0]-s/2,p[1]-s/2,p[2]-s/2,s,s,s,c);line(lines,[p[0],p[1],0],p,c.map(v=>v*.55));
for(let i=1;i<n.samples.length;i++){if(n.samples[i-1][0]>t)break;let end=n.samples[i][0]<=t?n.samples[i].slice(1):sample(n.samples,t);line(lines,n.samples[i-1].slice(1),end,c.map(v=>v*.45))}}
let active=DATA.radio.filter(e=>Math.abs(e.t-t)<.3&&visibleIds.includes(e.src)&&visibleIds.includes(e.next_hop)).slice(-30);
for(const e of active){let a=sample(nodes[e.src].samples,e.t),b=sample(nodes[e.next_hop].samples,e.t);if(a&&b)line(lines,a,b,e.received?[1,.68,.24]:[1,.19,.19])}
function draw(a,mode){gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(a),gl.DYNAMIC_DRAW);gl.drawArrays(mode,0,a.length/6)}draw(triangles,gl.TRIANGLES);draw(lines,gl.LINES);
clock.textContent=t.toFixed(2)+" s";scrub.value=String(Math.round(t/duration*1000));
let recent=DATA.events.filter(e=>e[0]<=t).slice(-6).reverse();info.textContent=`建筑 ${DATA.buildings.length}　显示节点 ${visibleIds.length}/${Object.keys(nodes).length}\n时刻 ${t.toFixed(2)} s / ${duration} s\n当前无线链路 ${active.length}\n\n事件\n`+recent.map(e=>`${e[0].toFixed(2)}s ${e[1]} ${e[2]} ${e[3]}`).join("\n")}
function frame(now){if(!playing)return;let dt=last?(now-last)/1000:0;last=now;t=Math.min(duration,t+dt*Number(document.getElementById("speed").value));render();if(t>=duration){playing=false;document.getElementById("play").textContent="播放"}else requestAnimationFrame(frame)}
document.getElementById("play").onclick=()=>{playing=!playing;last=0;if(playing&&t>=duration)t=0;document.getElementById("play").textContent=playing?"暂停":"播放";if(playing)requestAnimationFrame(frame)};
document.getElementById("reset").onclick=()=>{playing=false;t=0;render();document.getElementById("play").textContent="播放"};
scrub.oninput=()=>{playing=false;t=Number(scrub.value)*duration/1000;render();document.getElementById("play").textContent="播放"};
canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};
canvas.onpointermove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(.12,Math.min(1.5,pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];render()};
canvas.onpointerup=()=>drag=null;canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.15,Math.min(12,zoom*Math.exp(e.deltaY*.001)));render()};window.onresize=render;render();
</script></body></html>'''
