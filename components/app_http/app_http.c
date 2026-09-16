/**
 * @file app_http.c
 * @brief HTTP 服务器实现：内嵌绘图界面与 REST 接口。
 * @details 负责下发绘图页面、查询播放状态、提交笔划以及中止播放。
 */

#include "app_http.h"
#include "app_pattern.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "http";

/** @brief POST 请求体允许的最大字节数（绘图界面自身的限制远低于此值）。 */
#define MAX_BODY_BYTES   65536

static httpd_handle_t s_server;

/**
 * @brief 内嵌的单页绘图界面（HTML + CSS + JS）。
 * @details 支持鼠标与触摸输入的 Canvas 绘图，提供速度 / 加速度滑块；
 *          通过 POST /api/points 发送笔划，通过 POST /api/abort 中止。
 */
static const char INDEX_HTML[] = R"raw(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Delta Robot Draw</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;}
body{
  font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;
  background: linear-gradient(135deg, #0f0c29, #302b63, #24243e);
  color:#eee;display:flex;flex-direction:column;align-items:center;
  min-height:100vh;padding:15px;
}
h1{
  font-size:1.6em;margin:10px 0 5px;
  background: linear-gradient(90deg, #e94560, #f39c12);
  -webkit-background-clip:text;-webkit-text-fill-color:transparent;
  background-clip:text;letter-spacing:1px;
}
.card{
  background: rgba(22,33,62,0.85);backdrop-filter:blur(10px);
  border:1px solid rgba(233,69,96,0.3);
  border-radius:16px;padding:15px;margin:6px 0;
  width:100%;max-width:440px;
  box-shadow:0 8px 32px rgba(0,0,0,0.4), inset 0 0 0 1px rgba(255,255,255,0.05);
}
canvas{
  display:block;margin:0 auto;
  border:2px solid rgba(233,69,96,0.4);
  border-radius:12px;background:#fff;
  touch-action:none;cursor:crosshair;
  width:100%;max-width:380px;
  box-shadow:0 0 20px rgba(233,69,96,0.2);
  transition:box-shadow 0.3s;
}
canvas:hover{box-shadow:0 0 30px rgba(233,69,96,0.4);}
.controls{
  display:flex;flex-wrap:wrap;gap:10px;align-items:center;
}
.controls label{
  font-size:0.9em;color:#aaa;min-width:70px;
  display:flex;align-items:center;gap:6px;
}
.controls input[type=range]{
  flex:1;min-width:100px;
  -webkit-appearance:none;appearance:none;
  height:6px;background:linear-gradient(90deg, #e94560, #f39c12);
  border-radius:3px;outline:none;
}
.controls input[type=range]::-webkit-slider-thumb{
  -webkit-appearance:none;width:20px;height:20px;
  background:#fff;border:2px solid #e94560;
  border-radius:50%;cursor:pointer;
  box-shadow:0 0 8px rgba(233,69,96,0.5);
}
.controls .val{
  min-width:45px;text-align:right;font-size:0.85em;
  background:linear-gradient(135deg, #e94560, #f39c12);
  -webkit-background-clip:text;-webkit-text-fill-color:transparent;
  background-clip:text;font-weight:bold;
}
.btn-row{display:flex;gap:10px;margin-top:10px;}
.btn{
  flex:1;padding:12px;border:none;border-radius:10px;
  font-size:1em;font-weight:bold;cursor:pointer;
  transition:all 0.3s;text-transform:uppercase;letter-spacing:1px;
  position:relative;overflow:hidden;
}
.btn::after{
  content:"";position:absolute;top:0;left:-100%;width:100%;height:100%;
  background:linear-gradient(120deg, transparent, rgba(255,255,255,0.2), transparent);
  transition:0.5s;
}
.btn:hover::after{left:100%;}
.btn:active{transform:scale(0.97);}
.btn-send{
  background:linear-gradient(135deg, #e94560, #c0392b);
  color:#fff;box-shadow:0 4px 15px rgba(233,69,96,0.4);
}
.btn-send:disabled{
  background:#555;color:#999;cursor:not-allowed;box-shadow:none;
}
.btn-clear{
  background:rgba(255,255,255,0.08);color:#ccc;
  border:1px solid rgba(255,255,255,0.2);
}
.btn-abort{
  background:linear-gradient(135deg, #f39c12, #e67e22);
  color:#000;box-shadow:0 4px 15px rgba(243,156,18,0.4);
}
#status{
  text-align:center;padding:10px;margin-top:10px;
  border-radius:10px;font-size:0.95em;font-weight:bold;
  transition:background 0.4s, color 0.4s;
  letter-spacing:0.5px;
}
.status-idle{background:rgba(39,174,96,0.2);color:#2ecc71;border:1px solid #27ae60;}
.status-drawing{background:rgba(41,128,185,0.2);color:#3498db;border:1px solid #2980b9;}
.status-sending{background:rgba(243,156,18,0.2);color:#f1c40f;border:1px solid #f39c12;}
.status-playing{background:rgba(142,68,173,0.2);color:#9b59b6;border:1px solid #8e44ad;}
.status-error{background:rgba(231,76,60,0.2);color:#e74c3c;border:1px solid #c0392b;}
.coord-info{
  font-size:0.75em;color:#aaa;text-align:center;margin-top:6px;
  opacity:0.8;letter-spacing:0.5px;
}
.stroke-indicator{
  display:flex;justify-content:center;gap:8px;margin-top:8px;flex-wrap:wrap;
}
.stroke-dot{
  width:8px;height:8px;border-radius:50%;background:#e94560;
  opacity:0.6;transition:all 0.3s;
}
.stroke-dot.active{opacity:1;background:#f39c12;box-shadow:0 0 8px #f39c12;}
</style>
</head>
<body>

<h1>&#129302; Delta Robot &middot; 绘图</h1>

<div class="card">
  <canvas id="cv" width="380" height="380"></canvas>
  <div class="coord-info">工作空间：&#x3A6;200mm 圆 | 圆心 = Delta(0,0) | 绘制 Z=-248 | 抬笔 Z=-200</div>
  <div class="stroke-indicator" id="strokeDots"></div>
</div>

<div class="card">
  <div class="controls">
    <label>&#x26A1; 速度 (0-100)</label>
    <input type="range" id="speedVal" min="0" max="100" value="50" step="1">
    <span class="val" id="speedDisp">50</span>
  </div>
  <div class="controls">
    <label>&#x1F4A8; 加速度 (0-20)</label>
    <input type="range" id="accelVal" min="0" max="20" value="5" step="1">
    <span class="val" id="accelDisp">5</span>
  </div>
</div>

<div class="card">
  <div class="btn-row">
    <button class="btn btn-clear" onclick="clearCanvas()">&#x1F5D1; 清除</button>
    <button class="btn btn-send" id="btnSend" onclick="sendPattern()">&#x27A1; 发送图案</button>
  </div>
  <div class="btn-row" style="margin-top:6px;">
    <button class="btn btn-abort" onclick="abortPattern()">&#x23F9; 中止</button>
  </div>
  <div id="status" class="status-idle">&#x2705; 就绪 &mdash; 在圆内绘制（支持多笔划）</div>
</div>

<script>
// ==================== Canvas 初始化 ====================
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
const W = cv.width, H = cv.height;
const CX = W/2, CY = H/2;
const RADIUS = 175;
const SCALE_MM = 200.0;
const SAMPLE_DIST = 4;

let isDrawing = false;
let strokes = [];            // strokes[strokeIdx] = [{cx,cy}, ...]
let currentStrokeIdx = -1;   // 当前正在绘制的笔划索引
let lastSampleX = null, lastSampleY = null;

// 笔划颜色表
const STROKE_COLORS = [
  '#e94560','#f39c12','#3498db','#2ecc71','#9b59b6',
  '#1abc9c','#e74c3c','#2980b9','#f1c40f','#e67e22'
];

function initCanvas(){
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#fff';
  ctx.fillRect(0,0,W,H);

  // 网格
  ctx.strokeStyle='#e8e8e8';
  ctx.lineWidth=0.5;
  for(let i=10;i<W;i+=10){
    ctx.beginPath();ctx.moveTo(i,0);ctx.lineTo(i,H);ctx.stroke();
    ctx.beginPath();ctx.moveTo(0,i);ctx.lineTo(W,i);ctx.stroke();
  }

  // 坐标轴
  ctx.strokeStyle='#777';
  ctx.lineWidth=1.2;
  ctx.beginPath();ctx.moveTo(10,CY);ctx.lineTo(W-10,CY);ctx.stroke();
  ctx.beginPath();ctx.moveTo(CX,10);ctx.lineTo(CX,H-10);ctx.stroke();

  // 箭头
  ctx.fillStyle='#777';
  ctx.beginPath();ctx.moveTo(CX+RADIUS+3,CY);ctx.lineTo(CX+RADIUS-5,CY-5);ctx.lineTo(CX+RADIUS-5,CY+5);ctx.fill();
  ctx.beginPath();ctx.moveTo(CX,CY-RADIUS-3);ctx.lineTo(CX-5,CY-RADIUS+5);ctx.lineTo(CX+5,CY-RADIUS+5);ctx.fill();
  ctx.font='bold 13px "Segoe UI", sans-serif';
  ctx.fillStyle='#444';
  ctx.fillText('X', CX+RADIUS-5, CY-8);
  ctx.fillText('Y', CX+10, CY-RADIUS+15);

  // 工作空间圆
  const grad = ctx.createLinearGradient(0,0,W,H);
  grad.addColorStop(0,'#e94560');
  grad.addColorStop(0.5,'#f39c12');
  grad.addColorStop(1,'#e94560');
  ctx.strokeStyle=grad;
  ctx.lineWidth=2.5;
  ctx.beginPath();
  ctx.arc(CX, CY, RADIUS, 0, Math.PI*2);
  ctx.stroke();

  // 原点
  ctx.fillStyle='#e94560';
  ctx.beginPath();
  ctx.arc(CX, CY, 5, 0, Math.PI*2);
  ctx.fill();
  ctx.fillStyle='#000';
  ctx.font='bold 14px sans-serif';
  ctx.fillText('O', CX+8, CY-8);

  // 重绘已有笔划
  redrawAllStrokes();
}

function redrawAllStrokes(){
  for(let s=0; s<strokes.length; s++){
    const pts = strokes[s];
    const color = STROKE_COLORS[s % STROKE_COLORS.length];
    if(pts.length < 1) continue;
    // 起点圆点
    ctx.fillStyle=color;
    ctx.beginPath();ctx.arc(pts[0].cx, pts[0].cy, 3, 0, Math.PI*2);ctx.fill();
    // 连线
    ctx.strokeStyle=color;
    ctx.lineWidth=2;
    ctx.lineCap='round';ctx.lineJoin='round';
    ctx.beginPath();
    ctx.moveTo(pts[0].cx, pts[0].cy);
    for(let i=1;i<pts.length;i++){
      ctx.lineTo(pts[i].cx, pts[i].cy);
    }
    ctx.stroke();
    // 终点圆点
    if(pts.length>1){
      ctx.fillStyle=color;
      ctx.beginPath();
      ctx.arc(pts[pts.length-1].cx, pts[pts.length-1].cy, 3, 0, Math.PI*2);
      ctx.fill();
    }
  }
}

function isInsideCircle(cx, cy){
  const dx = cx - CX, dy = cy - CY;
  return dx*dx + dy*dy <= RADIUS*RADIUS;
}

function getPos(e){
  const rect = cv.getBoundingClientRect();
  const sx = W / rect.width, sy = H / rect.height;
  if(e.touches){
    return {x:(e.touches[0].clientX-rect.left)*sx, y:(e.touches[0].clientY-rect.top)*sy};
  }
  return {x:(e.clientX-rect.left)*sx, y:(e.clientY-rect.top)*sy};
}

function updateStrokeDots(){
  const container = document.getElementById('strokeDots');
  container.innerHTML = '';
  for(let i=0;i<strokes.length;i++){
    const dot = document.createElement('span');
    dot.className = 'stroke-dot';
    if(i===currentStrokeIdx) dot.classList.add('active');
    dot.title = '笔划'+(i+1)+' ('+strokes[i].length+'点)';
    container.appendChild(dot);
  }
}

// 鼠标事件
cv.addEventListener('mousedown', e => {
  e.preventDefault();
  const p = getPos(e);
  if(!isInsideCircle(p.x, p.y)) return;
  isDrawing = true;
  // 创建新笔划
  strokes.push([{cx:p.x, cy:p.y}]);
  currentStrokeIdx = strokes.length - 1;
  lastSampleX = p.x; lastSampleY = p.y;
  updateStrokeDots();
  setStatus('drawing','&#x270F; 绘制中... 笔划 '+strokes.length);
});
cv.addEventListener('mousemove', e => {
  if(!isDrawing) return;
  e.preventDefault();
  const p = getPos(e);
  if(!isInsideCircle(p.x, p.y)) return;
  if(lastSampleX!==null){
    const dx = p.x - lastSampleX, dy = p.y - lastSampleY;
    if(Math.sqrt(dx*dx+dy*dy) < SAMPLE_DIST) return;
  }
  strokes[currentStrokeIdx].push({cx:p.x, cy:p.y});
  lastSampleX = p.x; lastSampleY = p.y;
  // 实时绘制当前笔划
  const pts = strokes[currentStrokeIdx];
  const color = STROKE_COLORS[currentStrokeIdx % STROKE_COLORS.length];
  ctx.strokeStyle=color;ctx.lineWidth=2;ctx.lineCap='round';ctx.lineJoin='round';
  ctx.beginPath();ctx.moveTo(pts[pts.length-2].cx, pts[pts.length-2].cy);
  ctx.lineTo(p.x, p.y);ctx.stroke();
  ctx.fillStyle=color;
  ctx.beginPath();ctx.arc(p.x,p.y,1.5,0,Math.PI*2);ctx.fill();
});
cv.addEventListener('mouseup', () => {
  if(!isDrawing) return;
  isDrawing = false;
  const totalPts = strokes.reduce((sum,s)=>sum+s.length, 0);
  setStatus('idle','&#x2705; 就绪 - '+strokes.length+' 笔划, 共 '+totalPts+' 点');
  lastSampleX=null;lastSampleY=null;
  currentStrokeIdx = -1;
  updateStrokeDots();
});
cv.addEventListener('mouseleave', () => {
  if(isDrawing){
    isDrawing = false;
    const totalPts = strokes.reduce((sum,s)=>sum+s.length, 0);
    setStatus('idle','&#x2705; 就绪 - '+strokes.length+' 笔划, 共 '+totalPts+' 点');
    lastSampleX=null;lastSampleY=null;
    currentStrokeIdx = -1;
    updateStrokeDots();
  }
});

// 触摸事件
cv.addEventListener('touchstart', e => {
  e.preventDefault();
  const p = getPos(e);
  if(!isInsideCircle(p.x, p.y)) return;
  isDrawing = true;
  strokes.push([{cx:p.x, cy:p.y}]);
  currentStrokeIdx = strokes.length - 1;
  lastSampleX = p.x; lastSampleY = p.y;
  updateStrokeDots();
  setStatus('drawing','&#x270F; 绘制中... 笔划 '+strokes.length);
},{passive:false});
cv.addEventListener('touchmove', e => {
  if(!isDrawing) return;
  e.preventDefault();
  const p = getPos(e);
  if(!isInsideCircle(p.x, p.y)) return;
  if(lastSampleX!==null){
    const dx = p.x - lastSampleX, dy = p.y - lastSampleY;
    if(Math.sqrt(dx*dx+dy*dy) < SAMPLE_DIST) return;
  }
  strokes[currentStrokeIdx].push({cx:p.x, cy:p.y});
  lastSampleX = p.x; lastSampleY = p.y;
  const pts = strokes[currentStrokeIdx];
  const color = STROKE_COLORS[currentStrokeIdx % STROKE_COLORS.length];
  ctx.strokeStyle=color;ctx.lineWidth=2;ctx.lineCap='round';ctx.lineJoin='round';
  ctx.beginPath();ctx.moveTo(pts[pts.length-2].cx, pts[pts.length-2].cy);
  ctx.lineTo(p.x, p.y);ctx.stroke();
},{passive:false});
cv.addEventListener('touchend', () => {
  if(!isDrawing) return;
  isDrawing = false;
  const totalPts = strokes.reduce((sum,s)=>sum+s.length, 0);
  setStatus('idle','&#x2705; 就绪 - '+strokes.length+' 笔划, 共 '+totalPts+' 点');
  lastSampleX=null;lastSampleY=null;
  currentStrokeIdx = -1;
  updateStrokeDots();
});

// ==================== 控件绑定 ====================
function bindSlider(id, dispId){
  const s = document.getElementById(id), d = document.getElementById(dispId);
  s.addEventListener('input', ()=>{ d.textContent = s.value; });
}
bindSlider('speedVal','speedDisp');
bindSlider('accelVal','accelDisp');

// ==================== 坐标映射 ====================
function pixelToMM(cx, cy){
  const x = (cx - CX) * (SCALE_MM / RADIUS);
  const y = -(cy - CY) * (SCALE_MM / RADIUS);
  return {x, y};
}

// ==================== 发送图案 ====================
async function sendPattern(){
  const totalPts = strokes.reduce((sum,s)=>sum+s.length, 0);
  if(totalPts < 2){
    setStatus('error','&#x26A0; 至少需要2个点');
    return;
  }
  const speed = parseInt(document.getElementById('speedVal').value);
  const accel = parseInt(document.getElementById('accelVal').value);
  const btn = document.getElementById('btnSend');
  btn.disabled = true;
  setStatus('sending','&#x1F4E4; 发送中 ('+strokes.length+'笔划, '+totalPts+'点) ...');

  // 转成三维数组: [[[x,y],[x,y],...], [[x,y],...]]
  const strokeData = strokes.map(stroke => {
    return stroke.map(p => {
      const m = pixelToMM(p.cx, p.cy);
      return [Math.round(m.x*10)/10, Math.round(m.y*10)/10];
    });
  });

  const payload = {
    strokes: strokeData,
    speed: speed,
    accel: accel
  };

  try{
    const resp = await fetch('/api/points', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify(payload)
    });
    const result = await resp.json();
    if(result.success){
      setStatus('playing','&#x25B6; 播放中 - '+result.point_count+'点, '+result.stroke_count+'笔划');
    } else {
      setStatus('error','&#x274C; 错误: '+(result.error||'未知'));
    }
  } catch(e){
    setStatus('error','&#x274C; 网络错误: '+e.message);
  }
  btn.disabled = false;
}

function abortPattern(){
  fetch('/api/abort', {method:'POST'})
    .then(r=>r.json())
    .then(r=>{ setStatus('idle','&#x23F9; 已中止'); })
    .catch(e=>setStatus('error','中止失败'));
}

function clearCanvas(){
  strokes=[];
  lastSampleX=null;lastSampleY=null;
  currentStrokeIdx=-1;
  initCanvas();
  updateStrokeDots();
  setStatus('idle','&#x2705; 就绪 - 已清除');
}

function setStatus(type, msg){
  const el = document.getElementById('status');
  el.className = 'status-'+type;
  el.innerHTML = msg;
}

initCanvas();
</script>
</body>
</html>
)raw";

/* ============================================================ 请求处理器 */

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

/** @brief GET / 路由：返回内嵌的绘图页面。 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

/** @brief GET /status 路由：返回播放状态。 */
static esp_err_t status_get_handler(httpd_req_t *req)
{
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"idle\":%s,\"progress\":%u}",
             pattern_player_is_idle() ? "true" : "false",
             pattern_player_progress());
    return send_json(req, resp);
}

/** @brief POST /api/abort 路由：中止当前播放。 */
static esp_err_t abort_post_handler(httpd_req_t *req)
{
    pattern_player_abort();
    return send_json(req, "{\"success\":true}");
}

/**
 * @brief POST /api/points 路由：解析笔划，将其展平为点列表后交给图案播放器。
 * @note 用 NaN 标记笔划之间的抬笔。
 */
static esp_err_t points_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > MAX_BODY_BYTES) {
        return send_json(req, "{\"success\":false,\"error\":\"invalid payload size\"}");
    }

    char *body = malloc(total_len + 1);
    if (!body) {
        return send_json(req, "{\"success\":false,\"error\":\"out of memory\"}");
    }

    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, body + received, total_len - received);
        if (ret <= 0) {
            free(body);
            return send_json(req, "{\"success\":false,\"error\":\"receive failed\"}");
        }
        received += ret;
    }
    body[received] = '\0';

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        return send_json(req, "{\"success\":false,\"error\":\"invalid JSON\"}");
    }

    /* ---- 可选参数及其服务端默认值 ---- */
    float    z_mm    = DRAW_Z_DEFAULT;
    uint32_t speed   = 50;
    uint8_t  accel   = 5;
    uint32_t timeout = 3000;

    cJSON *speed_item = cJSON_GetObjectItem(root, "speed");
    if (cJSON_IsNumber(speed_item)) {
        speed = (uint32_t)speed_item->valueint;
    }
    cJSON *accel_item = cJSON_GetObjectItem(root, "accel");
    if (cJSON_IsNumber(accel_item)) {
        accel = (uint8_t)accel_item->valueint;
    }

    /* ---- 将笔划展平为单个点列表 ---- */
    pattern_point_t *points = malloc(PATTERN_MAX_POINTS * sizeof(*points));
    if (!points) {
        cJSON_Delete(root);
        return send_json(req, "{\"success\":false,\"error\":\"out of memory\"}");
    }

    uint16_t count = 0;
    cJSON *strokes = cJSON_GetObjectItem(root, "strokes");
    if (cJSON_IsArray(strokes)) {
        int num_strokes = cJSON_GetArraySize(strokes);

        for (int s = 0; s < num_strokes && count < PATTERN_MAX_POINTS - 1; s++) {
            cJSON *stroke = cJSON_GetArrayItem(strokes, s);
            if (!cJSON_IsArray(stroke)) {
                continue;
            }

            int pt_count = cJSON_GetArraySize(stroke);
            int added = 0;
            for (int i = 0; i < pt_count && count < PATTERN_MAX_POINTS - 1; i++) {
                cJSON *pt = cJSON_GetArrayItem(stroke, i);
                if (!cJSON_IsArray(pt) || cJSON_GetArraySize(pt) < 2) {
                    continue;
                }
                cJSON *x = cJSON_GetArrayItem(pt, 0);
                cJSON *y = cJSON_GetArrayItem(pt, 1);
                if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y)) {
                    continue;
                }
                points[count].x = (float)x->valuedouble;
                points[count].y = (float)y->valuedouble;
                count++;
                added++;
            }

            /* 在包含真实几何图形的笔划之间插入抬笔标记。 */
            if (added >= 2 && s < num_strokes - 1 && count < PATTERN_MAX_POINTS - 1) {
                points[count].x = NAN;
                points[count].y = NAN;
                count++;
            }
        }
    }
    cJSON_Delete(root);

    if (count < 2) {
        free(points);
        return send_json(req, "{\"success\":false,\"error\":\"not enough valid points\"}");
    }

    int stroke_count = 0;
    bool in_stroke = false;
    for (uint16_t i = 0; i < count; i++) {
        if (isnan(points[i].x)) {
            in_stroke = false;
        } else if (!in_stroke) {
            stroke_count++;
            in_stroke = true;
        }
    }

    ESP_LOGI(TAG, "Received %u points in %d strokes (Z=%.1f, speed=%lu, accel=%u)",
             count, stroke_count, (double)z_mm, (unsigned long)speed, accel);

    esp_err_t ret = pattern_player_load(points, count, z_mm, speed, accel, timeout);
    free(points);

    if (ret != ESP_OK) {
        return send_json(req, "{\"success\":false,\"error\":\"player busy\"}");
    }

    char resp[128];
    snprintf(resp, sizeof(resp),
             "{\"success\":true,\"point_count\":%u,\"stroke_count\":%d}",
             count, stroke_count);
    return send_json(req, resp);
}

/* ============================================================= 生命周期 */

esp_err_t http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers  = 8;
    config.stack_size        = 12288;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }

    const httpd_uri_t routes[] = {
        { .uri = "/",           .method = HTTP_GET,  .handler = root_get_handler   },
        { .uri = "/status",     .method = HTTP_GET,  .handler = status_get_handler },
        { .uri = "/api/points", .method = HTTP_POST, .handler = points_post_handler },
        { .uri = "/api/abort",  .method = HTTP_POST, .handler = abort_post_handler },
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        esp_err_t ret = httpd_register_uri_handler(s_server, &routes[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register %s", routes[i].uri);
        }
    }

    ESP_LOGI(TAG, "HTTP server listening on port %d", config.server_port);
    return ESP_OK;
}
