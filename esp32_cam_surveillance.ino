/**
 * ESP32-CAM Surveillance Camera
 * ================================
 * Serves a live MJPEG stream and web dashboard over local WiFi.
 * 
 * Board:   AI Thinker ESP32-CAM
 * Flash:   4MB, Huge APP (3MB No OTA) partition scheme
 * 
 * Setup:
 *   1. Edit WIFI_SSID and WIFI_PASS below
 *   2. Select board: "AI Thinker ESP32-CAM" in Arduino IDE
 *   3. Set partition scheme to "Huge APP (3MB No OTA / 1MB SPIFFS)"
 *   4. Upload, then open Serial Monitor at 115200 baud to see IP address
 */

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

// ─── WiFi Credentials ────────────────────────────────────────────────────────
#define WIFI_SSID   "YOUR_WIFI_SSID"
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"
// ─────────────────────────────────────────────────────────────────────────────

// ─── AI Thinker ESP32-CAM Pin Map ────────────────────────────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_GPIO_NUM       4   // Built-in flash LED
// ─────────────────────────────────────────────────────────────────────────────

// Global state
static bool flashOn = false;
static int  frameCount = 0;
static unsigned long startTime = 0;

// ─── Camera Init ─────────────────────────────────────────────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel  = LEDC_CHANNEL_0;
  config.ledc_timer    = LEDC_TIMER_0;
  config.pin_d0        = Y2_GPIO_NUM;
  config.pin_d1        = Y3_GPIO_NUM;
  config.pin_d2        = Y4_GPIO_NUM;
  config.pin_d3        = Y5_GPIO_NUM;
  config.pin_d4        = Y6_GPIO_NUM;
  config.pin_d5        = Y7_GPIO_NUM;
  config.pin_d6        = Y8_GPIO_NUM;
  config.pin_d7        = Y9_GPIO_NUM;
  config.pin_xclk      = XCLK_GPIO_NUM;
  config.pin_pclk      = PCLK_GPIO_NUM;
  config.pin_vsync     = VSYNC_GPIO_NUM;
  config.pin_href      = HREF_GPIO_NUM;
  config.pin_sscb_sda  = SIOD_GPIO_NUM;
  config.pin_sscb_scl  = SIOC_GPIO_NUM;
  config.pin_pwdn      = PWDN_GPIO_NUM;
  config.pin_reset     = RESET_GPIO_NUM;
  config.xclk_freq_hz  = 20000000;
  config.pixel_format  = PIXFORMAT_JPEG;
  config.grab_mode     = CAMERA_GRAB_LATEST;
  config.fb_location   = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;   // 640×480
    config.jpeg_quality = 12;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;  // 320×240 fallback
    config.jpeg_quality = 15;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed: 0x%x\n", err);
    return false;
  }

  // Sensor tweaks for better image
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);       // Auto white balance
    s->set_exposure_ctrl(s, 1); // Auto exposure
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
  }

  Serial.println("[CAM] Initialized OK");
  return true;
}

// ─── MJPEG Stream Handler ─────────────────────────────────────────────────────
#define PART_BOUNDARY "framebound"
static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char partBuf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  // Disable Nagle buffering for lower latency
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate",                 "15");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[STREAM] Frame capture failed");
      res = ESP_FAIL;
      break;
    }

    // Send boundary
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    // Send part header
    size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, partBuf, hlen);
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    // Send JPEG payload
    res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;

    frameCount++;
  }
  return res;
}

// ─── Snapshot Handler (single JPEG) ──────────────────────────────────────────
static esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ─── Flash Toggle Handler ─────────────────────────────────────────────────────
static esp_err_t flashHandler(httpd_req_t *req) {
  flashOn = !flashOn;
  digitalWrite(LED_GPIO_NUM, flashOn ? HIGH : LOW);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  const char* state = flashOn ? "{\"flash\":true}" : "{\"flash\":false}";
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, state);
}

// ─── Status/Info Handler ──────────────────────────────────────────────────────
static esp_err_t statusHandler(httpd_req_t *req) {
  unsigned long upSec = (millis() - startTime) / 1000;
  float fps = (upSec > 0) ? (float)frameCount / (float)upSec : 0;

  char json[256];
  snprintf(json, sizeof(json),
    "{"
      "\"uptime\":%lu,"
      "\"frames\":%d,"
      "\"fps\":%.1f,"
      "\"flash\":%s,"
      "\"psram\":%s,"
      "\"heap\":%lu"
    "}",
    upSec, frameCount, fps,
    flashOn ? "true" : "false",
    psramFound() ? "true" : "false",
    (unsigned long)ESP.getFreeHeap()
  );

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, json);
}

// ─── Camera Settings Handler (brightness / contrast / resolution) ─────────────
static esp_err_t settingsHandler(httpd_req_t *req) {
  char buf[128];
  int len = httpd_req_get_url_query_len(req) + 1;
  if (len > 1 && len <= (int)sizeof(buf)) {
    httpd_req_get_url_query_str(req, buf, len);
    // Parse individual params
    char val[8];
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
      if (httpd_query_key_value(buf, "brightness", val, sizeof(val)) == ESP_OK)
        s->set_brightness(s, atoi(val));
      if (httpd_query_key_value(buf, "contrast",   val, sizeof(val)) == ESP_OK)
        s->set_contrast(s, atoi(val));
      if (httpd_query_key_value(buf, "vflip",      val, sizeof(val)) == ESP_OK)
        s->set_vflip(s, atoi(val));
      if (httpd_query_key_value(buf, "hmirror",    val, sizeof(val)) == ESP_OK)
        s->set_hmirror(s, atoi(val));
    }
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// ─── Web Dashboard HTML ───────────────────────────────────────────────────────
// Served from PROGMEM to save heap
static const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32-CAM Surveillance</title>
<style>
  :root {
    --bg: #0d0f14;
    --surface: #161b24;
    --border: #252d3a;
    --accent: #00e5ff;
    --accent2: #ff4b6e;
    --text: #e2e8f0;
    --muted: #64748b;
    --green: #22d3a5;
    --radius: 10px;
    --font: 'Segoe UI', system-ui, sans-serif;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--font);
    min-height: 100vh;
  }

  /* ── Header ── */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 14px 24px;
    background: var(--surface);
    border-bottom: 1px solid var(--border);
  }
  .logo {
    display: flex;
    align-items: center;
    gap: 10px;
    font-size: 1.05rem;
    font-weight: 700;
    letter-spacing: 0.04em;
    color: var(--accent);
    text-transform: uppercase;
  }
  .logo svg { width: 22px; height: 22px; }

  .badge {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 0.78rem;
    color: var(--muted);
  }
  .dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: var(--accent2);
    animation: blink 1.4s ease-in-out infinite;
  }
  .dot.live { background: var(--green); }
  @keyframes blink {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.25; }
  }

  /* ── Layout ── */
  .main {
    display: grid;
    grid-template-columns: 1fr 300px;
    gap: 20px;
    padding: 20px 24px;
    max-width: 1200px;
    margin: 0 auto;
  }
  @media (max-width: 800px) {
    .main { grid-template-columns: 1fr; }
  }

  /* ── Feed ── */
  .feed-panel {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    overflow: hidden;
  }
  .feed-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 12px 16px;
    border-bottom: 1px solid var(--border);
    font-size: 0.82rem;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 0.06em;
  }
  #stream-img {
    display: block;
    width: 100%;
    aspect-ratio: 4/3;
    object-fit: cover;
    background: #000;
  }
  .feed-overlay {
    padding: 10px 16px;
    font-size: 0.78rem;
    color: var(--muted);
    display: flex;
    gap: 18px;
  }
  .feed-overlay span { color: var(--text); font-weight: 600; }

  /* ── Sidebar ── */
  .sidebar { display: flex; flex-direction: column; gap: 16px; }

  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 16px;
  }
  .card-title {
    font-size: 0.72rem;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    color: var(--muted);
    margin-bottom: 14px;
  }

  /* Stats grid */
  .stats {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
  }
  .stat {
    background: var(--bg);
    border-radius: 6px;
    padding: 10px 12px;
  }
  .stat-label {
    font-size: 0.7rem;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 0.05em;
    margin-bottom: 4px;
  }
  .stat-value {
    font-size: 1.1rem;
    font-weight: 700;
    color: var(--accent);
  }

  /* Controls */
  .controls { display: flex; flex-direction: column; gap: 10px; }
  
  .btn {
    width: 100%;
    padding: 10px 16px;
    border: 1px solid var(--border);
    border-radius: 6px;
    background: var(--bg);
    color: var(--text);
    font-family: var(--font);
    font-size: 0.85rem;
    font-weight: 600;
    cursor: pointer;
    transition: background 0.15s, border-color 0.15s, color 0.15s;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
  }
  .btn:hover { background: var(--surface); border-color: var(--accent); color: var(--accent); }
  .btn.active { background: rgba(0,229,255,0.1); border-color: var(--accent); color: var(--accent); }
  .btn.danger { border-color: #ff4b6e33; }
  .btn.danger:hover { background: rgba(255,75,110,0.1); border-color: var(--accent2); color: var(--accent2); }
  .btn.snapshot-btn { border-color: #22d3a533; }
  .btn.snapshot-btn:hover { background: rgba(34,211,165,0.1); border-color: var(--green); color: var(--green); }

  /* Sliders */
  .setting-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 12px;
    gap: 10px;
  }
  .setting-row label {
    font-size: 0.8rem;
    color: var(--muted);
    min-width: 80px;
  }
  .setting-row input[type=range] {
    flex: 1;
    accent-color: var(--accent);
    cursor: pointer;
    height: 4px;
  }
  .setting-val {
    font-size: 0.8rem;
    font-weight: 700;
    color: var(--text);
    min-width: 24px;
    text-align: right;
  }

  /* Toggle row */
  .toggle-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 12px;
  }
  .toggle-row label { font-size: 0.8rem; color: var(--muted); }
  .toggle {
    position: relative;
    width: 40px; height: 22px;
  }
  .toggle input { opacity: 0; width: 0; height: 0; }
  .toggle-slider {
    position: absolute;
    inset: 0;
    background: var(--border);
    border-radius: 22px;
    cursor: pointer;
    transition: background 0.2s;
  }
  .toggle-slider::before {
    content: '';
    position: absolute;
    width: 16px; height: 16px;
    left: 3px; top: 3px;
    background: var(--muted);
    border-radius: 50%;
    transition: transform 0.2s, background 0.2s;
  }
  .toggle input:checked + .toggle-slider { background: rgba(0,229,255,0.2); }
  .toggle input:checked + .toggle-slider::before {
    transform: translateX(18px);
    background: var(--accent);
  }

  /* Snapshot preview */
  #snapshot-preview {
    width: 100%;
    border-radius: 6px;
    display: none;
    margin-top: 10px;
    border: 1px solid var(--border);
  }
  #snap-time {
    font-size: 0.72rem;
    color: var(--muted);
    margin-top: 6px;
    display: none;
  }
</style>
</head>
<body>

<header>
  <div class="logo">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
      <path d="M15 10l4.553-2.069A1 1 0 0121 8.87v6.26a1 1 0 01-1.447.9L15 14M3 8a2 2 0 012-2h10a2 2 0 012 2v8a2 2 0 01-2 2H5a2 2 0 01-2-2V8z"/>
    </svg>
    ESP32-CAM
  </div>
  <div class="badge">
    <div class="dot live" id="status-dot"></div>
    <span id="status-text">Connecting…</span>
  </div>
</header>

<div class="main">

  <!-- Live Feed -->
  <div class="feed-panel">
    <div class="feed-header">
      <span>Live Feed</span>
      <span id="fps-badge">— fps</span>
    </div>
    <img id="stream-img" alt="Live stream">
    <div class="feed-overlay">
      Uptime: <span id="uptime-val">—</span> &nbsp;|&nbsp; Frames: <span id="frame-val">—</span> &nbsp;|&nbsp; Heap: <span id="heap-val">—</span>
    </div>
  </div>

  <!-- Sidebar -->
  <div class="sidebar">

    <!-- Stats -->
    <div class="card">
      <div class="card-title">System</div>
      <div class="stats">
        <div class="stat">
          <div class="stat-label">FPS</div>
          <div class="stat-value" id="s-fps">—</div>
        </div>
        <div class="stat">
          <div class="stat-label">Uptime</div>
          <div class="stat-value" id="s-uptime">—</div>
        </div>
        <div class="stat">
          <div class="stat-label">Frames</div>
          <div class="stat-value" id="s-frames">—</div>
        </div>
        <div class="stat">
          <div class="stat-label">Heap</div>
          <div class="stat-value" id="s-heap">—</div>
        </div>
      </div>
    </div>

    <!-- Controls -->
    <div class="card">
      <div class="card-title">Controls</div>
      <div class="controls">
        <button class="btn snapshot-btn" id="snap-btn" onclick="takeSnapshot()">
          📸 Take Snapshot
        </button>
        <button class="btn" id="flash-btn" onclick="toggleFlash()">
          💡 Flash: OFF
        </button>
        <button class="btn danger" onclick="toggleStream()">
          ⏸ Pause Stream
        </button>
      </div>
      <img id="snapshot-preview" alt="Snapshot">
      <div id="snap-time"></div>
    </div>

    <!-- Image Settings -->
    <div class="card">
      <div class="card-title">Image Settings</div>

      <div class="setting-row">
        <label>Brightness</label>
        <input type="range" min="-2" max="2" value="0" id="sl-brightness"
               oninput="document.getElementById('v-brightness').textContent=this.value; applySetting('brightness',this.value)">
        <span class="setting-val" id="v-brightness">0</span>
      </div>
      <div class="setting-row">
        <label>Contrast</label>
        <input type="range" min="-2" max="2" value="1" id="sl-contrast"
               oninput="document.getElementById('v-contrast').textContent=this.value; applySetting('contrast',this.value)">
        <span class="setting-val" id="v-contrast">1</span>
      </div>

      <div class="toggle-row">
        <label>Flip Vertical</label>
        <label class="toggle">
          <input type="checkbox" id="tgl-vflip" onchange="applySetting('vflip', this.checked?1:0)">
          <span class="toggle-slider"></span>
        </label>
      </div>
      <div class="toggle-row">
        <label>Mirror Horizontal</label>
        <label class="toggle">
          <input type="checkbox" id="tgl-hmirror" onchange="applySetting('hmirror', this.checked?1:0)">
          <span class="toggle-slider"></span>
        </label>
      </div>
    </div>

  </div><!-- /sidebar -->
</div><!-- /main -->

<script>
  const BASE = window.location.origin;
  let streamRunning = true;
  let flashState = false;

  // ── Start stream ──────────────────────────────────────────────────────────
  const img = document.getElementById('stream-img');
  function startStream() {
    img.src = BASE + '/stream?' + Date.now();
    img.onerror = () => {
      setStatus(false);
      setTimeout(startStream, 3000);
    };
    img.onload = () => setStatus(true);
  }
  startStream();

  // ── Status helpers ────────────────────────────────────────────────────────
  function setStatus(online) {
    document.getElementById('status-dot').className = 'dot ' + (online ? 'live' : '');
    document.getElementById('status-text').textContent = online ? 'Live' : 'Reconnecting…';
  }

  // ── Pause / resume ────────────────────────────────────────────────────────
  function toggleStream() {
    const btn = document.querySelector('.btn.danger');
    if (streamRunning) {
      img.src = '';
      btn.textContent = '▶ Resume Stream';
      streamRunning = false;
    } else {
      startStream();
      btn.textContent = '⏸ Pause Stream';
      streamRunning = true;
    }
  }

  // ── Flash toggle ──────────────────────────────────────────────────────────
  function toggleFlash() {
    fetch(BASE + '/flash')
      .then(r => r.json())
      .then(d => {
        flashState = d.flash;
        const btn = document.getElementById('flash-btn');
        btn.textContent = '💡 Flash: ' + (flashState ? 'ON' : 'OFF');
        btn.className = 'btn' + (flashState ? ' active' : '');
      })
      .catch(console.error);
  }

  // ── Snapshot ──────────────────────────────────────────────────────────────
  function takeSnapshot() {
    const prev = document.getElementById('snapshot-preview');
    const ts   = document.getElementById('snap-time');
    prev.src = BASE + '/capture?' + Date.now();
    prev.style.display = 'block';
    ts.style.display = 'block';
    ts.textContent = 'Captured: ' + new Date().toLocaleTimeString();

    // Also trigger browser download
    const a = document.createElement('a');
    a.href = prev.src;
    a.download = 'snapshot_' + Date.now() + '.jpg';
    a.click();
  }

  // ── Camera settings ───────────────────────────────────────────────────────
  function applySetting(key, val) {
    fetch(BASE + '/settings?' + key + '=' + val).catch(console.error);
  }

  // ── Status polling ────────────────────────────────────────────────────────
  function formatUptime(sec) {
    const h = Math.floor(sec / 3600);
    const m = Math.floor((sec % 3600) / 60);
    const s = sec % 60;
    return (h ? h + 'h ' : '') + (m ? m + 'm ' : '') + s + 's';
  }

  function pollStatus() {
    fetch(BASE + '/status')
      .then(r => r.json())
      .then(d => {
        const ut = formatUptime(d.uptime);
        const fps = d.fps.toFixed(1);
        const heap = (d.heap / 1024).toFixed(0) + 'k';

        document.getElementById('s-fps').textContent    = fps;
        document.getElementById('s-uptime').textContent = ut;
        document.getElementById('s-frames').textContent = d.frames;
        document.getElementById('s-heap').textContent   = heap;
        document.getElementById('fps-badge').textContent = fps + ' fps';
        document.getElementById('uptime-val').textContent = ut;
        document.getElementById('frame-val').textContent  = d.frames;
        document.getElementById('heap-val').textContent   = heap;
        setStatus(true);
      })
      .catch(() => setStatus(false));
  }

  setInterval(pollStatus, 2000);
  pollStatus();
</script>
</body>
</html>
)HTML";

static esp_err_t dashboardHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, DASHBOARD_HTML, strlen(DASHBOARD_HTML));
}

// ─── Start HTTP Servers ───────────────────────────────────────────────────────
// Two servers: port 80 (dashboard + API) and port 81 (MJPEG stream)
httpd_handle_t dashServer  = NULL;
httpd_handle_t streamServer = NULL;

void startServers() {
  // Stream server — dedicated port for high-bandwidth MJPEG
  httpd_config_t streamCfg = HTTPD_DEFAULT_CONFIG();
  streamCfg.server_port      = 81;
  streamCfg.ctrl_port        = 32769;
  streamCfg.max_uri_handlers = 2;
  streamCfg.stack_size       = 8192;
  streamCfg.recv_wait_timeout = 10;
  streamCfg.send_wait_timeout = 10;

  if (httpd_start(&streamServer, &streamCfg) == ESP_OK) {
    httpd_uri_t streamUri = {
      .uri = "/stream", .method = HTTP_GET,
      .handler = streamHandler, .user_ctx = NULL
    };
    httpd_register_uri_handler(streamServer, &streamUri);
    Serial.println("[HTTP] Stream server on port 81");
  }

  // Dashboard server
  httpd_config_t dashCfg = HTTPD_DEFAULT_CONFIG();
  dashCfg.server_port      = 80;
  dashCfg.max_uri_handlers = 8;
  dashCfg.stack_size       = 8192;

  if (httpd_start(&dashServer, &dashCfg) == ESP_OK) {
    httpd_uri_t routes[] = {
      { .uri="/",         .method=HTTP_GET, .handler=dashboardHandler, .user_ctx=NULL },
      { .uri="/capture",  .method=HTTP_GET, .handler=captureHandler,   .user_ctx=NULL },
      { .uri="/flash",    .method=HTTP_GET, .handler=flashHandler,     .user_ctx=NULL },
      { .uri="/status",   .method=HTTP_GET, .handler=statusHandler,    .user_ctx=NULL },
      { .uri="/settings", .method=HTTP_GET, .handler=settingsHandler,  .user_ctx=NULL },
    };
    for (auto& r : routes) httpd_register_uri_handler(dashServer, &r);
    Serial.println("[HTTP] Dashboard server on port 80");
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  // Disable brownout detector (camera draws spikes)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println("\n[BOOT] ESP32-CAM Surveillance");

  // Flash LED
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

  // Camera
  if (!initCamera()) {
    Serial.println("[BOOT] Camera init failed — halting");
    while (true) { delay(1000); }
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Connecting to " WIFI_SSID);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500); Serial.print(".");
    tries++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WiFi] Failed to connect — rebooting in 5s");
    delay(5000);
    ESP.restart();
  }

  Serial.println();
  Serial.println("╔════════════════════════════════╗");
  Serial.println("║  ESP32-CAM Surveillance Ready  ║");
  Serial.println("╠════════════════════════════════╣");
  Serial.printf( "║  Dashboard: http://%s\n", WiFi.localIP().toString().c_str());
  Serial.printf( "║  Stream:    http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  Serial.println("╚════════════════════════════════╝");

  startServers();
  startTime = millis();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  // WiFi watchdog — reconnect if dropped
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost connection, reconnecting…");
    WiFi.reconnect();
    int t = 0;
    while (WiFi.status() != WL_CONNECTED && t < 20) {
      delay(500); t++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[WiFi] Reconnected: " + WiFi.localIP().toString());
    }
  }
  delay(5000);
}
