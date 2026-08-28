/*
 * web_dashboard.cpp — HTTP + WebSocket dashboard for Tesla FSD ESP32
 *
 * HTTP  :80  → serves the embedded HTML page
 * WS    :81  → pushes JSON state every 1 s; receives control commands
 *
 * All HTML/CSS/JS is embedded as a raw-string literal — no external CDN.
 * State is copied and updated under the shared FSDState lock.
 *
 * Web/WiFi work runs in a dedicated FreeRTOS task pinned to Core 0.
 * CAN work runs separately from main.cpp on Core 1.
 */

#include "web_dashboard.h"
#include "can_command.h"
#include "can_dump.h"
#include "prefs.h"
#include "wifi_manager.h"
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdarg.h>

// ── Module state ──────────────────────────────────────────────────────────────
static FSDState     *g_state     = nullptr;   // shared with main
static portMUX_TYPE *g_state_mux = nullptr;   // owned by main
static QueueHandle_t g_can_command_queue = nullptr;
static uint32_t      g_can_request_id = 0;

static WebServer        g_http(80);
static WebSocketsServer g_ws(81);

#if defined(BOARD_LILYGO)
static constexpr bool k_sd_available = true;
#else
static constexpr bool k_sd_available = false;
#endif

static uint32_t g_start_ms    = 0;
static uint32_t g_last_rx     = 0;
static uint32_t g_last_fps_ms = 0;
static uint32_t g_last_can_seen_ms = 0;
static float    g_fps         = 0.0f;
static TaskHandle_t g_web_task_handle = nullptr;
static char     g_json_buf[4096];

#define CAN_VEHICLE_ALIVE_MS 3000u

static void web_server_task(void *param);

static void state_enter() {
    if (g_state_mux != nullptr) portENTER_CRITICAL(g_state_mux);
}

static void state_exit() {
    if (g_state_mux != nullptr) portEXIT_CRITICAL(g_state_mux);
}

static bool state_copy(FSDState *out) {
    if (g_state == nullptr || out == nullptr) return false;
    state_enter();
    *out = *g_state;
    state_exit();
    return true;
}

static const char *reset_reason_name(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON: return "Power-on";
        case ESP_RST_EXT: return "External";
        case ESP_RST_SW: return "Software";
        case ESP_RST_PANIC: return "Panic";
        case ESP_RST_INT_WDT: return "Interrupt WDT";
        case ESP_RST_TASK_WDT: return "Task WDT";
        case ESP_RST_WDT: return "Other WDT";
        case ESP_RST_DEEPSLEEP: return "Deep Sleep";
        case ESP_RST_BROWNOUT: return "Brownout";
        case ESP_RST_SDIO: return "SDIO";
        default: return "Unknown";
    }
}

// ── Embedded HTML/CSS/JS ──────────────────────────────────────────────────────
// Tesla dark theme; mobile-first (max 480 px); WebSocket on :81
static const char WEB_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="theme-color" content="#0a0a1a">
<link rel="icon" href="data:,">
<title>Tesla FSD</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
  --bg:#0a0a1a;--card:#111827;--card2:#1a1f35;
  --accent:#00d4aa;--accent2:#00b894;
  --red:#ff6b6b;--yellow:#ffd93d;--blue:#4dabf7;
  --border:#1e293b;--text:#e2e8f0;--text2:#94a3b8;--text3:#475569
}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
  background:var(--bg);color:var(--text);min-height:100vh}
.wrap{max-width:480px;margin:0 auto;padding:16px 16px 40px}

/* ── Header ── */
.hdr{text-align:center;padding:20px 0 12px;position:relative}
.hdr h1{font-size:1.65em;font-weight:700;
  background:linear-gradient(135deg,var(--accent),var(--blue));
  -webkit-background-clip:text;-webkit-text-fill-color:transparent;
  letter-spacing:-.02em}
.hdr .sub{font-size:.68em;color:var(--text3);margin-top:3px;
  letter-spacing:.1em;text-transform:uppercase}
.cdot{position:absolute;right:0;top:26px;width:10px;height:10px;
  border-radius:50%;background:var(--accent);
  box-shadow:0 0 10px var(--accent);transition:.4s}
.cdot.off{background:var(--red);box-shadow:0 0 10px var(--red)}

/* ── OTA Warning ── */
.ota{display:none;background:rgba(255,107,107,.1);border:1px solid rgba(255,107,107,.4);
  border-radius:12px;padding:12px 16px;margin-bottom:12px;text-align:center;
  color:var(--red);font-weight:700;font-size:.9em;letter-spacing:.04em;
  animation:pulse 1s ease-in-out infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.55}}

/* ── Error banner ── */
.err{display:none;color:var(--red);text-align:center;font-size:.78em;padding:8px;
  background:rgba(255,107,107,.07);border-radius:10px;margin-bottom:10px;
  border:1px solid rgba(255,107,107,.18)}

/* ── Cards ── */
.card{background:var(--card);border-radius:16px;padding:16px;
  margin-bottom:12px;border:1px solid var(--border)}
.card-head{display:flex;align-items:center;gap:8px;margin-bottom:12px}
.icon{width:28px;height:28px;border-radius:8px;display:flex;
  align-items:center;justify-content:center;font-size:.85em;font-weight:700}
.ic-s{background:rgba(0,212,170,.14);color:var(--accent)}
.ic-b{background:rgba(77,171,247,.14);color:var(--blue)}
.ic-c{background:rgba(255,217,61,.14);color:var(--yellow)}
.ic-d{background:rgba(148,163,184,.14);color:var(--text2)}
.card-head h2{font-size:.78em;font-weight:600;color:var(--text2);
  text-transform:uppercase;letter-spacing:.07em}

/* ── Rows ── */
.row{display:flex;justify-content:space-between;align-items:center;padding:9px 0}
.row+.row{border-top:1px solid rgba(255,255,255,.04)}
.lbl{color:var(--text2);font-size:.85em}

/* ── Pills ── */
.pill{display:inline-flex;align-items:center;gap:5px;
  padding:3px 10px;border-radius:20px;font-size:.8em;font-weight:600}
.pill.on{background:rgba(0,212,170,.14);color:var(--accent)}
.pill.off{background:rgba(71,85,105,.22);color:var(--text3)}
.pill.warn{background:rgba(255,107,107,.14);color:var(--red)}
.pd{width:6px;height:6px;border-radius:50%;flex-shrink:0;
  background:currentColor;box-shadow:0 0 5px currentColor}

/* ── Battery Hero ── */
.hero{text-align:center;padding-bottom:4px}
.soc-ring{width:120px;height:120px;margin:0 auto 14px;position:relative}
.soc-ring svg{transform:rotate(-90deg)}
.trk{fill:none;stroke:#1e293b;stroke-width:8}
.bar{fill:none;stroke:var(--accent);stroke-width:8;stroke-linecap:round;
  transition:stroke-dashoffset .8s ease,stroke .5s}
.soc-val{position:absolute;inset:0;display:flex;flex-direction:column;
  align-items:center;justify-content:center}
.soc-num{font-size:2em;font-weight:700;line-height:1;
  font-variant-numeric:tabular-nums}
.soc-lbl{font-size:.6em;color:var(--text3);margin-top:3px;text-transform:uppercase}
.hg{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;text-align:center}
.hg .hv{font-size:1.1em;font-weight:600;font-variant-numeric:tabular-nums}
.hg .hl{font-size:.65em;color:var(--text3);margin-top:2px}

/* ── CAN stat grid ── */
.sg{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.sb{background:var(--card2);border-radius:10px;padding:10px 12px}
.sb .sv{font-size:1.15em;font-weight:700;font-variant-numeric:tabular-nums}
.sb .sl{font-size:.64em;color:var(--text3);margin-top:2px}

/* ── Controls ── */
.btn-main{width:100%;padding:14px;border:none;border-radius:12px;
  font-size:.95em;font-weight:700;cursor:pointer;letter-spacing:.04em;
  transition:opacity .2s;margin-bottom:10px}
.btn-main:active{opacity:.75}
.btn-act{background:linear-gradient(135deg,var(--accent),var(--accent2));color:#000}
.btn-stop{background:rgba(255,107,107,.14);color:var(--red);
  border:1px solid rgba(255,107,107,.3)}
.sw{position:relative;width:44px;height:24px;flex-shrink:0}
.sw input{opacity:0;width:0;height:0}
.sl2{position:absolute;cursor:pointer;inset:0;background:#2a2a3e;
  border-radius:24px;transition:.3s}
.sl2:before{content:"";position:absolute;height:18px;width:18px;
  left:3px;bottom:3px;background:#555;border-radius:50%;transition:.3s}
input:checked+.sl2{background:var(--accent)}
input:checked+.sl2:before{transform:translateX(20px);background:#fff}

/* ── Driving profile ── */
.seg-group{display:inline-flex;gap:4px;background:var(--card2);border-radius:8px;padding:3px}
.seg-btn{padding:6px 14px;border:none;background:transparent;color:var(--text2);
  border-radius:6px;cursor:pointer;font-size:.8em}
.seg-btn.active{background:var(--accent);color:#000;font-weight:600}
.num-ctrl{display:flex;align-items:center;gap:6px}
.num-ctrl input{width:70px;background:var(--card2);border:1px solid var(--border);
  color:var(--text);padding:5px 8px;border-radius:6px;text-align:right}
.num-ctrl span{font-size:.75em;color:var(--text3)}
.tier-box{margin-top:10px;display:none}
.tier-row{display:grid;grid-template-columns:1fr auto 1fr;gap:8px;align-items:center;
  padding:7px 0;border-top:1px solid rgba(255,255,255,.04)}
.tier-row:first-child{border-top:0}
.tier-row input{width:64px;background:var(--card2);border:1px solid var(--border);
  color:var(--text);padding:5px 7px;border-radius:6px;text-align:right}
.tier-mid{font-size:.75em;color:var(--text3);white-space:nowrap}
.mode-row{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin-top:12px}
@media(max-width:430px){.mode-row{grid-template-columns:repeat(3,1fr)}}
.mode-card{background:rgba(255,255,255,.04);border:2px solid rgba(255,255,255,.06);
  border-radius:8px;padding:10px 6px;text-align:center;cursor:pointer;transition:.2s;min-height:58px}
.mode-card.active{border-color:var(--accent);background:rgba(0,212,170,.1)}
.mode-card.disabled{opacity:.35;pointer-events:none}
.mode-name{font-size:.78em;font-weight:600;line-height:1.25}

/* ── OTA firmware update ── */
.ota-file{display:none}.ota-progress{display:none;margin-top:12px}
.ota-track{background:var(--card2);border-radius:8px;height:10px;overflow:hidden}
.ota-bar{background:var(--accent);height:100%;width:0%;transition:width .2s}
.ota-status{text-align:center;margin-top:8px;font-size:.85em;color:var(--text2)}
.ota-bytes{text-align:center;margin-top:4px;font-size:.72em;color:var(--text3)}
.ota-info{margin-top:10px;padding:10px 12px;background:rgba(77,171,247,.07);
  border-radius:8px;border:1px solid rgba(77,171,247,.15);font-size:.72em;color:var(--text3);line-height:1.4}
.btn-blue{background:rgba(77,171,247,.14);color:var(--blue);border:1px solid rgba(77,171,247,.3)}
.btn-yellow{background:rgba(255,217,61,.14);color:var(--yellow);border:1px solid rgba(255,217,61,.3)}
.logbox{background:#080d1c;border:1px solid rgba(148,163,184,.18);border-radius:8px;
  min-height:150px;max-height:240px;overflow:auto;padding:10px 12px;
  font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:.68em;
  line-height:1.55;color:var(--text2);white-space:pre-wrap;word-break:break-word}
.log-empty{color:var(--text3)}

/* ── Footer ── */
.foot{text-align:center;padding:16px 0 0;font-size:.64em;color:var(--text3)}
</style>
</head>
<body>
<div class="wrap">

<!-- Header -->
<div class="hdr">
  <h1>Tesla FSD</h1>
  <div class="sub">ESP32 CAN Controller &middot; 192.168.4.1</div>
  <div class="cdot" id="dot"></div>
</div>
<div id="connErr" class="err">Connection lost &mdash; retrying&hellip;</div>

<!-- OTA Warning -->
<div id="otaBanner" class="ota">&#9888;&#xFE0F; OTA UPDATE IN PROGRESS &mdash; CAN TX SUSPENDED</div>

<!-- FSD Status -->
<div class="card">
  <div class="card-head"><div class="icon ic-s">S</div><h2>FSD Status</h2></div>
  <div class="row">
    <span class="lbl">FSD Active</span>
    <span class="pill off" id="fsdSt"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">Mode</span>
    <span class="pill off" id="opMode"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">Hardware</span>
    <span class="pill off" id="hwVer"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">NAG Killer</span>
    <span class="pill off" id="nagSt"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">CAN Vehicle</span>
    <span class="pill off" id="canVeh"><span class="pd"></span>--</span>
  </div>
</div>

<!-- Battery -->
<div class="card" id="batteryCard" style="display:none">
  <div class="card-head"><div class="icon ic-b">B</div><h2>Battery</h2></div>
  <div class="row">
    <span class="lbl">BMS Status</span>
    <span class="pill off" id="bmsSt"><span class="pd"></span>Waiting Frames</span>
  </div>
  <div class="row">
    <span class="lbl">BMS Frames</span>
    <span id="bmsFrames" style="font-size:.8em;color:var(--text2)">HV:0 SOC:0 TH:0</span>
  </div>
  <div class="hero">
    <div class="soc-ring">
      <svg viewBox="0 0 120 120" width="120" height="120">
        <circle class="trk" cx="60" cy="60" r="52"/>
        <circle class="bar" id="socBar" cx="60" cy="60" r="52"
          stroke-dasharray="326.73" stroke-dashoffset="326.73"/>
      </svg>
      <div class="soc-val">
        <span class="soc-num" id="bSoc">--</span>
        <span class="soc-lbl">SOC</span>
      </div>
    </div>
    <div class="hg">
      <div><div class="hv" id="bVolt">--</div><div class="hl">Voltage</div></div>
      <div><div class="hv" id="bCurr">--</div><div class="hl">Current</div></div>
      <div><div class="hv" id="bTemp">--</div><div class="hl">Temp</div></div>
    </div>
  </div>
</div>

<!-- CAN Stats -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">C</div><h2>CAN Bus</h2></div>
  <div class="sg">
    <div class="sb"><div class="sv" id="rxCnt">0</div><div class="sl">RX Frames</div></div>
    <div class="sb"><div class="sv" id="txCnt">0</div><div class="sl">Modified</div></div>
    <div class="sb"><div class="sv" id="txSent">0</div><div class="sl">TX Sent</div></div>
    <div class="sb"><div class="sv" id="txFail">0</div><div class="sl">TX Failed</div></div>
    <div class="sb"><div class="sv" id="rxMissed">0</div><div class="sl">RX Missed</div></div>
    <div class="sb"><div class="sv" id="busErr">0</div><div class="sl">Bus Errors</div></div>
    <div class="sb"><div class="sv" id="rxOverrun">0</div><div class="sl">RX Overrun</div></div>
    <div class="sb"><div class="sv" id="twaiRestarts">0</div><div class="sl">TWAI Restarts</div></div>
    <div class="sb"><div class="sv" id="twaiState">--</div><div class="sl">TWAI State</div></div>
    <div class="sb"><div class="sv" id="fps">0.0</div><div class="sl">Frames/s</div></div>
  </div>
</div>

<!-- Controls -->
<div class="card">
  <div class="card-head"><div class="icon ic-c">C</div><h2>Controls</h2></div>
  <button id="btnMode" class="btn-main btn-act" onclick="toggleMode()">ACTIVATE FSD</button>
  <div class="row">
    <span class="lbl">NAG Killer</span>
    <label class="sw"><input type="checkbox" id="swNag" onchange="cmd('nag',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" id="bmsDisplayRow" style="display:none">
    <span class="lbl">BMS Display</span>
    <label class="sw"><input type="checkbox" id="swBms" onchange="cmd('bms',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">Force FSD</span>
    <label class="sw"><input type="checkbox" id="swFsd" onchange="cmd('force_fsd',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">Suppress Speed Chime</span>
    <label class="sw"><input type="checkbox" id="swChime" onchange="cmd('suppress_speed_chime',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">China Mode</span>
    <label class="sw"><input type="checkbox" id="swChina" onchange="cmd('china_mode',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">TLSSC Restore</span>
    <label class="sw"><input type="checkbox" id="swTlssc" onchange="cmd('tlssc_restore',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" id="dumpRow" style="display:none">
    <span class="lbl">CAN Dump</span>
    <label class="sw"><input type="checkbox" id="swDump" onchange="cmd('dump',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">Hardware Source</span>
    <div class="seg-group">
      <button class="seg-btn active" id="btnHwAuto" onclick="setHwAuto(true)">Auto</button>
      <button class="seg-btn" id="btnHwMan" onclick="setHwAuto(false)">Manual</button>
    </div>
  </div>
  <div class="mode-row" id="hwRow">
    <div class="mode-card" data-hw="1" onclick="selectHW(1)"><div class="mode-name">Legacy</div></div>
    <div class="mode-card active" data-hw="2" onclick="selectHW(2)"><div class="mode-name">HW3</div></div>
    <div class="mode-card" data-hw="3" onclick="selectHW(3)"><div class="mode-name">HW4</div></div>
  </div>
</div>

<!-- Driving Profile -->
<div class="card" id="profileCard" style="display:none">
  <div class="card-head"><div class="icon ic-c">P</div><h2>Driving Profile</h2></div>
  <div class="row">
    <span class="lbl">Profile Source</span>
    <div class="seg-group">
      <button class="seg-btn active" id="btnProfAuto" onclick="setProfileMode(true)">Auto</button>
      <button class="seg-btn" id="btnProfMan" onclick="setProfileMode(false)">Manual</button>
    </div>
  </div>
  <div class="mode-row" id="modeRow">
    <div class="mode-card" data-val="3" onclick="selectProfile(3)"><div class="mode-name">Max</div></div>
    <div class="mode-card" data-val="2" onclick="selectProfile(2)"><div class="mode-name">Hurry</div></div>
    <div class="mode-card active" data-val="1" onclick="selectProfile(1)"><div class="mode-name">Normal</div></div>
    <div class="mode-card" data-val="0" onclick="selectProfile(0)"><div class="mode-name">Chill</div></div>
    <div class="mode-card" data-val="4" onclick="selectProfile(4)"><div class="mode-name">Sloth</div></div>
  </div>
  <div class="row" id="hw3OffsetModeRow">
    <span class="lbl">Offset Mode</span>
    <div class="seg-group">
      <button class="seg-btn active" id="btnOffAuto" onclick="setHw3OffsetMode('auto')">Auto</button>
      <button class="seg-btn" id="btnOffFixed" onclick="setOffsetMode(false)">Fixed</button>
      <button class="seg-btn" id="btnOffPct" onclick="setOffsetMode(true)">%</button>
    </div>
  </div>
  <div class="row" id="hw3OffsetRow">
    <span class="lbl">Base Raw Offset</span>
    <div class="num-ctrl">
      <input type="number" id="numHw3Offset" min="0" max="40" step="1" onchange="setHw3Offset(this.value)">
      <span id="hw3OffsetUnit"></span>
    </div>
  </div>
  <div class="row" id="fixedOffsetRow">
    <span class="lbl">Fixed Offset</span>
    <div class="num-ctrl">
      <input type="number" id="numHw4Offset" min="0" max="50" step="1" onchange="setHw4Offset(this.value)">
      <span>%</span>
    </div>
  </div>
  <div class="row" id="dasLimitRow">
    <span class="lbl">Current Limit</span>
    <span id="dasLimit" style="font-size:.85em;color:var(--text2)">--</span>
  </div>
  <div class="row" id="activeOffsetRow">
    <span class="lbl" id="activeOffsetLabel">Active Offset</span>
    <span id="activeOffset" style="font-size:.85em;color:var(--text2)">--</span>
  </div>
  <div class="tier-box" id="pctOffsetBox">
    <div class="tier-row">
      <span class="lbl">Limit 1</span>
      <span class="tier-mid">&le;</span>
      <div class="num-ctrl"><input type="number" id="tierLimit0" min="0" max="155" step="5" onchange="setOffsetTier(0,'limit',this.value)"><span>km/h</span></div>
    </div>
    <div class="tier-row">
      <span class="lbl" id="tierOffsetLabel0">Offset 1</span>
      <span class="tier-mid">+</span>
      <div class="num-ctrl"><input type="number" id="tierPct0" min="0" max="50" step="1" onchange="setOffsetTier(0,'percent',this.value)"><span id="tierUnit0">%</span></div>
    </div>
    <div class="tier-row">
      <span class="lbl">Limit 2</span>
      <span class="tier-mid">&le;</span>
      <div class="num-ctrl"><input type="number" id="tierLimit1" min="0" max="155" step="5" onchange="setOffsetTier(1,'limit',this.value)"><span>km/h</span></div>
    </div>
    <div class="tier-row">
      <span class="lbl" id="tierOffsetLabel1">Offset 2</span>
      <span class="tier-mid">+</span>
      <div class="num-ctrl"><input type="number" id="tierPct1" min="0" max="50" step="1" onchange="setOffsetTier(1,'percent',this.value)"><span id="tierUnit1">%</span></div>
    </div>
    <div class="tier-row">
      <span class="lbl">Limit 3</span>
      <span class="tier-mid">&le;</span>
      <div class="num-ctrl"><input type="number" id="tierLimit2" min="0" max="155" step="5" onchange="setOffsetTier(2,'limit',this.value)"><span>km/h</span></div>
    </div>
    <div class="tier-row">
      <span class="lbl" id="tierOffsetLabel2">Offset 3</span>
      <span class="tier-mid">+</span>
      <div class="num-ctrl"><input type="number" id="tierPct2" min="0" max="50" step="1" onchange="setOffsetTier(2,'percent',this.value)"><span id="tierUnit2">%</span></div>
    </div>
    <div class="tier-row">
      <span class="lbl">Limit 4</span>
      <span class="tier-mid">&le;</span>
      <div class="num-ctrl"><input type="number" id="tierLimit3" min="0" max="155" step="5" onchange="setOffsetTier(3,'limit',this.value)"><span>km/h</span></div>
    </div>
    <div class="tier-row">
      <span class="lbl" id="tierOffsetLabel3">Offset 4</span>
      <span class="tier-mid">+</span>
      <div class="num-ctrl"><input type="number" id="tierPct3" min="0" max="50" step="1" onchange="setOffsetTier(3,'percent',this.value)"><span id="tierUnit3">%</span></div>
    </div>
  </div>
</div>

<!-- WiFi Config -->
<div class="card">
  <div class="card-head"><div class="icon ic-c">W</div><h2>WiFi Configuration</h2></div>
  <div class="row">
    <span class="lbl">SSID</span>
    <input type="text" id="wifiSsid" maxlength="32" style="width:140px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right">
  </div>
  <div class="row">
    <span class="lbl">Password</span>
    <input type="password" id="wifiPass" maxlength="64" style="width:140px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right">
  </div>
  <div class="row">
    <span class="lbl">Stealth Mode (Hidden)</span>
    <label class="sw"><input type="checkbox" id="swWifiHid"><span class="sl2"></span></label>
  </div>
  <button class="btn-main btn-stop" onclick="saveWifi()" style="margin-top:12px">SAVE & RESTART WIFI</button>
</div>

<!-- OTA Update -->
<div class="card">
  <div class="card-head"><div class="icon ic-c">U</div><h2>OTA Firmware Update</h2></div>
  <div style="font-size:.75em;color:var(--text3);margin-bottom:12px;line-height:1.5">
    Upload a .bin firmware file. Device will reboot after a successful update.
  </div>
  <form id="otaForm" enctype="multipart/form-data" style="margin:0">
    <input type="file" id="otaFile" class="ota-file" accept=".bin" onchange="uploadFirmware()">
    <button type="button" class="btn-main btn-blue" id="otaSelectBtn" onclick="document.getElementById('otaFile').click()">
      SELECT FIRMWARE (.bin)
    </button>
  </form>
  <div id="otaProgress" class="ota-progress">
    <div class="ota-track"><div id="otaBar" class="ota-bar"></div></div>
    <div id="otaStatus" class="ota-status">Preparing...</div>
    <div id="otaBytes" class="ota-bytes"></div>
  </div>
  <div id="otaRollbackInfo" class="ota-info">
    <b style="color:var(--blue)">Partition Safety</b><br>
    OTA writes to the next app partition when available. Keep USB reflashing available as a recovery path.
  </div>
</div>

<!-- SD Card -->
<div class="card" id="sdCard" style="display:none">
  <div class="card-head"><div class="icon ic-d">S</div><h2>SD Card</h2></div>
  <div class="row">
    <span class="lbl">Dump Status</span>
    <span class="pill off" id="dumpSt"><span class="pd"></span>Idle</span>
  </div>
  <button id="btnFmt" class="btn-main btn-stop" onclick="sdFormat()" style="margin-top:8px">FORMAT SD CARD</button>
  <div id="fmtOut" style="font-size:.75em;color:var(--text2);margin-top:8px;display:none"></div>
</div>

<!-- Device Info -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">D</div><h2>Device</h2></div>
  <div class="row">
    <span class="lbl">Firmware</span>
    <span id="fwBuild" style="font-size:.8em;color:var(--text2)">--</span>
  </div>
  <div class="row">
    <span class="lbl">Uptime</span>
    <span id="uptime" style="font-variant-numeric:tabular-nums">--</span>
  </div>
  <div class="row">
    <span class="lbl">Reset Reason</span>
    <span id="resetReason" style="font-size:.8em;color:var(--text2)">--</span>
  </div>
  <div class="row">
    <span class="lbl">Chip Temp</span>
    <span id="chipTemp" style="font-size:.8em;color:var(--text2)">--</span>
  </div>
  <div class="row">
    <span class="lbl">Free Heap</span>
    <span id="freeHeap" style="font-size:.8em;color:var(--text2)">--</span>
  </div>
  <div class="row">
    <span class="lbl">Min Heap</span>
    <span id="minHeap" style="font-size:.8em;color:var(--text2)">--</span>
  </div>
  <div class="row">
    <span class="lbl">Web Stack</span>
    <span id="webStack" style="font-size:.8em;color:var(--text2)">--</span>
  </div>
  <div class="row">
    <span class="lbl">CAN Stack</span>
    <span id="canStack" style="font-size:.8em;color:var(--text2)">--</span>
  </div>
  <div class="row">
    <span class="lbl">WiFi Clients</span>
    <span id="wifiCl">--</span>
  </div>
  <div class="row">
    <span class="lbl">OTA Partition</span>
    <span id="otaPartInfo" style="font-size:.78em;color:var(--text2)">--</span>
  </div>
  <button class="btn-main btn-yellow" onclick="restartDevice()" style="margin-top:12px">RESTART DEVICE</button>
</div>

<!-- Debug Log -->
<div class="card" id="debugCard" style="display:none">
  <div class="card-head"><div class="icon ic-d">L</div><h2>Debug Log</h2></div>
  <div id="debugLog" class="logbox"></div>
</div>

<div class="foot">Tesla FSD ESP32 &middot; M5Stack ATOM Lite + ATOMIC CAN Base</div>
</div><!-- /wrap -->

<script>
var ws,rt,busy=0,wifiOnce=false,offsetHw=0,lastState=null;
var HW=['Unknown','Legacy','HW3','HW4'];
var TWAI=['Stopped','Running','Bus-Off','Recovering'];
var CIRC=326.73;
var logLines=[],lastLog='';

function initWifi(d){
  if(wifiOnce)return;
  wifiOnce=true;
  document.getElementById('wifiSsid').value=d.wifi_ssid||'';
  document.getElementById('wifiPass').value=d.wifi_pass||'';
  document.getElementById('swWifiHid').checked=!!d.wifi_hidden;
}

function fmt(s){
  var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;
  return h+':'+(m<10?'0':'')+m+':'+(sc<10?'0':'')+sc;
}
function kb(v){return ((v||0)/1024).toFixed(1)+' KB';}
function socCol(p){return p>60?'var(--accent)':p>30?'var(--yellow)':'var(--red)';}
function pill(id,on,txt,warnClass){
  var e=document.getElementById(id);
  e.className='pill '+(warnClass||''+(on?'on':'off'));
  e.innerHTML='<span class="pd"></span>'+txt;
}
function ring(p){
  var b=document.getElementById('socBar');
  b.style.strokeDashoffset=CIRC-(CIRC*Math.min(p,100)/100);
  b.style.stroke=socCol(p);
}

function hw3ValueToBase(value){
  var val=parseInt(value,10);
  if(isNaN(val))val=0;
  if(val<0)val=0;
  if(val>200)val=200;
  return Math.round(val/5);
}

function hw3BaseToValue(base){
  var val=parseInt(base,10);
  if(isNaN(val))val=0;
  if(val<0)val=0;
  if(val>40)val=40;
  return val*5;
}

function appendLog(line){
  if(!line || line===lastLog)return;
  lastLog=line;
  var t=new Date();
  var ts=(t.getHours()<10?'0':'')+t.getHours()+':'+(t.getMinutes()<10?'0':'')+t.getMinutes()+':'+(t.getSeconds()<10?'0':'')+t.getSeconds();
  logLines.push(ts+' '+line);
  if(logLines.length>12)logLines.shift();
  var box=document.getElementById('debugLog');
  var card=document.getElementById('debugCard');
  if(card)card.style.display='block';
  if(box){
    box.textContent=logLines.join('\n');
    box.scrollTop=box.scrollHeight;
  }
}

function upd(d){
  if(!d || Date.now() < busy) return;
  // Status
  pill('fsdSt', d.fsd_enabled, d.fsd_enabled?'Active':'Waiting');
  if(d.can_mode_switch_pending){
    pill('opMode',false,'Switching…');
  }else if(d.can_mode_switch_failed){
    pill('opMode',false,'Switch Failed');
  }else{
    pill('opMode',d.op_mode===1,d.op_mode===1?'Active':'Listen-Only');
  }

  var hwEl=document.getElementById('hwVer');
  if(hwEl){
    hwEl.className='pill '+(d.hw_version>0?'on':'off');
    hwEl.innerHTML='<span class="pd"></span>'+(HW[d.hw_version]||'?');
  }

  pill('nagSt', d.nag_killer, d.nag_killer?'ON':'OFF');
  pill('canVeh', d.can_vehicle_detected, d.can_vehicle_detected?'Detected':'No CAN Traffic');
  var bmsSeen=!!(d.bms&&d.bms.seen);
  var batteryCard=document.getElementById('batteryCard');
  var bmsDisplayRow=document.getElementById('bmsDisplayRow');
  if(batteryCard)batteryCard.style.display=bmsSeen?'block':'none';
  if(bmsDisplayRow)bmsDisplayRow.style.display=bmsSeen?'flex':'none';
  if(bmsSeen)pill('bmsSt',true,'Live');
  var bF=document.getElementById('bmsFrames');
  if(bF) bF.textContent='HV:'+(d.bms_hv_seen||0)+' SOC:'+(d.bms_soc_seen||0)+' TH:'+(d.bms_thermal_seen||0);

  // OTA banner
  var otaB=document.getElementById('otaBanner');
  if(otaB) otaB.style.display=d.ota?'block':'none';

  // Mode button
  var act=d.op_mode===1;
  var btn=document.getElementById('btnMode');
  if(btn){
    btn.disabled=!!d.can_mode_switch_pending;
    btn.textContent=d.can_mode_switch_pending?'SWITCHING…':(act?'STOP FSD  \u2192  Listen-Only':'ACTIVATE FSD  \u2192  Active');
    btn.className='btn-main '+(act?'btn-stop':'btn-act');
  }

  // Switches sync
  if(document.getElementById('swNag')) document.getElementById('swNag').checked=d.nag_killer;
  if(document.getElementById('swBms')) document.getElementById('swBms').checked=bmsSeen&&d.bms_output;
  if(document.getElementById('swFsd')) document.getElementById('swFsd').checked=d.force_fsd;
  if(document.getElementById('swChime')) document.getElementById('swChime').checked=d.suppress_speed_chime;
  if(document.getElementById('swChina')) document.getElementById('swChina').checked=d.china_mode;
  if(document.getElementById('swTlssc')) document.getElementById('swTlssc').checked=d.tlssc_restore;
  var sdAvailable=!!d.sd_available;
  var dumpRow=document.getElementById('dumpRow');
  var sdCard=document.getElementById('sdCard');
  if(dumpRow)dumpRow.style.display=sdAvailable?'flex':'none';
  if(sdCard)sdCard.style.display=sdAvailable?'block':'none';
  if(document.getElementById('swDump')) document.getElementById('swDump').checked=sdAvailable&&!!d.can_dump;
  if(sdAvailable)pill('dumpSt',d.can_dump,d.can_dump?'Recording':'Idle');

  // Hardware override
  if(d.hw_mode_auto!==undefined){
    var hwAuto=document.getElementById('btnHwAuto');
    var hwMan=document.getElementById('btnHwMan');
    if(hwAuto&&hwMan){
      hwAuto.className=d.hw_mode_auto?'seg-btn active':'seg-btn';
      hwMan.className=d.hw_mode_auto?'seg-btn':'seg-btn active';
    }
    document.querySelectorAll('#hwRow .mode-card').forEach(function(c){
      c.classList.toggle('disabled',d.hw_mode_auto);
    });
  }
  var selectedHw=d.hw_mode_auto?d.hw_version:d.manual_hw_version;
  document.querySelectorAll('#hwRow .mode-card').forEach(function(c){
    c.classList.toggle('active',c.dataset.hw===String(selectedHw));
  });

  // Driving profile (HW3/HW4)
  var profCard=document.getElementById('profileCard');
  var isHw3=d.hw_version===2;
  var isHw4=d.hw_version===3;
  offsetHw=d.hw_version||0;
  if(profCard) profCard.style.display=(isHw3||isHw4)?'block':'none';
  if(d.profile_mode_auto!==undefined){
    var btnAuto=document.getElementById('btnProfAuto');
    var btnMan=document.getElementById('btnProfMan');
    if(btnAuto&&btnMan){
      btnAuto.className=d.profile_mode_auto?'seg-btn active':'seg-btn';
      btnMan.className=d.profile_mode_auto?'seg-btn':'seg-btn active';
    }
    document.querySelectorAll('#modeRow .mode-card').forEach(function(c){
      c.classList.toggle('disabled',d.profile_mode_auto);
    });
  }
  var activeProfile=d.speed_profile;
  document.querySelectorAll('#modeRow .mode-card').forEach(function(c){
    var val=parseInt(c.dataset.val,10);
    var overMax=isHw3 && val>2;
    c.classList.toggle('disabled',d.profile_mode_auto||overMax);
    c.classList.toggle('active',!overMax && c.dataset.val===String(activeProfile));
  });
  var hw3AutoRow=document.getElementById('hw3OffsetModeRow');
  var hw3Row=document.getElementById('hw3OffsetRow');
  if(hw3AutoRow) hw3AutoRow.style.display=(isHw3||isHw4)?'flex':'none';
  if(hw3Row) hw3Row.style.display=(isHw3 && !d.hw3_offset_auto && !d.hw3_offset_percent_mode)?'flex':'none';
  syncOffsetLabels(isHw3);
  syncOffsetButtons(isHw3?!!d.hw3_offset_auto:false,isHw3?!!d.hw3_offset_percent_mode:!!d.hw4_offset_percent_mode,isHw3);
  var hw3off=document.getElementById('numHw3Offset');
  if(hw3off && document.activeElement.id!=='numHw3Offset' && d.hw3_offset!==undefined){
    hw3off.value=hw3ValueToBase(d.hw3_offset);
  }
  var hw4off=document.getElementById('numHw4Offset');
  if(hw4off && document.activeElement.id!=='numHw4Offset' && d.hw4_offset!==undefined){
    hw4off.value=d.hw4_offset;
  }
  syncOffsetMode(isHw3?!!d.hw3_offset_percent_mode:!!d.hw4_offset_percent_mode,isHw3,isHw4);
  if(isHw3 && d.hw3_offset_auto){
    if(hw3Row) hw3Row.style.display='none';
    var pctBoxAuto=document.getElementById('pctOffsetBox');
    if(pctBoxAuto) pctBoxAuto.style.display='none';
  }
  var activeRow=document.getElementById('activeOffsetRow');
  if(activeRow && isHw3) activeRow.style.display='flex';
  for(var ti=0;ti<4;ti++){
    var lim=document.getElementById('tierLimit'+ti);
    var pct=document.getElementById('tierPct'+ti);
    var prefix=isHw3?'hw3_tier':'hw4_tier';
    if(lim && document.activeElement.id!==lim.id && d[prefix+ti+'_limit']!==undefined) lim.value=d[prefix+ti+'_limit'];
    if(pct && document.activeElement.id!==pct.id && d[prefix+ti+'_percent']!==undefined){
      pct.value=isHw3?hw3ValueToBase(d[prefix+ti+'_percent']):d[prefix+ti+'_percent'];
    }
    if(pct) pct.max=isHw3?'33':'50';
  }
  var dasLimit=document.getElementById('dasLimit');
  if(dasLimit) dasLimit.textContent=(d.das_speed_limit_kph>0)?(d.das_speed_limit_kph+' km/h'):'--';
  var activeOffset=document.getElementById('activeOffset');
  if(activeOffset){
    if(isHw3){
      var activeVal=d.hw3_offset_active||0;
      activeOffset.textContent=hw3ValueToBase(activeVal)+' (Value '+activeVal+')';
    }else{
      activeOffset.textContent=(d.hw4_offset_active||0)+'%';
    }
  }
  appendLog(d.debug_log);

  // CAN stats
  if(document.getElementById('rxCnt')) document.getElementById('rxCnt').textContent=(d.rx_count||0).toLocaleString();
  if(document.getElementById('txCnt')) document.getElementById('txCnt').textContent=(d.tx_count||0).toLocaleString();
  if(document.getElementById('txSent')) document.getElementById('txSent').textContent=(d.tx_sent||0).toLocaleString();
  if(document.getElementById('txFail')) document.getElementById('txFail').textContent=(d.tx_failed||0).toLocaleString();
  if(document.getElementById('rxMissed')) document.getElementById('rxMissed').textContent=(d.rx_missed||0).toLocaleString();
  if(document.getElementById('busErr')) document.getElementById('busErr').textContent=(d.bus_errors||0).toLocaleString();
  if(document.getElementById('rxOverrun')) document.getElementById('rxOverrun').textContent=(d.rx_overrun||0).toLocaleString();
  if(document.getElementById('twaiRestarts')) document.getElementById('twaiRestarts').textContent=(d.twai_restarts||0).toLocaleString();
  if(document.getElementById('twaiState')) document.getElementById('twaiState').textContent=TWAI[d.twai_state]||'Unknown';
  if(document.getElementById('fps')) document.getElementById('fps').textContent=(d.fps||0.0).toFixed(1);

  // Battery
  if(d.bms && d.bms.seen){
    var sn=document.getElementById('bSoc');
    if(sn){
      sn.textContent=d.bms.soc.toFixed(0)+'%';
      sn.style.color=socCol(d.bms.soc);
    }
    ring(d.bms.soc);
    if(document.getElementById('bVolt')) document.getElementById('bVolt').textContent=d.bms.voltage.toFixed(0)+'V';
    var ce=document.getElementById('bCurr');
    if(ce){
      ce.textContent=(d.bms.current>=0?'+':'')+d.bms.current.toFixed(1)+'A';
      ce.style.color=d.bms.current>=0?'var(--accent)':'var(--red)';
    }
    if(document.getElementById('bTemp')) document.getElementById('bTemp').textContent=d.bms.temp_min+'~'+d.bms.temp_max+'\u00b0C';
  }

  // Device
  if(document.getElementById('fwBuild')) document.getElementById('fwBuild').textContent=d.fw_build;
  if(document.getElementById('uptime')) document.getElementById('uptime').textContent=fmt(d.uptime_s||0);
  if(document.getElementById('resetReason')) document.getElementById('resetReason').textContent=d.reset_reason||'--';
  if(document.getElementById('chipTemp')) document.getElementById('chipTemp').textContent=d.chip_temp_valid?((d.chip_temp_c||0).toFixed(1)+'\u00b0C'):'N/A';
  if(document.getElementById('freeHeap')) document.getElementById('freeHeap').textContent=kb(d.free_heap);
  if(document.getElementById('minHeap')) document.getElementById('minHeap').textContent=kb(d.min_free_heap);
  if(document.getElementById('webStack')) document.getElementById('webStack').textContent=kb((d.web_stack_free_words||0)*4);
  if(document.getElementById('canStack')) document.getElementById('canStack').textContent=kb((d.can_stack_free_words||0)*4);
  if(document.getElementById('wifiCl')) document.getElementById('wifiCl').textContent=d.wifi_clients||0;
  var partEl=document.getElementById('otaPartInfo');
  if(partEl && d.ota_partition){
    var p=d.ota_partition;
    var stateStr=(p.state===0)?'New':(p.state===1)?'Pending':(p.state===2)?'Valid':(p.state===3)?'Invalid':'State '+p.state;
    partEl.textContent=p.running+' ('+stateStr+') - '+(p.has_ota?'OTA capable':'No OTA partition');
    var info=document.getElementById('otaRollbackInfo');
    if(info && !p.has_ota){
      info.innerHTML='<b style="color:var(--red)">No OTA Partition</b><br>This build appears to be running from a factory/single app partition. Use an OTA partition table before relying on Web updates.';
    }
  }
}

function uploadFirmware(){
  var input=document.getElementById('otaFile');
  var file=input.files[0];
  if(!file)return;
  if(!file.name.endsWith('.bin')){alert('Error: Please select a .bin firmware file');input.value='';return;}
  var MAX_SIZE=16*1024*1024;
  if(file.size>MAX_SIZE){alert('Error: Firmware file too large (max 16MB)');input.value='';return;}
  if(file.size<32768 && !confirm('Warning: This file is very small ('+Math.round(file.size/1024)+' KB).\nAre you sure it is a valid ESP32 firmware?')){input.value='';return;}
  if(!confirm('Flash firmware: '+file.name+' ('+Math.round(file.size/1024)+' KB)?\n\nDevice will reboot after update.')){input.value='';return;}
  var prog=document.getElementById('otaProgress'),bar=document.getElementById('otaBar'),status=document.getElementById('otaStatus'),bytes=document.getElementById('otaBytes'),btn=document.getElementById('otaSelectBtn');
  prog.style.display='block';bar.style.width='0%';bar.style.background='var(--accent)';status.textContent='Uploading firmware...';status.style.color='var(--text2)';bytes.textContent='0 / '+Math.round(file.size/1024)+' KB';btn.disabled=true;btn.style.opacity='.5';
  var xhr=new XMLHttpRequest();
  xhr.upload.addEventListener('progress',function(e){if(e.lengthComputable){var pct=Math.round((e.loaded/e.total)*100);bar.style.width=pct+'%';status.textContent='Uploading: '+pct+'%';bytes.textContent=Math.round(e.loaded/1024)+' / '+Math.round(e.total/1024)+' KB';}});
  xhr.addEventListener('load',function(){btn.disabled=false;btn.style.opacity='1';if(xhr.status===200&&xhr.responseText==='OK'){bar.style.width='100%';status.textContent='Upload complete - rebooting...';status.style.color='var(--accent)';var c=8;var t=setInterval(function(){c--;bytes.textContent='Reconnecting in '+c+'s...';if(c<=0){clearInterval(t);location.reload();}},1000);}else{bar.style.background='var(--red)';status.textContent='Update failed';status.style.color='var(--red)';bytes.textContent='Server response: '+(xhr.responseText||xhr.statusText||'Unknown error');input.value='';}});
  xhr.addEventListener('error',function(){btn.disabled=false;btn.style.opacity='1';bar.style.background='var(--red)';status.textContent='Connection lost during upload';status.style.color='var(--red)';bytes.textContent='Check WiFi connection and try again';input.value='';});
  xhr.addEventListener('timeout',function(){btn.disabled=false;btn.style.opacity='1';bar.style.background='var(--yellow)';status.textContent='Upload timed out';status.style.color='var(--yellow)';bytes.textContent='The device may have rebooted - check if new firmware is running';input.value='';});
  var fd=new FormData();fd.append('firmware',file);xhr.open('POST','/update',true);xhr.timeout=120000;xhr.send(fd);
}

function restartDevice(){
  if(!confirm('Restart the device now?'))return;
  fetch('/restart').then(function(){setTimeout(function(){location.reload();},8000);}).catch(function(){setTimeout(function(){location.reload();},8000);});
}

function sdFormat(){
  if(!confirm('Format SD card? All data will be lost.'))return;
  var btn=document.getElementById('btnFmt');
  var out=document.getElementById('fmtOut');
  btn.disabled=true;btn.textContent='FORMATTING\u2026';
  fetch('/sdformat').then(function(r){return r.json();}).then(function(d){
    out.style.display='block';
    out.style.color=d.ok?'var(--accent)':'var(--red)';
    out.textContent=d.msg+(d.ok?' \u2014 '+d.free_mb+' MB free':'');
  }).catch(function(){
    out.style.display='block';out.style.color='var(--red)';out.textContent='Request failed';
  }).then(function(){
    btn.disabled=false;btn.textContent='FORMAT SD CARD';
  });
}
function saveWifi(){
  var s=document.getElementById('wifiSsid').value;
  var p=document.getElementById('wifiPass').value;
  var h=document.getElementById('swWifiHid').checked;
  if(s.length<1){alert('SSID required');return;}
  if(p.length>0 && p.length<8){alert('Password must be 8+ chars');return;}
  if(confirm('WiFi settings will be updated and the device will restart.')){
    var b=document.activeElement; if(b&&b.tagName==='BUTTON'){b.disabled=true;b.textContent='SAVING...';}
    cmd('wifi_cfg',{ssid:s,pass:p,hidden:h});
  }
}
function cmd(c,v){
  if(ws&&ws.readyState===1) {
    ws.send(JSON.stringify({cmd:c,value:v}));
    busy = Date.now() + 3000;
  }
}
function toggleMode(){
  if(!lastState||lastState.can_mode_switch_pending)return;
  cmd('mode',lastState.op_mode===1?0:1);
  var btn=document.getElementById('btnMode');
  if(btn){btn.disabled=true;btn.textContent='SWITCHING…';}
}

function setHwAuto(isAuto){
  var hwAuto=document.getElementById('btnHwAuto');
  var hwMan=document.getElementById('btnHwMan');
  if(hwAuto&&hwMan){
    hwAuto.className=isAuto?'seg-btn active':'seg-btn';
    hwMan.className=isAuto?'seg-btn':'seg-btn active';
  }
  document.querySelectorAll('#hwRow .mode-card').forEach(function(c){
    c.classList.toggle('disabled',isAuto);
  });
  if(isAuto){
    applyLocalHwSelection(0);
    cmd('hw_mode_auto',true);
  }else{
    var active=document.querySelector('#hwRow .mode-card.active');
    var val=active?parseInt(active.dataset.hw,10):(lastState?lastState.manual_hw_version:2);
    val=val||2;
    applyLocalHwSelection(val);
    cmd('manual_hw_version',val);
  }
}

function applyLocalHwSelection(val){
  offsetHw=val;
  var isHw3=val===2;
  var isHw4=val===3;
  var profCard=document.getElementById('profileCard');
  if(profCard)profCard.style.display=(isHw3||isHw4)?'block':'none';
  var profileAuto=lastState?!!lastState.profile_mode_auto:false;
  document.querySelectorAll('#modeRow .mode-card').forEach(function(c){
    var p=parseInt(c.dataset.val,10);
    var overMax=isHw3&&p>2;
    c.classList.toggle('disabled',profileAuto||overMax);
    if(overMax)c.classList.remove('active');
  });
  var autoMode=isHw3&&lastState?!!lastState.hw3_offset_auto:false;
  var percentMode=lastState?(isHw3?!!lastState.hw3_offset_percent_mode:!!lastState.hw4_offset_percent_mode):false;
  document.querySelectorAll('[id^="tierPct"]').forEach(function(pct){
    pct.max=isHw3?'33':'50';
  });
  syncOffsetLabels(isHw3);
  syncOffsetButtons(autoMode,percentMode,isHw3);
  syncOffsetMode(percentMode,isHw3,isHw4);
  if(isHw3&&autoMode){
    var row=document.getElementById('hw3OffsetRow');
    var pctBox=document.getElementById('pctOffsetBox');
    if(row)row.style.display='none';
    if(pctBox)pctBox.style.display='none';
  }
}

function selectHW(val){
  var card=document.querySelector('#hwRow .mode-card[data-hw="'+val+'"]');
  if(!card || card.classList.contains('disabled'))return;
  document.querySelectorAll('#hwRow .mode-card').forEach(function(c){
    c.classList.remove('active');
  });
  card.classList.add('active');
  applyLocalHwSelection(val);
  cmd('manual_hw_version',val);
}

function setProfileMode(isAuto){
  var btnAuto=document.getElementById('btnProfAuto');
  var btnMan=document.getElementById('btnProfMan');
  if(btnAuto&&btnMan){
    btnAuto.className=isAuto?'seg-btn active':'seg-btn';
    btnMan.className=isAuto?'seg-btn':'seg-btn active';
  }
  document.querySelectorAll('#modeRow .mode-card').forEach(function(c){
    c.classList.toggle('disabled',isAuto);
  });
  cmd('profile_mode_auto',isAuto);
}

function selectProfile(val){
  var card=document.querySelector('#modeRow .mode-card[data-val="'+val+'"]');
  if(!card || card.classList.contains('disabled'))return;
  document.querySelectorAll('#modeRow .mode-card').forEach(function(c){
    c.classList.remove('active');
  });
  card.classList.add('active');
  cmd('manual_profile',val);
}

function syncOffsetButtons(autoMode,percentMode,isHw3){
  var autoBtn=document.getElementById('btnOffAuto');
  var fixedBtn=document.getElementById('btnOffFixed');
  var pctBtn=document.getElementById('btnOffPct');
  if(autoBtn){
    autoBtn.style.display=isHw3?'inline-block':'none';
    autoBtn.className=autoMode?'seg-btn active':'seg-btn';
  }
  if(fixedBtn)fixedBtn.className=(!autoMode&&!percentMode)?'seg-btn active':'seg-btn';
  if(pctBtn){
    pctBtn.textContent=isHw3?'Limit':'%';
    pctBtn.className=(!autoMode&&percentMode)?'seg-btn active':'seg-btn';
  }
}

function syncOffsetLabels(isHw3){
  var activeLabel=document.getElementById('activeOffsetLabel');
  if(activeLabel)activeLabel.textContent=isHw3?'Active Base':'Active Offset';
  var hw3Unit=document.getElementById('hw3OffsetUnit');
  if(hw3Unit)hw3Unit.textContent='';
  for(var i=0;i<4;i++){
    var label=document.getElementById('tierOffsetLabel'+i);
    if(label)label.textContent=isHw3?('Base Raw '+(i+1)):('Offset '+(i+1));
    var unit=document.getElementById('tierUnit'+i);
    if(unit)unit.textContent=isHw3?'':'%';
  }
}

function setHw3OffsetMode(mode){
  if(offsetHw!==2)return;
  var autoMode=(mode==='auto');
  var percentMode=(mode==='percent');
  syncOffsetButtons(autoMode,percentMode,true);
  var row=document.getElementById('hw3OffsetRow');
  var pctBox=document.getElementById('pctOffsetBox');
  if(row)row.style.display=(!autoMode&&!percentMode)?'flex':'none';
  if(pctBox)pctBox.style.display=(!autoMode&&percentMode)?'block':'none';
  cmd('hw3_offset_auto',autoMode);
  if(!autoMode)cmd('hw3_offset_percent_mode',percentMode);
}

function setHw3Offset(value){
  var val=parseInt(value,10);
  if(isNaN(val))val=0;
  if(val<0)val=0;
  if(val>40)val=40;
  var input=document.getElementById('numHw3Offset');
  if(input)input.value=val;
  cmd('hw3_offset',hw3BaseToValue(val));
}

function setHw4Offset(value){
  var val=parseInt(value,10);
  if(isNaN(val))val=0;
  if(val<0)val=0;
  if(val>50)val=50;
  var input=document.getElementById('numHw4Offset');
  if(input)input.value=val;
  cmd('hw4_offset',val);
}

function syncOffsetMode(percent,showHw3,showHw4){
  var modeRow=document.getElementById('hw3OffsetModeRow');
  var hw3Row=document.getElementById('hw3OffsetRow');
  var fixedRow=document.getElementById('fixedOffsetRow');
  var pctBox=document.getElementById('pctOffsetBox');
  var limitRow=document.getElementById('dasLimitRow');
  var activeRow=document.getElementById('activeOffsetRow');
  if(modeRow)modeRow.style.display=(showHw3||showHw4)?'flex':'none';
  if(hw3Row)hw3Row.style.display=(showHw3&&!percent)?'flex':'none';
  if(fixedRow)fixedRow.style.display=(showHw4&&!percent)?'flex':'none';
  if(pctBox)pctBox.style.display=((showHw3||showHw4)&&percent)?'block':'none';
  if(limitRow)limitRow.style.display=(showHw3||showHw4)?'flex':'none';
  if(activeRow)activeRow.style.display=(showHw3||showHw4)?'flex':'none';
}

function setOffsetMode(percent){
  if(offsetHw===2){
    setHw3OffsetMode(percent?'percent':'fixed');
    return;
  }
  syncOffsetButtons(false,percent,false);
  syncOffsetMode(percent,false,true);
  cmd('hw4_offset_percent_mode',!!percent);
}

function setOffsetTier(idx,field,value){
  var val=parseInt(value,10);
  if(isNaN(val))val=0;
  if(field==='limit'){
    if(val<0)val=0;
    if(val>155)val=155;
  }else{
    if(val<0)val=0;
    var maxValue=(offsetHw===2)?40:50;
    if(val>maxValue)val=maxValue;
  }
  var input=document.getElementById((field==='limit'?'tierLimit':'tierPct')+idx);
  if(input)input.value=val;
  var sendVal=(offsetHw===2&&field!=='limit')?hw3BaseToValue(val):val;
  cmd((offsetHw===2?'hw3_tier':'hw4_tier')+idx+'_'+field,sendVal);
}

function conn(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=function(){
    document.getElementById('dot').className='cdot';
    document.getElementById('connErr').style.display='none';
    clearTimeout(rt);
  };
  ws.onmessage=function(e){ try{var d=JSON.parse(e.data);lastState=d;initWifi(d);upd(d);}catch(x){} };
  ws.onclose=function(){
    document.getElementById('dot').className='cdot off';
    document.getElementById('connErr').style.display='block';
    rt=setTimeout(conn,2000);
  };
  ws.onerror=function(){ ws.close(); };
}
conn();
</script>
</body>
</html>
)rawliteral";

// ── JSON helpers ──────────────────────────────────────────────────────────────
static bool json_appendf(char *&p, char *end, const char *fmt, ...) {
    if (p >= end) return false;
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(p, (size_t)(end - p), fmt, args);
    va_end(args);
    if (n < 0) {
        if (p < end) *p = '\0';
        return false;
    }
    if (n >= (end - p)) {
        p = end - 1;
        *p = '\0';
        return false;
    }
    p += n;
    return true;
}

static bool json_append_escaped(char *&p, char *end, const char *s) {
    if (s == nullptr) return true;
    for (; *s; ++s) {
        const char *esc = nullptr;
        if (*s == '"') esc = "\\\"";
        else if (*s == '\\') esc = "\\\\";

        if (esc != nullptr) {
            while (*esc) {
                if (p >= end - 1) { *p = '\0'; return false; }
                *p++ = *esc++;
            }
        } else {
            if (p >= end - 1) { *p = '\0'; return false; }
            *p++ = *s;
        }
    }
    if (p < end) *p = '\0';
    return true;
}

// ── JSON builder ──────────────────────────────────────────────────────────────
static size_t build_json(char *out, size_t out_len) {
    if (out == nullptr || out_len == 0) return 0;

    FSDState state;
    if (!state_copy(&state)) {
        snprintf(out, out_len, "{}");
        return strlen(out);
    }

    uint32_t uptime_s = (millis() - g_start_ms) / 1000;
    bool can_vehicle_detected = false;
    if (state.rx_count > 0) {
        can_vehicle_detected = (millis() - g_last_can_seen_ms) <= CAN_VEHICLE_ALIVE_MS;
    }

    // BMS sub-object
    char bms[128];
    if (state.bms_seen) {
        snprintf(bms, sizeof(bms),
            "{\"seen\":true,\"voltage\":%.1f,\"current\":%.1f,"
            "\"soc\":%.1f,\"temp_min\":%d,\"temp_max\":%d}",
            state.pack_voltage_v,
            state.pack_current_a,
            state.soc_percent,
            (int)state.batt_temp_min_c,
            (int)state.batt_temp_max_c);
    } else {
        strcpy(bms, "{\"seen\":false}");
    }

    char ota_part[128] = {};
    {
        const esp_partition_t *running = esp_ota_get_running_partition();
        const char *running_label = running ? running->label : "unknown";
        esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;
        if (running) esp_ota_get_state_partition(running, &ota_state);
        bool has_ota = (running &&
            (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 ||
             running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1));
        snprintf(ota_part, sizeof(ota_part),
            "{\"running\":\"%s\",\"state\":%d,\"has_ota\":%s}",
            running_label, (int)ota_state, has_ota ? "true" : "false");
    }

    // fps as fixed-point string
    char fps_s[12];
    snprintf(fps_s, sizeof(fps_s), "%.1f", g_fps);
    esp_reset_reason_t reset_reason = esp_reset_reason();
    uint32_t web_stack_free_words =
        (g_web_task_handle != nullptr) ? uxTaskGetStackHighWaterMark(g_web_task_handle) : 0;

    char *p = out;
    char *end = out + out_len;
#define JAPP(...) json_appendf(p, end, __VA_ARGS__)
    JAPP("{");
    JAPP("\"fsd_enabled\":%s,", state.fsd_enabled ? "true" : "false");
    JAPP("\"op_mode\":%d,", (int)state.op_mode);
    JAPP("\"can_mode_switch_pending\":%s,", state.can_mode_switch_pending ? "true" : "false");
    JAPP("\"can_mode_switch_failed\":%s,", state.can_mode_switch_failed ? "true" : "false");
    JAPP("\"can_mode_switch_request_id\":%lu,", (unsigned long)state.can_mode_switch_request_id);
    JAPP("\"hw_version\":%d,", (int)state.hw_version);
    JAPP("\"hw_mode_auto\":%s,", state.hw_mode_auto ? "true" : "false");
    JAPP("\"manual_hw_version\":%d,", (int)state.manual_hw_version);
    JAPP("\"speed_profile\":%d,", (int)state.speed_profile);
    JAPP("\"profile_mode_auto\":%s,", state.profile_mode_auto ? "true" : "false");
    JAPP("\"manual_speed_profile\":%d,", (int)state.manual_speed_profile);
    JAPP("\"hw3_offset_auto\":%s,", state.hw3_offset_auto ? "true" : "false");
    JAPP("\"hw3_offset\":%d,", (int)state.hw3_offset);
    JAPP("\"hw3_offset_percent_mode\":%s,", state.hw3_offset_percent_mode ? "true" : "false");
    JAPP("\"hw3_offset_active\":%d,", (int)state.hw3_offset_active);
    JAPP("\"hw4_offset\":%d,", (int)state.hw4_offset);
    JAPP("\"hw4_offset_percent_mode\":%s,", state.hw4_offset_percent_mode ? "true" : "false");
    JAPP("\"hw4_offset_active\":%d,", (int)state.hw4_offset_active);
    JAPP("\"das_speed_limit_kph\":%d,", (int)state.das_speed_limit_active * 5);
    for (uint8_t i = 0; i < 4; ++i) {
        JAPP("\"hw3_tier%u_limit\":%d,", i, (int)state.hw3_offset_tier_limit[i]);
        JAPP("\"hw3_tier%u_percent\":%d,", i, (int)state.hw3_offset_tier_percent[i]);
        JAPP("\"hw4_tier%u_limit\":%d,", i, (int)state.hw4_offset_tier_limit[i]);
        JAPP("\"hw4_tier%u_percent\":%d,", i, (int)state.hw4_offset_tier_percent[i]);
    }
    JAPP("\"ota\":%s,", state.tesla_ota_in_progress ? "true" : "false");
    JAPP("\"nag_killer\":%s,", state.nag_killer ? "true" : "false");
    JAPP("\"bms_output\":%s,", state.bms_output ? "true" : "false");
    JAPP("\"force_fsd\":%s,", state.force_fsd ? "true" : "false");
    JAPP("\"suppress_speed_chime\":%s,", state.suppress_speed_chime ? "true" : "false");
    JAPP("\"china_mode\":%s,", state.china_mode ? "true" : "false");
    JAPP("\"tlssc_restore\":%s,", state.tlssc_restore ? "true" : "false");
    JAPP("\"can_vehicle_detected\":%s,", can_vehicle_detected ? "true" : "false");
    JAPP("\"bms_hv_seen\":%lu,", (unsigned long)state.seen_bms_hv);
    JAPP("\"bms_soc_seen\":%lu,", (unsigned long)state.seen_bms_soc);
    JAPP("\"bms_thermal_seen\":%lu,", (unsigned long)state.seen_bms_thermal);
    JAPP("\"rx_count\":%lu,", (unsigned long)state.rx_count);
    JAPP("\"tx_count\":%lu,", (unsigned long)state.frames_modified);
    JAPP("\"tx_sent\":%lu,", (unsigned long)state.frames_sent);
    JAPP("\"tx_failed\":%lu,", (unsigned long)state.tx_fail_count);
    JAPP("\"debug_log\":\"");
    json_append_escaped(p, end, state.web_debug_log);
    JAPP("\",");
    JAPP("\"rx_missed\":%lu,", (unsigned long)state.rx_missed_count);
    JAPP("\"bus_errors\":%lu,", (unsigned long)state.bus_error_count);
    JAPP("\"rx_overrun\":%lu,", (unsigned long)state.rx_overrun_count);
    JAPP("\"twai_restarts\":%lu,", (unsigned long)state.twai_restart_count);
    JAPP("\"twai_state\":%u,", (unsigned)state.twai_state);
    JAPP("\"crc_errors\":%lu,", (unsigned long)(state.rx_missed_count + state.bus_error_count + state.rx_overrun_count));
    JAPP("\"fps\":%s,", fps_s);
    JAPP("\"bms\":%s,", bms);
    JAPP("\"uptime_s\":%lu,", (unsigned long)uptime_s);
    JAPP("\"reset_reason\":\"%s\",", reset_reason_name(reset_reason));
    JAPP("\"reset_reason_code\":%d,", (int)reset_reason);
    JAPP("\"chip_temp_valid\":%s,", state.chip_temp_valid ? "true" : "false");
    JAPP("\"chip_temp_c\":%.1f,", state.chip_temp_c);
    JAPP("\"free_heap\":%lu,", (unsigned long)esp_get_free_heap_size());
    JAPP("\"min_free_heap\":%lu,", (unsigned long)esp_get_minimum_free_heap_size());
    JAPP("\"web_stack_free_words\":%lu,", (unsigned long)web_stack_free_words);
    JAPP("\"can_stack_free_words\":%lu,", (unsigned long)state.can_stack_free_words);
    JAPP("\"fw_build\":\"%s %s\",", __DATE__, __TIME__);
    JAPP("\"sd_available\":%s,", k_sd_available ? "true" : "false");
    JAPP("\"can_dump\":%s,", can_dump_active() ? "true" : "false");
    JAPP("\"wifi_ssid\":\"");
    json_append_escaped(p, end, state.wifi_ssid);
    JAPP("\",");
    JAPP("\"wifi_pass\":\"***\",");
    JAPP("\"wifi_hidden\":%s,", state.wifi_hidden ? "true" : "false");
    JAPP("\"wifi_clients\":%d,", (int)WiFi.softAPgetStationNum());
    JAPP("\"ota_partition\":%s", ota_part);
    JAPP("}");
#undef JAPP
    return strlen(out);
}

// ── WebSocket event handler ───────────────────────────────────────────────────
static void ws_event(uint8_t num, WStype_t type,
                     uint8_t *payload, size_t length)
{
    if (type == WStype_CONNECTED) {
        // Avoid a large synchronous send during the connect handshake.
        // The normal 1 Hz broadcast will deliver state within one tick.
        return;
    }

    if (type != WStype_TEXT || g_state == nullptr || length == 0) return;

    // Use a slightly more robust way to find the value after the second colon
    char buf[256] = {};
    size_t n = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 1;
    memcpy(buf, payload, n);

    // Find the "value" part of {"cmd":"xxx","value":yyy}
    const char *vptr = strstr(buf, "\"value\":");
    if (vptr) vptr = strstr(vptr, ":") + 1;

    if (strstr(buf, "\"mode\"")) {
        if (vptr && g_can_command_queue != nullptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int requested = atoi(vptr);
            OpMode requested_mode = requested == (int)OpMode_Active
                ? OpMode_Active : OpMode_ListenOnly;

            bool already_pending;
            uint32_t request_id;
            state_enter();
            already_pending = g_state->can_mode_switch_pending;
            request_id = ++g_can_request_id;
            if (!already_pending) {
                g_state->can_mode_switch_pending = true;
                g_state->can_mode_switch_failed = false;
                g_state->can_mode_switch_request_id = request_id;
            }
            state_exit();

            if (!already_pending) {
                CanCommand command = {CanCommandType::SetMode, requested_mode, request_id};
                if (xQueueSend(g_can_command_queue, &command, 0) != pdTRUE) {
                    state_enter();
                    if (g_state->can_mode_switch_request_id == request_id) {
                        g_state->can_mode_switch_pending = false;
                        g_state->can_mode_switch_failed = true;
                    }
                    state_exit();
                    Serial.println("[Web] CAN mode command queue full");
                }
            }
        }
    } else if (strstr(buf, "\"nag\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->nag_killer = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] NAG Killer: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"bms\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->bms_output = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] BMS output: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"tlssc_restore\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->tlssc_restore = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] TLSSC Restore: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"force_fsd\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->force_fsd = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Force FSD: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"suppress_speed_chime\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->suppress_speed_chime = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Suppress Speed Chime: %s\n",
                enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"china_mode\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->china_mode = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] China Mode: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"hw_mode_auto\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->hw_mode_auto = enabled;
            if (enabled) {
                fsd_apply_hw_version(g_state, TeslaHW_Unknown);
            } else {
                fsd_apply_hw_version(g_state, g_state->manual_hw_version);
            }
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Hardware Mode: %s\n",
                enabled ? "Auto Detect" : "Manual Override");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"manual_hw_version\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int val = atoi(vptr);
            if (val >= (int)TeslaHW_Legacy && val <= (int)TeslaHW_HW4) {
                FSDState saved;
                state_enter();
                g_state->hw_mode_auto = false;
                g_state->manual_hw_version = (TeslaHWVersion)val;
                fsd_apply_hw_version(g_state, g_state->manual_hw_version);
                saved = *g_state;
                state_exit();
                const char *names[] = {"Unknown", "Legacy", "HW3", "HW4"};
                Serial.printf("[Web] Manual Hardware: %s\n", names[val]);
                prefs_save(&saved);
            }
        }
    } else if (strstr(buf, "\"profile_mode_auto\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->profile_mode_auto = enabled;
            fsd_apply_hw_version(g_state, g_state->hw_version);
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] Profile Mode: %s\n",
                enabled ? "Auto (Follow Distance)" : "Manual (Web UI)");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"manual_profile\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int val = atoi(vptr);
            if (val >= 0 && val <= 4) {
                FSDState saved;
                bool accepted = false;
                state_enter();
                uint8_t max_profile = (g_state->hw_version == TeslaHW_HW4) ? 4 : 2;
                if (val <= max_profile) {
                    g_state->manual_speed_profile = (uint8_t)val;
                    if (!g_state->profile_mode_auto) {
                        g_state->speed_profile = val;
                    }
                    saved = *g_state;
                    accepted = true;
                }
                state_exit();
                const char *names[] = {"Chill", "Normal", "Hurry", "Max", "Sloth"};
                if (accepted) {
                    Serial.printf("[Web] Manual Profile: %d (%s)\n", val, names[val]);
                    prefs_save(&saved);
                } else {
                    Serial.printf("[Web] Ignored unsupported profile %d for current HW\n", val);
                }
            }
        }
    } else if (strstr(buf, "\"hw3_offset_auto\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->hw3_offset_auto = enabled;
            if (enabled) {
                g_state->hw3_offset_active = 0;
                g_state->hw3_offset_auto_valid = false;
            } else {
                g_state->speed_offset = g_state->hw3_offset;
                g_state->hw3_offset_active = g_state->hw3_offset;
                g_state->hw3_offset_auto_valid = false;
            }
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] HW3 Offset Source: %s\n",
                enabled ? "Auto" : "Manual");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"hw3_offset_percent_mode\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->hw3_offset_auto = false;
            g_state->hw3_offset_percent_mode = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] HW3 Offset Mode: %s\n",
                enabled ? "Limit Tiers" : "Fixed");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"hw3_offset\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int val = atoi(vptr);
            if (val < 0) val = 0;
            if (val > 200) val = 200;
            FSDState saved;
            state_enter();
            g_state->hw3_offset = (uint8_t)val;
            if (!g_state->hw3_offset_auto) {
                g_state->speed_offset = val;
                g_state->hw3_offset_active = (uint8_t)val;
            }
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] HW3 Speed Offset Value: %d\n", val);
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"hw3_tier")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int val = atoi(vptr);
            for (uint8_t i = 0; i < 4; ++i) {
                char key[28];
                snprintf(key, sizeof(key), "\"hw3_tier%u_limit\"", i);
                if (strstr(buf, key)) {
                    if (val < 0) val = 0;
                    if (val > 155) val = 155;
                    FSDState saved;
                    state_enter();
                    g_state->hw3_offset_tier_limit[i] = (uint8_t)val;
                    saved = *g_state;
                    state_exit();
                    Serial.printf("[Web] HW3 Offset Tier %u Limit: %d km/h\n", i + 1, val);
                    prefs_save(&saved);
                    break;
                }
                snprintf(key, sizeof(key), "\"hw3_tier%u_percent\"", i);
                if (strstr(buf, key)) {
                    if (val < 0) val = 0;
                    if (val > 200) val = 200;
                    FSDState saved;
                    state_enter();
                    g_state->hw3_offset_tier_percent[i] = (uint8_t)val;
                    saved = *g_state;
                    state_exit();
                    Serial.printf("[Web] HW3 Offset Tier %u Value: %d\n", i + 1, val);
                    prefs_save(&saved);
                    break;
                }
            }
        }
    } else if (strstr(buf, "\"hw4_offset_percent_mode\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            FSDState saved;
            bool enabled = (strncmp(vptr, "true", 4) == 0);
            state_enter();
            g_state->hw4_offset_percent_mode = enabled;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] HW4 Offset Mode: %s\n",
                enabled ? "Percent" : "Fixed");
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"hw4_tier")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int val = atoi(vptr);
            for (uint8_t i = 0; i < 4; ++i) {
                char key[28];
                snprintf(key, sizeof(key), "\"hw4_tier%u_limit\"", i);
                if (strstr(buf, key)) {
                    if (val < 0) val = 0;
                    if (val > 155) val = 155;
                    FSDState saved;
                    state_enter();
                    g_state->hw4_offset_tier_limit[i] = (uint8_t)val;
                    saved = *g_state;
                    state_exit();
                    Serial.printf("[Web] HW4 Offset Tier %u Limit: %d km/h\n", i + 1, val);
                    prefs_save(&saved);
                    break;
                }
                snprintf(key, sizeof(key), "\"hw4_tier%u_percent\"", i);
                if (strstr(buf, key)) {
                    if (val < 0) val = 0;
                    if (val > 50) val = 50;
                    FSDState saved;
                    state_enter();
                    g_state->hw4_offset_tier_percent[i] = (uint8_t)val;
                    saved = *g_state;
                    state_exit();
                    Serial.printf("[Web] HW4 Offset Tier %u Percent: %d%%\n", i + 1, val);
                    prefs_save(&saved);
                    break;
                }
            }
        }
    } else if (strstr(buf, "\"hw4_offset\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int val = atoi(vptr);
            if (val < 0) val = 0;
            if (val > 50) val = 50;
            FSDState saved;
            state_enter();
            g_state->hw4_offset = (uint8_t)val;
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] HW4 Speed Offset: %d%%\n", val);
            prefs_save(&saved);
        }
    } else if (strstr(buf, "\"dump\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool want = (strncmp(vptr, "true", 4) == 0);
            if (!k_sd_available) {
                Serial.println("[Web] CAN Dump unavailable on this board");
            } else if (want) {
                can_dump_start();
                Serial.println("[Web] CAN Dump: START");
            } else {
                can_dump_stop();
                Serial.println("[Web] CAN Dump: STOP");
            }
        }
    } else if (strstr(buf, "\"wifi_cfg\"")) {
        // Find the "value":{ object start
        const char *vobj = strstr(buf, "\"value\":");
        if (vobj) {
            char *s = strstr(vobj, "\"ssid\":\"");
            char *p = strstr(vobj, "\"pass\":\"");
            char *h = strstr(vobj, "\"hidden\":");
            FSDState saved;
            state_enter();
            if (s) {
                s += 8;
                char *end = strchr(s, '\"');
                if (end) {
                    int len = end - s;
                    if (len > 32) len = 32;
                    if (memchr(s, '\\', len) == nullptr) {
                        memcpy(g_state->wifi_ssid, s, len);
                        g_state->wifi_ssid[len] = '\0';
                    }
                }
            }
            if (p) {
                p += 8;
                char *end = strchr(p, '\"');
                if (end) {
                    int len = end - p;
                    if (len > 64) len = 64;
                    if (memchr(p, '\\', len) == nullptr &&
                        !(len == 3 && memcmp(p, "***", 3) == 0)) {
                        memcpy(g_state->wifi_pass, p, len);
                        g_state->wifi_pass[len] = '\0';
                    }
                }
            }
            if (h) {
                h += 9;
                while (*h == ' ' || *h == ':') h++;
                if (strncmp(h, "true", 4) == 0) g_state->wifi_hidden = true;
                else if (strncmp(h, "false", 5) == 0) g_state->wifi_hidden = false;
            }
            saved = *g_state;
            state_exit();
            Serial.printf("[Web] WiFi config: SSID=\"%s\" PASS=*** HIDDEN=%d\n",
                saved.wifi_ssid, saved.wifi_hidden);
            prefs_save(&saved);
            delay(500);
            ESP.restart();
        }
    }
}

// ── HTTP handlers ─────────────────────────────────────────────────────────────
static void handle_root() {
    g_http.send_P(200, "text/html", WEB_HTML);
}

static void handle_status() {
    if (g_state == nullptr) { g_http.send(503, "application/json", "{}"); return; }
    build_json(g_json_buf, sizeof(g_json_buf));
    g_http.send(200, "application/json", g_json_buf);
}

static void handle_captive_portal() {
    String ip = WiFi.softAPIP().toString();
    String host = g_http.hostHeader();
    if (host.length() > 0 && host != ip && host != ip + ":80") {
        g_http.sendHeader("Location", "http://" + ip + "/", true);
        g_http.send(302, "text/plain", "");
        return;
    }
    handle_root();
}

static void handle_probe_redirect() {
    String ip = WiFi.softAPIP().toString();
    g_http.sendHeader("Location", "http://" + ip + "/", true);
    g_http.send(302, "text/plain", "");
}

static void handle_sdformat() {
    String result = sd_format_card();
    g_http.send(200, "application/json", result);
}

static void handle_restart() {
    g_http.send(200, "text/plain", "OK");
    delay(500);
    ESP.restart();
}

// ── OTA Update handlers ───────────────────────────────────────────────────────
static size_t ota_total_size = 0;
static bool ota_error_flag = false;

static void handle_ota_upload() {
    HTTPUpload& upload = g_http.upload();

    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA] Start: %s\n", upload.filename.c_str());
        ota_error_flag = false;
        ota_total_size = 0;

        if (!upload.filename.endsWith(".bin")) {
            Serial.println("[OTA] ERROR: File must be .bin");
            ota_error_flag = true;
            return;
        }

        size_t max_size = UPDATE_SIZE_UNKNOWN;
        const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
        if (partition != NULL) {
            max_size = partition->size;
            Serial.printf("[OTA] Target partition: %s, size: %u bytes\n",
                partition->label, (unsigned)max_size);
        }

        if (!Update.begin(max_size, U_FLASH)) {
            Update.printError(Serial);
            Serial.println("[OTA] ERROR: Update.begin() failed");
            ota_error_flag = true;
            return;
        }

        Serial.println("[OTA] Update started successfully");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (ota_error_flag) return;

        size_t written = Update.write(upload.buf, upload.currentSize);
        if (written != upload.currentSize) {
            Update.printError(Serial);
            Serial.printf("[OTA] ERROR: Write failed, expected %u, wrote %u\n",
                upload.currentSize, (unsigned)written);
            ota_error_flag = true;
            return;
        }

        ota_total_size = upload.totalSize;
        if (ota_total_size % 65536 == 0) {
            Serial.printf("[OTA] Progress: %u bytes\n", (unsigned)ota_total_size);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (ota_error_flag) {
            Serial.println("[OTA] Upload aborted due to previous error");
            Update.abort();
            return;
        }

        if (Update.end(true)) {
            Serial.printf("[OTA] Success: %u bytes total\n", (unsigned)ota_total_size);
            if (!Update.isFinished()) {
                Serial.println("[OTA] ERROR: Update not finished properly");
                ota_error_flag = true;
            }
        } else {
            Update.printError(Serial);
            Serial.println("[OTA] ERROR: Update.end() failed");
            ota_error_flag = true;
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Serial.println("[OTA] Upload aborted by client");
        Update.abort();
        ota_error_flag = true;
    }
}

static void handle_ota_done() {
    if (ota_error_flag || Update.hasError()) {
        String error_msg = "FAIL: ";
        if (Update.hasError()) {
            error_msg += "Error code " + String(Update.getError());
        } else {
            error_msg += "Upload error";
        }

        Serial.printf("[OTA] %s\n", error_msg.c_str());
        g_http.send(500, "text/plain", error_msg);
        Update.abort();
        return;
    }

    g_http.send(200, "text/plain", "OK");

    Serial.println("[OTA] Firmware update successful!");
    Serial.println("[OTA] Rebooting in 2 seconds...");

    delay(2000);
    ESP.restart();
}

// ── Public API ────────────────────────────────────────────────────────────────
void web_dashboard_init(FSDState *state, QueueHandle_t can_command_queue,
                        portMUX_TYPE *state_mux) {
    g_state       = state;
    g_state_mux   = state_mux;
    g_can_command_queue = can_command_queue;
    g_start_ms    = millis();
    g_last_fps_ms = millis();
    FSDState initial_state;
    bool has_state = state_copy(&initial_state);
    g_last_rx     = has_state ? initial_state.rx_count : 0;
    g_last_can_seen_ms = (has_state && initial_state.rx_count > 0) ? millis() : 0;

    g_http.on("/",           HTTP_GET,  handle_root);
    g_http.on("/api/status", HTTP_GET,  handle_status);
    g_http.on("/sdformat",   HTTP_GET,  handle_sdformat);
    g_http.on("/restart",    HTTP_GET,  handle_restart);
    g_http.on("/update",     HTTP_POST, handle_ota_done, handle_ota_upload);
    g_http.on("/generate_204", HTTP_GET, handle_probe_redirect);
    g_http.on("/gen_204", HTTP_GET, handle_probe_redirect);
    g_http.on("/hotspot-detect.html", HTTP_GET, handle_probe_redirect);
    g_http.on("/library/test/success.html", HTTP_GET, handle_probe_redirect);
    g_http.on("/connecttest.txt", HTTP_GET, handle_probe_redirect);
    g_http.on("/success.txt", HTTP_GET, handle_probe_redirect);
    g_http.on("/redirect", HTTP_GET, handle_probe_redirect);
    g_http.on("/fwlink", HTTP_GET, handle_probe_redirect);
    g_http.onNotFound(handle_captive_portal);
    g_http.begin();

    g_ws.begin();
    g_ws.onEvent(ws_event);

    if (g_web_task_handle == nullptr) {
        BaseType_t web_task_ok = xTaskCreatePinnedToCore(
            web_server_task,
            "WebServer",
            12288,
            nullptr,
            1,
            &g_web_task_handle,
            0);
        if (web_task_ok == pdPASS) {
            Serial.println("[Web] Task created on Core 0");
        } else {
            Serial.println("[Web] ERROR: could not create Core 0 task");
            g_web_task_handle = nullptr;
        }
    }

    Serial.println("[Web] HTTP :80  WS :81 — ready");
}

static void web_server_task(void *param) {
    (void)param;
    Serial.printf("[Web] Task started on Core %d\n", xPortGetCoreID());

    while (true) {
        if (g_state != nullptr) {
            wifi_process_dns();
            g_http.handleClient();
            g_ws.loop();

            // FPS calculation + 1 Hz WebSocket broadcast
            uint32_t now = millis();
            if ((now - g_last_fps_ms) >= 1000u) {
                FSDState state;
                if (!state_copy(&state)) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                uint32_t rx = state.rx_count;
                float    dt = (now - g_last_fps_ms) / 1000.0f;
                if (rx != g_last_rx) g_last_can_seen_ms = now;
                g_fps        = (float)(rx - g_last_rx) / dt;
                g_last_rx    = rx;
                g_last_fps_ms = now;

                if (g_ws.connectedClients(false) > 0) {
                    size_t len = build_json(g_json_buf, sizeof(g_json_buf));
                    g_ws.broadcastTXT(g_json_buf, len);
                    taskYIELD();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void web_dashboard_update() {
    // Web work runs in web_server_task on Core 0.
}
