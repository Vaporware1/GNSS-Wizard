// ============================================================
//  GNSS WIZARD HUD — ESP32-WROOM-32  (standalone Wi-Fi web HUD)
// ============================================================
//  What it does:
//   - ESP32 hosts its OWN open Wi-Fi AP   ->  "GNSS-WIZARD" (no password)
//   - Join it on any phone/tablet, open   ->  http://192.168.4.1
//   - Tactical web HUD shows: azimuth (magnetic), MGRS grid,
//     satellite count, HDOP, GPS fix -- with a wizard at center.
//   - Magnetic declination (+E / -W) is set on the page and saved to
//     flash; the TRUE toggle applies it in the browser. The device
//     itself always reports RAW magnetic heading.
//
//  Required library (Arduino Library Manager):
//    ->  TinyGPSPlus  by Mikal Hart
//
//  Board:      ESP32 Dev Module
//  Partition:  default should fit now (Bluetooth removed = small binary).
//              If it ever complains it won't fit, pick
//              "No OTA (2MB APP / 2MB SPIFFS)".
//  Upload:     speed 115200; hold BOOT during upload if it won't connect.
//
//  Wiring (ARK DAN L1/L5 6-pin JST-GH -> ESP32):
//    5V  -> VIN          TX -> GPIO16 (RX2)        RX -> GPIO17 (TX2)
//    SCL -> GPIO22       SDA-> GPIO21              GND-> GND
//  GPS: 38400 baud UART.   Magnetometer (IIS2MDC): I2C addr 0x1E.
//
//  NOTE: if heading reads -1 / frozen, the magnetometer isn't talking.
//  Reseat the SDA/SCL solder joints (that was the v2 dropout cause).
// ============================================================

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

// ── Access point ───────────────────────────────────────────
#define AP_SSID    "GNSS-WIZARD"     // open network, no password

// ── GPS ────────────────────────────────────────────────────
#define GPS_RX     16
#define GPS_TX     17
#define GPS_BAUD   38400

// ── IIS2MDC magnetometer ───────────────────────────────────
#define MAG_ADDR   0x1E
#define CFG_REG_A  0x60
#define CFG_REG_C  0x62
#define STATUS_REG 0x67
#define OUTX_L_REG 0x68

#define CAL_MS     15000
#define SMOOTH_N   10

// ══════════════════════════════════════════════════════════
//  CONFIG  (saved to NVS flash)
// ══════════════════════════════════════════════════════════
struct Config {
  float declination = 0.0f;   // +East / -West
  float calOX = 0.0f, calOY = 0.0f, calSX = 1.0f, calSY = 1.0f;
  float calRadius = 0.0f;     // clean-field horizontal magnitude (interference ref)
  bool  calValid = false;
};
Config      cfg;
Preferences prefs;

void loadConfig() {
  prefs.begin("gnss", true);
  cfg.declination = prefs.getFloat("decl",     0.0f);
  cfg.calOX       = prefs.getFloat("calOX",    0.0f);
  cfg.calOY       = prefs.getFloat("calOY",    0.0f);
  cfg.calSX       = prefs.getFloat("calSX",    1.0f);
  cfg.calSY       = prefs.getFloat("calSY",    1.0f);
  cfg.calRadius   = prefs.getFloat("calR",     0.0f);
  cfg.calValid    = prefs.getBool ("calValid", false);
  prefs.end();
}
void saveDecl() {
  prefs.begin("gnss", false);
  prefs.putFloat("decl", cfg.declination);
  prefs.end();
}
void saveCal() {
  prefs.begin("gnss", false);
  prefs.putFloat("calOX", cfg.calOX);
  prefs.putFloat("calOY", cfg.calOY);
  prefs.putFloat("calSX", cfg.calSX);
  prefs.putFloat("calSY", cfg.calSY);
  prefs.putFloat("calR",  cfg.calRadius);
  prefs.putBool ("calValid", cfg.calValid);
  prefs.end();
}

// ══════════════════════════════════════════════════════════
//  GLOBALS
// ══════════════════════════════════════════════════════════
TinyGPSPlus gps;
WebServer   server(80);

bool  magOK       = false;
bool  calibrating = false;
float lastHdg     = 0.0f;     // last good RAW magnetic heading

// interference + calibration-quality state
bool  magWarn    = false;     // live field-anomaly flag
float magDevPct  = 0.0f;      // current field deviation from clean radius (0..1)
float lastCalCov = 0.0f;      // angular coverage of most recent cal (0..1)
bool  lastCalOK  = true;      // did the most recent cal pass the quality gate
#define FIELD_THRESH   0.30f  // >30% field deviation = interference / heavy tilt
#define CAL_MIN_COV    0.70f  // require >=70% of the circle swept during cal
#define CAL_MIN_RATIO  0.45f  // min X/Y range ratio (reject distorted cal)

// circular heading smoother
float hSin[SMOOTH_N] = {}, hCos[SMOOTH_N] = {};
int   hIdx = 0;  bool hFull = false;
float smoothHdg(float deg) {
  float r = deg * M_PI / 180.0f;
  hSin[hIdx] = sinf(r); hCos[hIdx] = cosf(r);
  hIdx = (hIdx + 1) % SMOOTH_N;
  if (!hIdx) hFull = true;
  int n = hFull ? SMOOTH_N : hIdx;
  float ss = 0, sc = 0;
  for (int i = 0; i < n; i++) { ss += hSin[i]; sc += hCos[i]; }
  float a = atan2f(ss / n, sc / n) * 180.0f / M_PI;
  return a < 0 ? a + 360.0f : a;
}

// ══════════════════════════════════════════════════════════
//  MAGNETOMETER
// ══════════════════════════════════════════════════════════
void writeMag(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
bool initMag() {
  Wire.beginTransmission(MAG_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[MAG] not found on I2C -- check SDA/SCL solder joints");
    return false;
  }
  writeMag(CFG_REG_A, 0x00);   // 10 Hz continuous
  writeMag(CFG_REG_C, 0x10);   // block data update on
  delay(20);
  Serial.println("[MAG] IIS2MDC online");
  return true;
}
bool readMagRaw(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(STATUS_REG);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)1);
  if (!Wire.available()) return false;
  if (!(Wire.read() & 0x01)) return false;

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(OUTX_L_REG);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return false;

  x = Wire.read() | Wire.read() << 8;
  y = Wire.read() | Wire.read() << 8;
  z = Wire.read() | Wire.read() << 8;
  return true;
}
void runCal() {
  Serial.printf("[CAL] rotate device LEVEL for %d s...\n", CAL_MS / 1000);
  float mnX=1e9,mxX=-1e9,mnY=1e9,mxY=-1e9; int16_t x,y,z;
  bool bins[36]; for (int i=0;i<36;i++) bins[i]=false;
  int binsHit=0; long samples=0;
  unsigned long t = millis();
  while (millis() - t < CAL_MS) {
    if (readMagRaw(x,y,z)) {
      if(x<mnX)mnX=x; if(x>mxX)mxX=x;
      if(y<mnY)mnY=y; if(y>mxY)mxY=y;
      samples++;
      // angular coverage vs running center -> confirms a FULL rotation
      float mx=(mxX+mnX)*0.5f, my=(mxY+mnY)*0.5f;
      float ang=atan2f((float)y-my,(float)x-mx);   // -pi..pi
      int b=(int)((ang+(float)M_PI)/(2.0f*(float)M_PI)*36.0f);
      if(b<0)b=0; if(b>35)b=35;
      if(!bins[b]){bins[b]=true;binsHit++;}
    }
    server.handleClient();                         // keep web UI alive during cal
    digitalWrite(LED_BUILTIN, (millis()/250)%2);   // blink while calibrating
    delay(20);
  }
  float oX=(mxX+mnX)*0.5f, oY=(mxY+mnY)*0.5f;
  float rx=(mxX-mnX)*0.5f, ry=(mxY-mnY)*0.5f, av=(rx+ry)*0.5f;
  float cov=binsHit/36.0f;
  float ratio=(rx>0&&ry>0)?((rx<ry)?rx/ry:ry/rx):0.0f;
  lastCalCov=cov;

  // quality gate: enough rotation, sane circle, non-degenerate field
  bool ok = (av>5.0f) && (cov>=CAL_MIN_COV) && (ratio>=CAL_MIN_RATIO);
  lastCalOK=ok;
  Serial.printf("[CAL] samples=%ld coverage=%.0f%% axisRatio=%.2f radius=%.1f -> %s\n",
                samples, cov*100.0f, ratio, av, ok?"ACCEPTED":"REJECTED");

  if (ok) {
    cfg.calOX=oX; cfg.calOY=oY;
    cfg.calSX=(rx>0)?av/rx:1.0f;
    cfg.calSY=(ry>0)?av/ry:1.0f;
    cfg.calRadius=av;
    cfg.calValid=true;
    saveCal();
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.printf("[CAL] saved  OX=%.1f OY=%.1f SX=%.3f SY=%.3f R=%.1f\n",
                  cfg.calOX,cfg.calOY,cfg.calSX,cfg.calSY,cfg.calRadius);
  } else {
    // keep any previous good cal; just tell the user to redo it properly
    digitalWrite(LED_BUILTIN, cfg.calValid?HIGH:LOW);
    Serial.println("[CAL] REJECTED -> kept previous cal. Rotate a FULL circle, LEVEL, clear of metal.");
  }
}
// RAW magnetic heading. Declination is applied in the browser (TRUE toggle).
float getAzimuth() {
  if (!magOK) return -1.0f;
  int16_t x,y,z;
  if (!readMagRaw(x,y,z)) return -1.0f;
  float cx = (x - cfg.calOX) * cfg.calSX;
  float cy = (y - cfg.calOY) * cfg.calSY;

  // live interference check: a clean horizontal field has constant magnitude
  // (= calRadius) no matter which way you point. A big deviation means nearby
  // metal/magnet (or heavy tilt) is bending the field -> flag it.
  float r = sqrtf(cx*cx + cy*cy);
  if (cfg.calValid && cfg.calRadius > 1.0f) {
    magDevPct = fabsf(r - cfg.calRadius) / cfg.calRadius;
    magWarn   = (magDevPct > FIELD_THRESH);
  } else { magDevPct = 0.0f; magWarn = false; }

  // NOTE: -cy flips the rotation sense. If heading ever spins backwards
  // again after a remount, this is the sign to toggle.
  float h = atan2f(-cy, cx) * 180.0f / M_PI;
  if (h < 0) h += 360.0f;
  return smoothHdg(h);
}

// ══════════════════════════════════════════════════════════
//  WEB PAGE  (served from flash)
// ══════════════════════════════════════════════════════════
const char PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>GNSS HUD</title>
<style>
  :root{
    --bg:#05070a;
    --phos:#3dfca0;
    --phos-dim:#1c6b48;
    --amber:#ffb02e;
    --red:#ff4d4d;
    --grid:#0e1a16;
    --dim:#46584f;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  html,body{height:100%}
  body{
    background:
      radial-gradient(circle at 50% 38%, #0a1410 0%, var(--bg) 60%),
      var(--bg);
    color:var(--phos);
    font-family:ui-monospace,"SFMono-Regular","JetBrains Mono","Menlo","Consolas",monospace;
    min-height:100%;
    display:flex;flex-direction:column;align-items:center;
    overflow-x:hidden;
    -webkit-tap-highlight-color:transparent;
  }
  body::before{
    content:"";position:fixed;inset:0;pointer-events:none;z-index:0;
    background-image:
      linear-gradient(var(--grid) 1px,transparent 1px),
      linear-gradient(90deg,var(--grid) 1px,transparent 1px);
    background-size:34px 34px;opacity:.45;
  }
  body::after{
    content:"";position:fixed;inset:0;pointer-events:none;z-index:2;
    background:repeating-linear-gradient(0deg,rgba(0,0,0,0) 0,rgba(0,0,0,0) 2px,rgba(0,0,0,.18) 3px);
    mix-blend-mode:multiply;
  }
  .wrap{position:relative;z-index:1;width:100%;max-width:460px;padding:14px 16px 28px;}

  .strip{
    display:flex;justify-content:space-between;align-items:stretch;
    border:1px solid var(--phos-dim);border-radius:6px;
    background:linear-gradient(180deg,rgba(61,252,160,.05),transparent);
    padding:9px 4px;margin-bottom:18px;
  }
  .stat{flex:1;text-align:center;border-right:1px solid var(--grid);padding:0 4px}
  .stat:last-child{border-right:none}
  .stat .k{font-size:11px;letter-spacing:2px;color:var(--dim);text-transform:uppercase}
  .stat .v{font-size:22px;font-weight:700;margin-top:3px;line-height:1;letter-spacing:.5px}
  .v.good{color:var(--phos);text-shadow:0 0 8px rgba(61,252,160,.5)}
  .v.warn{color:var(--amber);text-shadow:0 0 8px rgba(255,176,46,.45)}
  .v.bad{color:var(--red);text-shadow:0 0 8px rgba(255,77,77,.5)}
  .dot{display:inline-block;width:7px;height:7px;border-radius:50%;vertical-align:middle;margin-right:5px}
  .dot.good{background:var(--phos);box-shadow:0 0 7px var(--phos);animation:pulse 1.6s infinite}
  .dot.bad{background:var(--red);box-shadow:0 0 7px var(--red)}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
  @media (prefers-reduced-motion:reduce){.dot.good{animation:none}}

  .compass{position:relative;width:100%;aspect-ratio:1/1;margin:0 auto;max-width:380px}
  .compass svg{width:100%;height:100%;display:block}
  .needle{transition:transform .35s cubic-bezier(.2,.8,.2,1);transform-origin:200px 200px}
  @media (prefers-reduced-motion:reduce){.needle{transition:none}}

  .center-read{position:absolute;inset:0;pointer-events:none}
  .az-block{position:absolute;top:26%;left:0;right:0;text-align:center}
  .az{font-size:38px;font-weight:800;line-height:.9;letter-spacing:-1px;
      text-shadow:0 0 14px rgba(61,252,160,.55)}
  .az .deg{font-size:22px;vertical-align:super;margin-left:1px}
  .az-mode{font-size:13px;letter-spacing:4px;color:var(--amber);margin-top:5px}

  .mgrs-box{
    margin-top:20px;border:1px solid var(--phos-dim);border-radius:6px;
    background:linear-gradient(180deg,rgba(61,252,160,.05),transparent);
    padding:12px 14px;text-align:center;
  }
  .mgrs-box .k{font-size:11px;letter-spacing:3px;color:var(--dim);text-transform:uppercase}
  .mgrs{font-size:30px;font-weight:700;margin-top:6px;letter-spacing:1px;word-spacing:4px;
        text-shadow:0 0 10px rgba(61,252,160,.4);min-height:30px}
  .latlon{font-size:13px;color:var(--dim);margin-top:7px;letter-spacing:.5px}

  .controls{margin-top:18px;display:flex;flex-direction:column;gap:12px}
  .toggle{display:flex;border:1px solid var(--phos-dim);border-radius:6px;overflow:hidden}
  .toggle button{
    flex:1;background:transparent;color:var(--dim);border:none;
    font-family:inherit;font-size:15px;font-weight:700;letter-spacing:2px;
    padding:11px 0;cursor:pointer;transition:all .15s;
  }
  .toggle button.on{background:rgba(61,252,160,.14);color:var(--phos);text-shadow:0 0 8px rgba(61,252,160,.5)}

  .decl{
    border:1px solid var(--phos-dim);border-radius:6px;padding:11px 13px;
    background:linear-gradient(180deg,rgba(61,252,160,.04),transparent);
  }
  .decl .lab{font-size:11px;letter-spacing:2px;color:var(--dim);text-transform:uppercase;margin-bottom:9px}
  .decl-row{display:flex;align-items:center;gap:10px}
  .step{
    width:46px;height:42px;flex:none;border:1px solid var(--phos-dim);border-radius:5px;
    background:transparent;color:var(--phos);font-size:22px;font-family:inherit;cursor:pointer;
  }
  .step:active{background:rgba(61,252,160,.16)}
  .decl-val{
    flex:1;text-align:center;font-size:28px;font-weight:700;
    text-shadow:0 0 10px rgba(61,252,160,.4)
  }
  .decl-val small{font-size:14px;color:var(--amber);letter-spacing:1px;display:block;margin-top:2px}
  .hint{font-size:12px;color:var(--dim);margin-top:9px;line-height:1.4;letter-spacing:.3px}

  .footer{margin-top:14px;display:flex;align-items:center;justify-content:space-between;gap:10px}
  .footer .cal{font-size:12px;letter-spacing:1px;color:var(--dim)}
  .footer button{
    background:transparent;border:1px solid var(--phos-dim);border-radius:6px;
    color:var(--amber);font-family:inherit;font-size:13px;letter-spacing:1px;
    padding:9px 12px;cursor:pointer;
  }
  .footer button:active{background:rgba(255,176,46,.12)}

  .banner{
    margin-top:14px;text-align:center;font-size:13px;letter-spacing:2px;
    color:var(--amber);border:1px dashed var(--phos-dim);border-radius:6px;padding:8px;
  }
</style>
</head>
<body>
<div class="wrap">

  <div class="strip">
    <div class="stat">
      <div class="k">Fix</div>
      <div class="v" id="fixV"><span class="dot bad" id="fixDot"></span><span id="fixT">--</span></div>
    </div>
    <div class="stat">
      <div class="k">Sats</div>
      <div class="v good" id="satsV">--</div>
    </div>
    <div class="stat">
      <div class="k">HDOP</div>
      <div class="v" id="hdopV">--</div>
    </div>
  </div>

  <div class="compass">
    <svg viewBox="0 0 400 400" aria-label="compass">
      <defs>
        <radialGradient id="face" cx="50%" cy="50%" r="50%">
          <stop offset="0%" stop-color="#0a1512"/>
          <stop offset="100%" stop-color="#040a08"/>
        </radialGradient>
        <linearGradient id="ndl" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%" stop-color="#7dffc4"/>
          <stop offset="100%" stop-color="#1f9c66"/>
        </linearGradient>
      </defs>

      <circle cx="200" cy="200" r="178" fill="url(#face)" stroke="#123a2a" stroke-width="1"/>
      <circle cx="200" cy="200" r="150" fill="none" stroke="#1c6b48" stroke-width="1.5" opacity=".85"/>
      <circle cx="200" cy="200" r="150" fill="none" stroke="#3dfca0" stroke-width="1" opacity=".25"/>

      <g id="ticks"></g>

      <g class="needle" id="needle">
        <polygon points="200,46 210,128 200,112 190,128" fill="url(#ndl)"
                 style="filter:drop-shadow(0 0 6px rgba(61,252,160,.7))"/>
        <line x1="200" y1="176" x2="200" y2="130" stroke="#3dfca0" stroke-width="2" opacity=".5"/>
      </g>

      <!-- wizard 'you' marker -->
      <g style="filter:drop-shadow(0 0 5px rgba(61,252,160,.85))">
        <path d="M185 192 Q190 178 198 170 Q203 165 207 160 Q206 170 208 180 Q210 187 215 192 Z" fill="#3dfca0"/>
        <rect x="179" y="189" width="42" height="4.2" rx="2.1" fill="#3dfca0"/>
        <rect x="188" y="185.5" width="22" height="2.3" fill="#05070a"/>
        <path d="M197 178 l1 2.4 2.6 .2 -2 1.7 .6 2.5 -2.2 -1.4 -2.2 1.4 .6 -2.5 -2 -1.7 2.6 -.2 z" fill="#ffb02e"/>
        <ellipse cx="200" cy="199" rx="8" ry="6" fill="#3dfca0"/>
        <path d="M193.5 195 q2.5 -1.8 5 -.6" stroke="#05070a" stroke-width="1.6" fill="none" stroke-linecap="round"/>
        <path d="M201.5 194.4 q2.5 -1.2 5 .6" stroke="#05070a" stroke-width="1.6" fill="none" stroke-linecap="round"/>
        <circle cx="196.5" cy="198" r="1.4" fill="#05070a"/>
        <circle cx="203.5" cy="198" r="1.4" fill="#05070a"/>
        <path d="M191 201 C188 212 192 222 200 228 C208 222 212 212 209 201 C200 206 200 206 191 201 Z" fill="#3dfca0"/>
        <path d="M200 204 Q195 205 192 208 Q197 207 200 205 Q203 207 208 208 Q205 205 200 204 Z" fill="#3dfca0"/>
        <path d="M197 209 Q196 217 199 225" stroke="#05070a" stroke-width=".9" fill="none"/>
        <path d="M203 209 Q204 217 201 225" stroke="#05070a" stroke-width=".9" fill="none"/>
        <line x1="205" y1="209" x2="218" y2="214" stroke="#05070a" stroke-width="4" stroke-linecap="round"/>
        <line x1="218" y1="214" x2="221" y2="207" stroke="#05070a" stroke-width="4.5" stroke-linecap="round"/>
        <line x1="205" y1="209" x2="218" y2="214" stroke="#3dfca0" stroke-width="2" stroke-linecap="round"/>
        <line x1="218" y1="214" x2="221" y2="207" stroke="#3dfca0" stroke-width="2.6" stroke-linecap="round"/>
        <path d="M221 205 q3 -2.5 0 -5 q-3 -2.5 0 -5" fill="none" stroke="#3dfca0" stroke-width="1" opacity=".4"/>
      </g>
    </svg>

    <div class="center-read">
      <div class="az-block">
        <div class="az"><span id="azNum">000</span><span class="deg">&deg;</span></div>
        <div class="az-mode" id="azMode">MAGNETIC</div>
      </div>
    </div>
  </div>

  <div class="mgrs-box">
    <div class="k">Grid &middot; MGRS</div>
    <div class="mgrs" id="mgrs">----------</div>
    <div class="latlon" id="latlon">lat --.-----   lon --.-----</div>
  </div>

  <div class="controls">
    <div class="toggle">
      <button id="magBtn" class="on">MAGNETIC</button>
      <button id="trueBtn">TRUE</button>
    </div>

    <div class="decl">
      <div class="lab">Magnetic Declination</div>
      <div class="decl-row">
        <button class="step" id="declMinus">&minus;</button>
        <div class="decl-val">
          <span id="declNum">0.0</span>&deg;
          <small id="declDir">ZERO</small>
        </div>
        <button class="step" id="declPlus">+</button>
      </div>
      <div class="hint">+ = East declination &middot; &minus; = West. Look yours up on NOAA's calculator for your location. Saved to the device. TRUE = Magnetic + Declination.</div>
    </div>
  </div>

  <div class="footer">
    <span class="cal" id="calStat">CAL: --</span>
    <button id="calBtn">RECALIBRATE 15s</button>
  </div>

  <div class="banner" id="banner">CONNECTING...</div>

</div>

<script>
let declination=0.0, mode='MAG', declInit=false, calUntil=0;
const els={
  azNum:document.getElementById('azNum'),
  azMode:document.getElementById('azMode'),
  needle:document.getElementById('needle'),
  mgrs:document.getElementById('mgrs'),
  latlon:document.getElementById('latlon'),
  satsV:document.getElementById('satsV'),
  hdopV:document.getElementById('hdopV'),
  fixDot:document.getElementById('fixDot'),
  fixT:document.getElementById('fixT'),
  declNum:document.getElementById('declNum'),
  declDir:document.getElementById('declDir'),
  calStat:document.getElementById('calStat'),
  banner:document.getElementById('banner')
};

/* compass ticks + cardinals (drawn once) */
(function drawTicks(){
  const g=document.getElementById('ticks');
  const cx=200,cy=200;
  const cardinals={0:'N',90:'E',180:'S',270:'W'};
  let s='';
  for(let a=0;a<360;a+=5){
    const major=(a%30===0);
    const r1=150, r2=major?132:140;
    const rad=(a-90)*Math.PI/180;
    const x1=cx+r1*Math.cos(rad), y1=cy+r1*Math.sin(rad);
    const x2=cx+r2*Math.cos(rad), y2=cy+r2*Math.sin(rad);
    s+='<line x1="'+x1+'" y1="'+y1+'" x2="'+x2+'" y2="'+y2+'" stroke="'+(major?'#3dfca0':'#1c6b48')+'" stroke-width="'+(major?2:1)+'" opacity="'+(major?.9:.5)+'"/>';
  }
  for(const a in cardinals){
    const rad=(a-90)*Math.PI/180;
    const x=cx+114*Math.cos(rad), y=cy+114*Math.sin(rad);
    const col=(a==0)?'#ffb02e':'#3dfca0';
    s+='<text x="'+x+'" y="'+(y+6)+'" fill="'+col+'" font-size="20" font-weight="700" text-anchor="middle" font-family="ui-monospace,monospace">'+cardinals[a]+'</text>';
  }
  g.innerHTML=s;
})();

/* ---- MGRS (lat/lon -> MGRS, proj4js algorithm) ---- */
function latLonToMGRS(lat, lon, accuracy){
  accuracy=accuracy||5;
  if(lat>84||lat<-80) return 'OUT OF MGRS RANGE';
  return encodeMGRS(LLtoUTM(lat,lon), accuracy);
}
function LLtoUTM(Lat, Long){
  const a=6378137.0, eccSquared=0.00669438, k0=0.9996;
  const LatRad=Lat*Math.PI/180, LongRad=Long*Math.PI/180;
  let ZoneNumber=Math.floor((Long+180)/6)+1;
  if(Long===180) ZoneNumber=60;
  if(Lat>=56&&Lat<64&&Long>=3&&Long<12) ZoneNumber=32;
  if(Lat>=72&&Lat<84){
    if(Long>=0&&Long<9)ZoneNumber=31;
    else if(Long>=9&&Long<21)ZoneNumber=33;
    else if(Long>=21&&Long<33)ZoneNumber=35;
    else if(Long>=33&&Long<42)ZoneNumber=37;
  }
  const LongOrigin=(ZoneNumber-1)*6-180+3;
  const LongOriginRad=LongOrigin*Math.PI/180;
  const eccPrimeSquared=eccSquared/(1-eccSquared);
  const N=a/Math.sqrt(1-eccSquared*Math.sin(LatRad)**2);
  const T=Math.tan(LatRad)**2;
  const C=eccPrimeSquared*Math.cos(LatRad)**2;
  const A=Math.cos(LatRad)*(LongRad-LongOriginRad);
  const M=a*((1-eccSquared/4-3*eccSquared**2/64-5*eccSquared**3/256)*LatRad
    -(3*eccSquared/8+3*eccSquared**2/32+45*eccSquared**3/1024)*Math.sin(2*LatRad)
    +(15*eccSquared**2/256+45*eccSquared**3/1024)*Math.sin(4*LatRad)
    -(35*eccSquared**3/3072)*Math.sin(6*LatRad));
  let UTMEasting=k0*N*(A+(1-T+C)*A**3/6+(5-18*T+T*T+72*C-58*eccPrimeSquared)*A**5/120)+500000.0;
  let UTMNorthing=k0*(M+N*Math.tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A**4/24
    +(61-58*T+T*T+600*C-330*eccPrimeSquared)*A**6/720));
  if(Lat<0) UTMNorthing+=10000000.0;
  return {northing:Math.round(UTMNorthing),easting:Math.round(UTMEasting),
          zoneNumber:ZoneNumber,zoneLetter:latBand(Lat)};
}
function latBand(lat){
  const b="CDEFGHJKLMNPQRSTUVWX";
  if(lat>=72&&lat<=84) return 'X';
  if(lat<-80||lat>84) return 'Z';
  return b.charAt(Math.floor((lat+80)/8));
}
const A=65,I=73,O=79,V=86,Z=90,NUM_SETS=6;
const COL_ORIGIN="AJSAJS", ROW_ORIGIN="AFAFAF";
function encodeMGRS(utm,acc){
  const se=""+utm.easting, sn=""+utm.northing;
  const id=get100kID(utm.easting,utm.northing,utm.zoneNumber);
  const e=se.slice(se.length-5).substr(0,acc);
  const n=sn.slice(sn.length-5).substr(0,acc);
  return utm.zoneNumber+utm.zoneLetter+" "+id+" "+e+" "+n;
}
function get100kID(easting,northing,zone){
  const set=(zone%NUM_SETS)||NUM_SETS;
  const col=Math.floor(easting/100000);
  const row=Math.floor(northing/100000)%20;
  return letter100k(col,row,set);
}
function letter100k(column,row,parm){
  const idx=parm-1;
  const colOrigin=COL_ORIGIN.charCodeAt(idx);
  const rowOrigin=ROW_ORIGIN.charCodeAt(idx);
  let colInt=colOrigin+column-1, rowInt=rowOrigin+row, roll=false;
  if(colInt>Z){colInt=colInt-Z+A-1;roll=true;}
  if(colInt===I||(colOrigin<I&&colInt>I)||((colInt>I||colOrigin<I)&&roll))colInt++;
  if(colInt===O||(colOrigin<O&&colInt>O)||((colInt>O||colOrigin<O)&&roll)){colInt++;if(colInt===I)colInt++;}
  if(colInt>Z)colInt=colInt-Z+A-1;
  if(rowInt>V){rowInt=rowInt-V+A-1;roll=true;}else roll=false;
  if((rowInt===I||(rowOrigin<I&&rowInt>I))||((rowInt>I||rowOrigin<I)&&roll))rowInt++;
  if((rowInt===O||(rowOrigin<O&&rowInt>O))||((rowInt>O||rowOrigin<O)&&roll)){rowInt++;if(rowInt===I)rowInt++;}
  if(rowInt>V)rowInt=rowInt-V+A-1;
  return String.fromCharCode(colInt)+String.fromCharCode(rowInt);
}

/* ---- data + render ---- */
function getData(){ return fetch('/status').then(r=>r.json()); }

function render(d){
  if(d.fix){
    els.fixDot.className='dot good';
    els.fixT.textContent='LOCK';
    els.fixT.parentElement.className='v good';
  }else{
    els.fixDot.className='dot bad';
    els.fixT.textContent='SRCH';
    els.fixT.parentElement.className='v bad';
  }
  els.satsV.textContent=d.fix?d.sats:'--';
  const hd=parseFloat(d.hdop);
  els.hdopV.textContent=d.fix?hd.toFixed(1):'--';
  els.hdopV.className='v '+(!d.fix?'':hd<1.5?'good':hd<3?'warn':'bad');

  if(!declInit && typeof d.decl==='number'){ applyDecl(d.decl); declInit=true; }

  // d.hdg is RAW magnetic from the device; TRUE = magnetic + declination
  let az=d.hdg;
  if(mode==='TRUE') az=d.hdg+declination;
  az=((az%360)+360)%360;
  els.azNum.textContent=String(Math.round(az)).padStart(3,'0');
  els.needle.style.transform='rotate('+az+'deg)';
  if(d.magwarn){
    els.azMode.textContent='! MAG INTERFERENCE '+(d.fdev||0)+'%';
    els.azMode.style.color='var(--red)';
    els.azNum.style.color='var(--red)';
  }else{
    els.azMode.textContent=(mode==='TRUE')?'TRUE':'MAGNETIC';
    els.azMode.style.color='var(--amber)';
    els.azNum.style.color='';
  }

  if(d.fix){
    els.mgrs.textContent=latLonToMGRS(d.lat,d.lon,5);
    els.latlon.textContent='lat '+d.lat.toFixed(5)+'   lon '+d.lon.toFixed(5);
  }else{
    els.mgrs.textContent='----------';
    els.latlon.textContent='lat --.-----   lon --.-----';
  }

  if(Date.now()<calUntil){
    els.calStat.textContent='CALIBRATING - ROTATE LEVEL';
    els.calStat.style.color='var(--amber)';
  }else if(d.cal){
    els.calStat.textContent='CAL: VALID'+(d.cov?(' '+d.cov+'%'):'');
    els.calStat.style.color='var(--phos)';
  }else{
    els.calStat.textContent='CAL: NOT SET';
    els.calStat.style.color='var(--amber)';
  }
}

/* ---- controls ---- */
document.getElementById('magBtn').onclick=()=>setMode('MAG');
document.getElementById('trueBtn').onclick=()=>setMode('TRUE');
function setMode(m){
  mode=m;
  document.getElementById('magBtn').classList.toggle('on',m==='MAG');
  document.getElementById('trueBtn').classList.toggle('on',m==='TRUE');
}
function applyDecl(v){
  declination=Math.max(-45,Math.min(45,Math.round(v*2)/2));
  els.declNum.textContent=Math.abs(declination).toFixed(1);
  els.declDir.textContent=declination>0?'EAST (+)':declination<0?'WEST (-)':'ZERO';
  els.declDir.style.color=declination!==0?'var(--amber)':'var(--dim)';
}
function setDecl(v){ applyDecl(v); fetch('/setdecl?d='+declination).catch(()=>{}); }
document.getElementById('declPlus').onclick=()=>setDecl(declination+0.5);
document.getElementById('declMinus').onclick=()=>setDecl(declination-0.5);
document.getElementById('calBtn').onclick=()=>{ fetch('/recal').catch(()=>{}); calUntil=Date.now()+16000; };

function link(ok){
  if(ok){
    els.banner.textContent='LINK ACTIVE';
    els.banner.style.color='var(--phos)';
    els.banner.style.borderColor='var(--phos-dim)';
  }else{
    els.banner.textContent='NO LINK - REJOIN GNSS-WIZARD WI-FI';
    els.banner.style.color='var(--red)';
    els.banner.style.borderColor='var(--red)';
  }
}

applyDecl(0);
function tick(){ getData().then(d=>{render(d);link(true);}).catch(()=>link(false)); }
tick();
setInterval(tick, 500);
</script>
</body>
</html>)HTML";

// ══════════════════════════════════════════════════════════
//  WEB HANDLERS
// ══════════════════════════════════════════════════════════
void handleRoot() { server.send_P(200, "text/html", PAGE_HTML); }

void handleStatus() {
  bool fix = gps.location.isValid();
  char j[320];
  snprintf(j, sizeof(j),
    "{\"fix\":%s,\"sats\":%d,\"hdop\":%.1f,\"lat\":%.6f,\"lon\":%.6f,"
    "\"hdg\":%.1f,\"cal\":%s,\"decl\":%.1f,\"magwarn\":%s,\"fdev\":%d,\"cov\":%d}",
    fix ? "true" : "false",
    gps.satellites.isValid() ? (int)gps.satellites.value() : 0,
    gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
    fix ? gps.location.lat() : 0.0,
    fix ? gps.location.lng() : 0.0,
    lastHdg,
    cfg.calValid ? "true" : "false",
    cfg.declination,
    magWarn ? "true" : "false",
    (int)(magDevPct * 100.0f),
    lastCalOK ? (int)(lastCalCov * 100.0f) : 0);
  server.send(200, "application/json", j);
}

void handleSetDecl() {
  if (!server.hasArg("d")) { server.send(400, "text/plain", "missing d"); return; }
  float d = server.arg("d").toFloat();
  if (d < -45) d = -45;
  if (d >  45) d =  45;
  cfg.declination = d;
  saveDecl();
  server.send(200, "text/plain", "ok");
}

void handleRecal() {
  if (calibrating) { server.send(200, "text/plain", "busy"); return; }
  server.send(200, "text/plain", "calibrating");
  calibrating = true;
  runCal();
  calibrating = false;
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n\n=== GNSS WIZARD HUD ===");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  loadConfig();
  Serial.printf("[CFG] declination %.1f  cal:%s\n",
                cfg.declination, cfg.calValid ? "valid" : "none");

  // magnetometer
  Wire.begin(21, 22);
  Wire.setTimeout(1000);
  magOK = initMag();

  // Wi-Fi access point (open, no router needed)
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(AP_SSID);
  Serial.printf("[WIFI] AP '%s'  ->  http://%s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());

  // routes
  server.on("/",        HTTP_GET, handleRoot);
  server.on("/status",  HTTP_GET, handleStatus);
  server.on("/setdecl", HTTP_GET, handleSetDecl);
  server.on("/recal",   HTTP_GET, handleRecal);
  server.begin();
  Serial.println("[WEB] server on :80");

  // first-time compass calibration
  if (magOK && !cfg.calValid) {
    Serial.println("[CAL] no saved cal -- calibrating now, rotate device");
    runCal();
  } else if (cfg.calValid) {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // GPS UART
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.printf("[GPS] UART RX=%d TX=%d @ %d baud\n", GPS_RX, GPS_TX, GPS_BAUD);
  Serial.println("=== ready ===\n");
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════
void loop() {
  while (Serial2.available()) gps.encode(Serial2.read());
  server.handleClient();

  // refresh heading ~6.7 Hz
  static unsigned long tHdg = 0;
  if (millis() - tHdg >= 150) {
    tHdg = millis();
    float h = getAzimuth();
    if (h >= 0) lastHdg = h;
  }

  // serial heartbeat every 2 s
  static unsigned long tLog = 0;
  if (millis() - tLog >= 2000) {
    tLog = millis();
    int clients = WiFi.softAPgetStationNum();
    if (gps.location.isValid())
      Serial.printf("[FIX] %.6f,%.6f  sats:%d  hdop:%.1f  hdg(mag):%.1f  clients:%d\n",
        gps.location.lat(), gps.location.lng(),
        gps.satellites.isValid() ? (int)gps.satellites.value() : 0,
        gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
        lastHdg, clients);
    else
      Serial.printf("[----] no fix  hdg(mag):%.1f  clients:%d\n", lastHdg, clients);
  }
}
