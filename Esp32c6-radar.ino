/*
  ESP32-C6 Mini Radar (Home Experiment)
  - Servo scan (LEDC hardware PWM) on GPIO18
  - HLK-LD2420 OUT (presence) on GPIO0
  - Web UI with live calibration (center / spans / bias / speed), saved to NVS

  Arduino-ESP32 core 3.x uses:
    ledcAttach(pin, freq, resolutionBits)
    ledcWrite(pin, duty)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

// -------------------- Pins --------------------
static const uint8_t SERVO_PIN = 18;
static const uint8_t RADAR_OUT_PIN = 0;   // (strap pin) works, but can affect boot on some setups

// -------------------- Wi-Fi AP --------------------
static const char* AP_SSID = "ESP32-RADAR";
static const char* AP_PASS = "12345678";

// -------------------- Servo PWM (LEDC) --------------------
// For ESP32-C6, LEDC resolution is typically up to 14 bits.
// We'll use 14 for compatibility.
static const uint8_t SERVO_PWM_BITS = 14;
static const uint32_t SERVO_PWM_FREQ = 50;          // 50 Hz (20 ms)
static const uint32_t SERVO_PERIOD_US = 20000;      // 20 ms
static const uint32_t SERVO_MAX_DUTY = (1UL << SERVO_PWM_BITS) - 1;

// Typical safe pulse limits for SG90 (can be narrower for safety)
static const int SERVO_US_MIN_CLAMP = 900;
static const int SERVO_US_MAX_CLAMP = 2100;

// -------------------- Config (stored) --------------------
struct Config {
  int centerUs = 1499;      // good starting point between твоими 1500 и 1498
  int leftSpanUs  = 60;     // left limit = center - leftSpanUs
  int rightSpanUs = 70;     // start a bit more to the right to fight "drift left"
  int biasRightUs = 6;      // direction compensation
  int biasLeftUs  = 0;

  // Scan speed: full sine cycle (left->right->left) period in ms
  int scanPeriodMs = 3500;  // adjust in UI

  // Smoothing: how often we refresh servo command (ms)
  int tickMs = 20;          // 20ms = 50Hz update (good)
};

Config cfg;

Preferences prefs;
WebServer server(80);

// runtime
unsigned long lastTick = 0;
unsigned long startMs = 0;

float lastSin = 0.0f;
bool lastDetected = false;
unsigned long lastDetRiseMs = 0;

// -------------------- Helpers --------------------
static inline int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline uint32_t usToDuty(int us) {
  us = clampInt(us, SERVO_US_MIN_CLAMP, SERVO_US_MAX_CLAMP);
  // duty = us / 20000 * MAX_DUTY
  // use 64-bit to avoid rounding errors
  uint64_t duty = (uint64_t)us * (uint64_t)SERVO_MAX_DUTY;
  duty /= (uint64_t)SERVO_PERIOD_US;
  if (duty > SERVO_MAX_DUTY) duty = SERVO_MAX_DUTY;
  return (uint32_t)duty;
}

static bool servoAttached = false;

static void servoBegin() {
  // Attach LEDC to SERVO_PIN
  // returns bool in Arduino-ESP32 3.x
  bool ok = ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, SERVO_PWM_BITS);
  servoAttached = ok;

  // Set center on boot
  if (servoAttached) {
    ledcWrite(SERVO_PIN, usToDuty(cfg.centerUs));
  } else {
    // Fallback (should rarely happen): keep pin low
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
  }
}

static void servoWriteUs(int us) {
  us = clampInt(us, SERVO_US_MIN_CLAMP, SERVO_US_MAX_CLAMP);
  if (servoAttached) {
    ledcWrite(SERVO_PIN, usToDuty(us));
  }
}

// Convert current scan position to "radar angle" for UI (0..180)
// We use sin-based scan. sin=-1 => 0°, sin=+1 => 180°, center => 90°
static inline int sinToAngleDeg(float s) {
  float a = (s + 1.0f) * 90.0f;
  int ai = (int)lroundf(a);
  return clampInt(ai, 0, 180);
}

// -------------------- Persistence --------------------
static void loadConfig() {
  prefs.begin("radar", true);
  cfg.centerUs     = prefs.getInt("center", cfg.centerUs);
  cfg.leftSpanUs   = prefs.getInt("lspan",  cfg.leftSpanUs);
  cfg.rightSpanUs  = prefs.getInt("rspan",  cfg.rightSpanUs);
  cfg.biasRightUs  = prefs.getInt("br",     cfg.biasRightUs);
  cfg.biasLeftUs   = prefs.getInt("bl",     cfg.biasLeftUs);
  cfg.scanPeriodMs = prefs.getInt("period", cfg.scanPeriodMs);
  cfg.tickMs       = prefs.getInt("tick",   cfg.tickMs);
  prefs.end();

  // sanitize
  cfg.leftSpanUs   = clampInt(cfg.leftSpanUs,  0, 250);
  cfg.rightSpanUs  = clampInt(cfg.rightSpanUs, 0, 250);
  cfg.centerUs     = clampInt(cfg.centerUs,  SERVO_US_MIN_CLAMP, SERVO_US_MAX_CLAMP);
  cfg.biasRightUs  = clampInt(cfg.biasRightUs, -30, 30);
  cfg.biasLeftUs   = clampInt(cfg.biasLeftUs,  -30, 30);
  cfg.scanPeriodMs = clampInt(cfg.scanPeriodMs, 1200, 12000);
  cfg.tickMs       = clampInt(cfg.tickMs, 10, 50);
}

static void saveConfig() {
  prefs.begin("radar", false);
  prefs.putInt("center", cfg.centerUs);
  prefs.putInt("lspan",  cfg.leftSpanUs);
  prefs.putInt("rspan",  cfg.rightSpanUs);
  prefs.putInt("br",     cfg.biasRightUs);
  prefs.putInt("bl",     cfg.biasLeftUs);
  prefs.putInt("period", cfg.scanPeriodMs);
  prefs.putInt("tick",   cfg.tickMs);
  prefs.end();
}

// -------------------- Radar OUT --------------------
static bool readRadarDetected() {
  return digitalRead(RADAR_OUT_PIN) == HIGH;
}

// -------------------- Scan / Control --------------------
static int calcServoUsFromScan(unsigned long nowMs, int &angleDegOut, bool &movingRightOut) {
  // Sine scan (smooth turns, less stress and drift)
  // phase: 0..2pi over scanPeriodMs
  float t = (float)((nowMs - startMs) % (unsigned long)cfg.scanPeriodMs) / (float)cfg.scanPeriodMs;
  float phase = t * 2.0f * (float)M_PI;

  float s = sinf(phase);
  float c = cosf(phase);

  movingRightOut = (c > 0.0f);  // derivative of sin is cos

  int span = (s < 0.0f) ? cfg.leftSpanUs : cfg.rightSpanUs;
  int base = cfg.centerUs + (int)lroundf(s * (float)span);

  // Direction compensation (backlash / hysteresis)
  base += movingRightOut ? cfg.biasRightUs : cfg.biasLeftUs;

  // absolute clamp
  int minUs = cfg.centerUs - cfg.leftSpanUs;
  int maxUs = cfg.centerUs + cfg.rightSpanUs;
  base = clampInt(base, minUs, maxUs);
  base = clampInt(base, SERVO_US_MIN_CLAMP, SERVO_US_MAX_CLAMP);

  angleDegOut = sinToAngleDeg(s);
  lastSin = s;
  return base;
}

// -------------------- Web UI --------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>ESP32 Radar</title>
<style>
  body{background:#000;color:#0f0;font-family:system-ui,Arial;margin:0;padding:12px}
  .row{display:flex;gap:12px;flex-wrap:wrap}
  canvas{background:#000;border:1px solid #063; border-radius:10px}
  .card{border:1px solid #063;border-radius:10px;padding:10px;min-width:300px;flex:1}
  label{display:block;margin-top:10px}
  input[type=range]{width:100%}
  button{background:#031;color:#0f0;border:1px solid #063;border-radius:8px;padding:8px 10px;margin:6px 6px 0 0}
  .muted{color:#6f6;font-size:12px}
</style>
</head>
<body>
<h2 style="margin:0 0 10px 0">🟢 Mini Radar (ESP32)</h2>

<div class="row">
  <canvas id="cv" width="360" height="240"></canvas>

  <div class="card">
    <div><b>Статус:</b> <span id="st">...</span></div>
    <div class="muted" id="dbg"></div>

    <label>Center (µs): <span id="v_center"></span>
      <input id="center" type="range" min="1400" max="1600" step="1"/>
    </label>

    <label>Left span (µs): <span id="v_lspan"></span>
      <input id="lspan" type="range" min="0" max="200" step="1"/>
    </label>

    <label>Right span (µs): <span id="v_rspan"></span>
      <input id="rspan" type="range" min="0" max="200" step="1"/>
    </label>

    <label>Bias right (µs): <span id="v_br"></span>
      <input id="br" type="range" min="-20" max="20" step="1"/>
    </label>

    <label>Bias left (µs): <span id="v_bl"></span>
      <input id="bl" type="range" min="-20" max="20" step="1"/>
    </label>

    <label>Scan period (ms): <span id="v_period"></span>
      <input id="period" type="range" min="1200" max="9000" step="50"/>
    </label>

    <label>Tick (ms): <span id="v_tick"></span>
      <input id="tick" type="range" min="10" max="50" step="1"/>
    </label>

    <div style="margin-top:10px">
      <button onclick="nudge(-1)">◀ -1</button>
      <button onclick="nudge(+1)">+1 ▶</button>
      <button onclick="save()">💾 Save</button>
      <button onclick="recenter()">🎯 Center</button>
      <button onclick="toggleVoice()">🔊 Voice: <span id="voice">ON</span></button>
    </div>

    <div class="muted" style="margin-top:10px">
      Подсказка: если диапазон “уползает” влево — обычно помогает:
      увеличить <b>Bias right</b> или <b>Right span</b> на пару единиц.
    </div>
  </div>
</div>

<script>
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');

let prevDet = false;
let voiceOn = true;
let points = []; // {a, t}
const maxAge = 3500; // ms

function speak(text){
  if(!voiceOn) return;
  if(!('speechSynthesis' in window)) return;
  const u = new SpeechSynthesisUtterance(text);
  u.lang = 'ru-RU';
  u.rate = 1.05;
  window.speechSynthesis.cancel();
  window.speechSynthesis.speak(u);
}

function toggleVoice(){
  voiceOn = !voiceOn;
  document.getElementById('voice').textContent = voiceOn ? 'ON' : 'OFF';
}

function drawGrid(){
  const w = cv.width, h = cv.height;
  ctx.clearRect(0,0,w,h);

  const cx = w/2, cy = h*0.92;
  const r = Math.min(w,h)*0.85;

  ctx.strokeStyle = '#0a3';
  ctx.lineWidth = 1;

  // arcs
  for(let k=1;k<=4;k++){
    ctx.beginPath();
    ctx.arc(cx, cy, r*(k/4), Math.PI, 2*Math.PI);
    ctx.stroke();
  }
  // radial lines
  for(let deg=0;deg<=180;deg+=30){
    const a = Math.PI + (deg*Math.PI/180);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + r*Math.cos(a), cy + r*Math.sin(a));
    ctx.stroke();
  }
}

function drawSweep(angleDeg){
  const w = cv.width, h = cv.height;
  const cx = w/2, cy = h*0.92;
  const r = Math.min(w,h)*0.85;

  const a = Math.PI + (angleDeg*Math.PI/180);
  ctx.strokeStyle = '#0f0';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + r*Math.cos(a), cy + r*Math.sin(a));
  ctx.stroke();
}

function drawPoints(now){
  const w = cv.width, h = cv.height;
  const cx = w/2, cy = h*0.92;
  const r = Math.min(w,h)*0.85;

  points = points.filter(p => (now - p.t) < maxAge);

  for(const p of points){
    const age = now - p.t;
    const alpha = Math.max(0, 1 - age/maxAge);
    const a = Math.PI + (p.a*Math.PI/180);
    const rr = r*0.72;

    ctx.fillStyle = `rgba(255,0,0,${alpha})`;
    ctx.beginPath();
    ctx.arc(cx + rr*Math.cos(a), cy + rr*Math.sin(a), 5 + 6*alpha, 0, 2*Math.PI);
    ctx.fill();
  }
}

function setUI(cfg){
  const setVal = (id, v) => { document.getElementById(id).value = v; document.getElementById('v_'+id).textContent = v; };
  setVal('center', cfg.center);
  setVal('lspan', cfg.lspan);
  setVal('rspan', cfg.rspan);
  setVal('br', cfg.br);
  setVal('bl', cfg.bl);
  setVal('period', cfg.period);
  setVal('tick', cfg.tick);
}

let debounceT=null;
function pushCfg(saveFlag=false){
  const cfg = {
    center: +document.getElementById('center').value,
    lspan: +document.getElementById('lspan').value,
    rspan: +document.getElementById('rspan').value,
    br: +document.getElementById('br').value,
    bl: +document.getElementById('bl').value,
    period: +document.getElementById('period').value,
    tick: +document.getElementById('tick').value
  };
  for(const k in cfg){
    document.getElementById('v_'+k).textContent = cfg[k];
  }
  const qs = new URLSearchParams(cfg);
  if(saveFlag) qs.set('save','1');
  fetch('/api/set?'+qs.toString()).catch(()=>{});
}

['center','lspan','rspan','br','bl','period','tick'].forEach(id=>{
  document.getElementById(id).addEventListener('input', ()=>{
    clearTimeout(debounceT);
    debounceT=setTimeout(()=>pushCfg(false), 150);
  });
});

function save(){ pushCfg(true); speak('Сохранено'); }
function nudge(d){ fetch('/api/nudge?d='+d).then(()=>{}); }
function recenter(){ fetch('/api/recenter').then(()=>{}); speak('Центр'); }

async function tick(){
  try{
    const r = await fetch('/api/state',{cache:'no-store'});
    const j = await r.json();

    document.getElementById('st').textContent = j.det ? 'Движение' : 'Пусто';
    document.getElementById('dbg').textContent =
      `angle=${j.angle}°, us=${j.us}, dir=${j.dir}, OUT=${j.det}`;

    // rising edge => add point + voice
    if(j.det && !prevDet){
      points.push({a:j.angle, t:Date.now()});
      speak(`Движение. Сектор ${j.angle} градусов`);
    }
    prevDet = !!j.det;

    drawGrid();
    drawPoints(Date.now());
    drawSweep(j.angle);

    if(!window.__uiInited){
      setUI(j.cfg);
      window.__uiInited=true;
    }
  }catch(e){
    document.getElementById('st').textContent = 'нет связи';
  }
}
setInterval(tick, 120);
tick();
</script>
</body>
</html>
)HTML";

static void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleState() {
  const bool det = readRadarDetected();

  int angleDeg = 90;
  bool movingRight = true;
  int us = calcServoUsFromScan(millis(), angleDeg, movingRight);

  // JSON without ArduinoJson
  String s = "{";
  s += "\"det\":"; s += (det ? "1" : "0"); s += ",";
  s += "\"angle\":"; s += String(angleDeg); s += ",";
  s += "\"us\":"; s += String(us); s += ",";
  s += "\"dir\":\""; s += (movingRight ? "R" : "L"); s += "\",";
  s += "\"cfg\":{";
  s += "\"center\":" + String(cfg.centerUs) + ",";
  s += "\"lspan\":" + String(cfg.leftSpanUs) + ",";
  s += "\"rspan\":" + String(cfg.rightSpanUs) + ",";
  s += "\"br\":" + String(cfg.biasRightUs) + ",";
  s += "\"bl\":" + String(cfg.biasLeftUs) + ",";
  s += "\"period\":" + String(cfg.scanPeriodMs) + ",";
  s += "\"tick\":" + String(cfg.tickMs);
  s += "}}";
  server.send(200, "application/json", s);
}

static void handleSet() {
  auto getInt = [&](const char* key, int &dst){
    if (server.hasArg(key)) dst = server.arg(key).toInt();
  };

  getInt("center", cfg.centerUs);
  getInt("lspan",  cfg.leftSpanUs);
  getInt("rspan",  cfg.rightSpanUs);
  getInt("br",     cfg.biasRightUs);
  getInt("bl",     cfg.biasLeftUs);
  getInt("period", cfg.scanPeriodMs);
  getInt("tick",   cfg.tickMs);

  // sanitize
  cfg.leftSpanUs   = clampInt(cfg.leftSpanUs,  0, 250);
  cfg.rightSpanUs  = clampInt(cfg.rightSpanUs, 0, 250);
  cfg.centerUs     = clampInt(cfg.centerUs, SERVO_US_MIN_CLAMP, SERVO_US_MAX_CLAMP);
  cfg.biasRightUs  = clampInt(cfg.biasRightUs, -30, 30);
  cfg.biasLeftUs   = clampInt(cfg.biasLeftUs,  -30, 30);
  cfg.scanPeriodMs = clampInt(cfg.scanPeriodMs, 1200, 12000);
  cfg.tickMs       = clampInt(cfg.tickMs, 10, 50);

  // optional save
  if (server.hasArg("save") && server.arg("save") == "1") {
    saveConfig();
  }

  server.send(200, "text/plain", "OK");
}

static void handleNudge() {
  int d = 0;
  if (server.hasArg("d")) d = server.arg("d").toInt();
  d = clampInt(d, -5, 5);
  cfg.centerUs = clampInt(cfg.centerUs + d, SERVO_US_MIN_CLAMP, SERVO_US_MAX_CLAMP);
  server.send(200, "text/plain", "OK");
}

static void handleRecenter() {
  // Put servo to center for ~0.6s (without stopping the scan forever)
  unsigned long tEnd = millis() + 600;
  while (millis() < tEnd) {
    servoWriteUs(cfg.centerUs);
    delay(20);
    server.handleClient(); // keep web responsive
  }
  server.send(200, "text/plain", "OK");
}

// -------------------- Setup / Loop --------------------
void setup() {
  // Radar OUT
  // If OUT is open drain, pullup helps. Also helps keep GPIO0 high in runtime.
  pinMode(RADAR_OUT_PIN, INPUT_PULLUP);

  loadConfig();
  servoBegin();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.on("/api/state", handleState);
  server.on("/api/set", handleSet);
  server.on("/api/nudge", handleNudge);
  server.on("/api/recenter", handleRecenter);
  server.begin();

  startMs = millis();
  lastTick = millis();
}

void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastTick >= (unsigned long)cfg.tickMs) {
    lastTick += (unsigned long)cfg.tickMs;

    int angleDeg = 90;
    bool movingRight = true;
    int us = calcServoUsFromScan(now, angleDeg, movingRight);

    servoWriteUs(us);
  }
}
