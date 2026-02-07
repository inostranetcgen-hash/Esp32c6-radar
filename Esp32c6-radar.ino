/*
  ESP32‑C6 Radar + Servo scanner + Web UI (2 tabs) + Voice (browser)
  Sensor: HLK‑LD2420 (UART basic status: "ON"/"OFF" + "Range XXXX")
  Servo: SG90 on SERVO_PIN (default GPIO18)

  Works with ESP32 Arduino core 3.x (ledcAttach/ledcWrite new API).
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// -------------------- ПИНЫ (под твою сборку) --------------------

// Серво (ты сказал, что подключено в отверстие "18")
#define SERVO_PIN 18

// LD2420 OUT (если подключал провод в "0" — обычно это GPIO0)
// Можно не использовать OUT, но оставим как дополнительный индикатор.
#define RADAR_OUT_PIN 0      // поставь -1 если OUT не подключён

// UART радара (LD2420). ВАЖНО: выставь под свои провода!
#define RADAR_BAUD 115200

// Вариант А (рекомендуется): радар на Serial1 (отдельные пины, не мешает USB)
#define RADAR_USE_SERIAL0 0  // 0=Serial1, 1=Serial (если ты реально воткнул радар в TX/RX которые делят USB)

// Эти пины поменяй под себя, если радар на Serial1.
// Если не уверен — скажи какие номера пинов на плате у RX/TX куда воткнул радар.
#define RADAR_RX_PIN 20
#define RADAR_TX_PIN 21

// Управление питанием радара (ТОЛЬКО если у тебя стоит MOSFET/ключ/EN пин у DC-DC)
// Если нет — оставь -1, будет “выключение” только программно.
#define RADAR_PWR_PIN -1
#define RADAR_PWR_ACTIVE_HIGH 1

// Измерение батареи (если сделаешь делитель на ADC). Если нет — оставь -1.
#define BATTERY_ADC_PIN -1
#define BATTERY_DIVIDER_RATIO 2.0f   // 100k/100k => 2.0 (4.2В -> 2.1В на ADC)

// -------------------- Wi‑Fi AP --------------------
static const char *AP_SSID = "RLS-RADAR";
static const char *AP_PASS = "12345678"; // минимум 8 символов

// -------------------- Серво PWM (LEDC) --------------------
static const uint16_t SERVO_FREQ = 50;       // 50Hz
static const uint8_t  SERVO_RES  = 16;       // 16-bit
static const uint32_t SERVO_MAX_DUTY = (1UL << SERVO_RES) - 1;
static const uint32_t SERVO_PERIOD_US = 1000000UL / SERVO_FREQ; // 20000us

// -------------------- Web --------------------
WebServer server(80);
Preferences prefs;

// -------------------- Настройки, которые сохраняем --------------------
struct Settings {
  int center_us;
  int left_span_us;
  int right_span_us;
  int bias_right_us;
  int bias_left_us;
  int scan_period_ms;   // полный цикл (лево->право->лево)
  int tick_ms;          // как часто обновляем команду серво
  bool servo_hold;      // удерживать PWM когда “стоп”
};

Settings cfg;

// -------------------- Состояния --------------------
bool servo_enabled = false;      // важно: после питания НЕ стартуем
bool servo_attached = false;

bool radar_enabled = true;
bool radar_powered = true;

uint32_t scan_start_ms = 0;
uint32_t last_servo_update_ms = 0;

int servo_us_cmd = 1500;
int servo_dir = +1; // +1 = вправо, -1 = влево

// Radar данные (из UART basic mode)
bool radar_presence = false;
int  radar_range_cm = 0;

uint32_t last_radar_line_ms = 0;
uint32_t last_range_ms = 0;

float speed_kmh = 0.0f;     // + = удаляется (дистанция растёт), - = приближается
int   move_dir = 0;         // +1 away, -1 to us, 0 unknown

// Для расчёта скорости
bool have_prev_range = false;
int  prev_range_cm = 0;
uint32_t prev_range_ms = 0;

// Battery
float battery_v = NAN;
int battery_pct = -1;
uint32_t last_batt_ms = 0;

// -------------------- UART радара --------------------
#if RADAR_USE_SERIAL0
  #define DBG(...) do{}while(0)
  #define DBGLN(...) do{}while(0)
  // В этом режиме Serial занят радаром => логов не будет
#else
  #define DBG(...)   Serial.print(__VA_ARGS__)
  #define DBGLN(...) Serial.println(__VA_ARGS__)
  HardwareSerial RadarSerial(1);
#endif

// -------------------- Утилиты --------------------
static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static uint32_t usToDuty(int us) {
  us = clampi(us, 500, 2500);
  // duty = us / period * maxDuty
  return (uint32_t)((uint64_t)us * SERVO_MAX_DUTY / SERVO_PERIOD_US);
}

static void servoAttachIfNeeded() {
  if (servo_attached) return;
  // новый API (ESP32 core 3.x)
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  servo_attached = true;
}

static void servoDetachIfNeeded() {
  if (!servo_attached) return;
  ledcDetach(SERVO_PIN);
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  servo_attached = false;
}

static void servoWriteUs(int us) {
  servoAttachIfNeeded();
  servo_us_cmd = clampi(us, 500, 2500);
  ledcWrite(SERVO_PIN, usToDuty(servo_us_cmd));
}

// Удобный “угол” 0..180, где 90 — центр
static int servoAngleDegFromUs(int us) {
  int c = cfg.center_us;
  int L = max(1, cfg.left_span_us);
  int R = max(1, cfg.right_span_us);

  if (us <= c) {
    float t = float(c - us) / float(L);
    t = constrain(t, 0.0f, 1.0f);
    return (int)roundf(90.0f - 90.0f * t);
  } else {
    float t = float(us - c) / float(R);
    t = constrain(t, 0.0f, 1.0f);
    return (int)roundf(90.0f + 90.0f * t);
  }
}

// -------------------- LD2420 команды (по протоколу) --------------------
// Берём формат кадров из документации/реализаций (FD FC FB FA ... 04 03 02 01)
static void ld2420SendFrame(uint16_t cmd, const uint8_t *payload, uint16_t payloadLen) {
#if RADAR_USE_SERIAL0
  // Serial == радар
  auto &S = Serial;
#else
  auto &S = RadarSerial;
#endif
  uint8_t buf[64];
  uint16_t len = 2 + payloadLen;
  if (len + 8 > sizeof(buf)) return;

  buf[0] = 0xFD; buf[1] = 0xFC; buf[2] = 0xFB; buf[3] = 0xFA;
  buf[4] = (uint8_t)(len & 0xFF);
  buf[5] = (uint8_t)(len >> 8);
  buf[6] = (uint8_t)(cmd & 0xFF);
  buf[7] = (uint8_t)(cmd >> 8);

  for (uint16_t i = 0; i < payloadLen; i++) buf[8 + i] = payload[i];

  uint16_t p = 8 + payloadLen;
  buf[p + 0] = 0x04; buf[p + 1] = 0x03; buf[p + 2] = 0x02; buf[p + 3] = 0x01;

  S.write(buf, p + 4);
  S.flush();
}

static void ld2420SetBasicStatusMode() {
  // Enable config mode: FF 00 + protocol ver 02 00
  const uint8_t proto[] = {0x02, 0x00};
  ld2420SendFrame(0x00FF, proto, sizeof(proto));
  delay(120);

  // Set sys param: cmd 12 00, payload: paramId 00 00, value 64 00 00 00 (basic status)
  const uint8_t sys_basic[] = {0x00, 0x00, 0x64, 0x00, 0x00, 0x00};
  ld2420SendFrame(0x0012, sys_basic, sizeof(sys_basic));
  delay(120);

  // Disable config mode: FE 00
  ld2420SendFrame(0x00FE, nullptr, 0);
  delay(120);
}

// -------------------- Radar parsing (text mode) --------------------
static String radarLine;

static void radarProcessLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  last_radar_line_ms = millis();

  // Возможные варианты строк: "ON", "OFF", "Range 123", "Range: 123"
  if (s == "ON") {
    radar_presence = true;
    return;
  }
  if (s == "OFF") {
    radar_presence = false;
    return;
  }

  int idx = s.indexOf("Range");
  if (idx >= 0) {
    // вытаскиваем число
    int num = 0;
    bool found = false;
    for (int i = idx; i < (int)s.length(); i++) {
      if (isDigit((unsigned char)s[i])) {
        found = true;
        num = atoi(s.c_str() + i);
        break;
      }
    }
    if (found && num > 0 && num < 20000) {
      radar_range_cm = num;
      last_range_ms = millis();

      // скорость
      if (!have_prev_range) {
        have_prev_range = true;
        prev_range_cm = radar_range_cm;
        prev_range_ms = last_range_ms;
        speed_kmh = 0;
        move_dir = 0;
      } else {
        uint32_t dt = last_range_ms - prev_range_ms;
        if (dt >= 150) {
          float dist_m = radar_range_cm / 100.0f;
          float prev_m = prev_range_cm / 100.0f;
          float dt_s = dt / 1000.0f;
          float inst_ms = (dist_m - prev_m) / dt_s;   // + => растёт дистанция
          float inst_kmh = inst_ms * 3.6f;

          // сглаживание
          speed_kmh = 0.80f * speed_kmh + 0.20f * inst_kmh;

          const float TH = 0.8f; // порог “движется”
          if (speed_kmh > TH) move_dir = +1;       // away
          else if (speed_kmh < -TH) move_dir = -1; // to us
          else move_dir = 0;

          prev_range_cm = radar_range_cm;
          prev_range_ms = last_range_ms;
        }
      }
    }
  }
}

static void radarPollSerial() {
  if (!radar_enabled) return;

#if RADAR_USE_SERIAL0
  auto &S = Serial;
#else
  auto &S = RadarSerial;
#endif

  while (S.available()) {
    char c = (char)S.read();
    if (c == '\r') continue;
    if (c == '\n') {
      radarProcessLine(radarLine);
      radarLine = "";
    } else {
      if (radarLine.length() < 80) radarLine += c;
    }
  }
}

// -------------------- Battery --------------------
static int socFromVoltage(float v) {
  // грубая таблица Li-Ion (без нагрузки сильно точнее не будет)
  struct P { float v; int p; };
  const P t[] = {
    {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.92f, 70}, {3.85f, 60},
    {3.79f, 50},  {3.73f, 40}, {3.68f, 30}, {3.62f, 20}, {3.55f, 10},
    {3.30f, 0}
  };
  if (v >= t[0].v) return 100;
  if (v <= t[sizeof(t)/sizeof(t[0]) - 1].v) return 0;
  for (size_t i = 0; i + 1 < sizeof(t)/sizeof(t[0]); i++) {
    if (v <= t[i].v && v >= t[i+1].v) {
      float k = (v - t[i+1].v) / (t[i].v - t[i+1].v);
      return (int)roundf(t[i+1].p + k * (t[i].p - t[i+1].p));
    }
  }
  return -1;
}

static void batteryUpdate() {
  if (BATTERY_ADC_PIN < 0) return;
  if (millis() - last_batt_ms < 1500) return;
  last_batt_ms = millis();

  // Подстраховка: если ADC не настроен — всё равно попробуем
  analogReadResolution(12);
  int raw = analogRead(BATTERY_ADC_PIN); // 0..4095
  float v_adc = (raw / 4095.0f) * 3.3f;
  float v_batt = v_adc * BATTERY_DIVIDER_RATIO;

  battery_v = v_batt;
  battery_pct = socFromVoltage(v_batt);
}

// -------------------- Settings load/save --------------------
static void loadSettings() {
  prefs.begin("rls", true);
  cfg.center_us     = prefs.getInt("center", 1500);
  cfg.left_span_us  = prefs.getInt("lspan", 100);
  cfg.right_span_us = prefs.getInt("rspan", 100);
  cfg.bias_right_us = prefs.getInt("biasR", 0);
  cfg.bias_left_us  = prefs.getInt("biasL", 0);
  cfg.scan_period_ms= prefs.getInt("period", 9000);
  cfg.tick_ms       = prefs.getInt("tick", 20);
  cfg.servo_hold    = prefs.getBool("hold", true);
  prefs.end();

  // Безопасные границы
  cfg.center_us      = clampi(cfg.center_us, 800, 2200);
  cfg.left_span_us   = clampi(cfg.left_span_us, 0, 600);
  cfg.right_span_us  = clampi(cfg.right_span_us, 0, 600);
  cfg.bias_right_us  = clampi(cfg.bias_right_us, -200, 200);
  cfg.bias_left_us   = clampi(cfg.bias_left_us, -200, 200);
  cfg.scan_period_ms = clampi(cfg.scan_period_ms, 1000, 30000);
  cfg.tick_ms        = clampi(cfg.tick_ms, 10, 200);
}

static void saveSettings() {
  prefs.begin("rls", false);
  prefs.putInt("center", cfg.center_us);
  prefs.putInt("lspan", cfg.left_span_us);
  prefs.putInt("rspan", cfg.right_span_us);
  prefs.putInt("biasR", cfg.bias_right_us);
  prefs.putInt("biasL", cfg.bias_left_us);
  prefs.putInt("period", cfg.scan_period_ms);
  prefs.putInt("tick", cfg.tick_ms);
  prefs.putBool("hold", cfg.servo_hold);
  prefs.end();
}

// -------------------- Servo scan update (главное: НЕ “прибавлять step”, а считать по времени) --------------------
static int computeServoUsFromPhase(uint32_t now) {
  int minUs = cfg.center_us - cfg.left_span_us;
  int maxUs = cfg.center_us + cfg.right_span_us;
  minUs = clampi(minUs, 500, 2500);
  maxUs = clampi(maxUs, 500, 2500);

  uint32_t T = (uint32_t)cfg.scan_period_ms;
  if (T < 1000) T = 1000;
  uint32_t t = (now - scan_start_ms) % T;

  float half = T * 0.5f;
  float f;
  if (t < half) {
    f = (float)t / half;     // 0..1 (лево -> право)
    servo_dir = +1;
  } else {
    f = (float)(T - t) / half; // 1..0 (право -> лево)
    servo_dir = -1;
  }

  float us = minUs + (maxUs - minUs) * f;

  // Directional bias (если надо компенсировать “серво несимметричное”)
  if (servo_dir > 0) us += cfg.bias_right_us;
  else us += cfg.bias_left_us;

  return clampi((int)roundf(us), 500, 2500);
}

static void updateServo() {
  uint32_t now = millis();

  if (!servo_enabled) return;
  if (now - last_servo_update_ms < (uint32_t)cfg.tick_ms) return;
  last_servo_update_ms = now;

  int us = computeServoUsFromPhase(now);
  servoWriteUs(us);
}

// -------------------- Radar power toggle --------------------
static void setRadarPower(bool on) {
  radar_powered = on;
  if (RADAR_PWR_PIN < 0) return;

  pinMode(RADAR_PWR_PIN, OUTPUT);
  if (RADAR_PWR_ACTIVE_HIGH) digitalWrite(RADAR_PWR_PIN, on ? HIGH : LOW);
  else digitalWrite(RADAR_PWR_PIN, on ? LOW : HIGH);
}

// -------------------- HTML --------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>РЛС (ESP32‑C6)</title>
<style>
  :root{ --g:#00ff66; --g2:#00aa44; --bg:#000; --card:#07120b; --b:#0b2a16; }
  body{ margin:0; background:var(--bg); color:var(--g); font-family:system-ui,Segoe UI,Roboto,Arial; }
  .top{ position:sticky; top:0; background:#000; border-bottom:1px solid var(--b); padding:10px; display:flex; gap:8px; align-items:center; z-index:5;}
  .btn{ background:#06210f; border:1px solid var(--b); color:var(--g); padding:10px 12px; border-radius:12px; font-weight:600;}
  .btn:active{ transform:scale(.99); }
  .tabbtn{ opacity:.9; }
  .tabbtn.active{ border-color:var(--g); }
  .wrap{ padding:12px; max-width:900px; margin:0 auto; }
  .card{ background:var(--card); border:1px solid var(--b); border-radius:16px; padding:12px; margin:12px 0; }
  canvas{ width:100%; height:auto; display:block; border-radius:14px; background:#000; border:1px solid var(--b); }
  .row{ display:flex; gap:10px; flex-wrap:wrap; align-items:center; }
  .stat{ font-size:18px; font-weight:700; }
  .sub{ opacity:.9; font-family:ui-monospace,Menlo,Consolas,monospace; }
  .slider{ width:100%; }
  label{ display:block; margin-top:12px; font-weight:700; }
  .kv{ display:flex; justify-content:space-between; gap:12px; font-family:ui-monospace,Menlo,Consolas,monospace; opacity:.95; }
  .hr{ height:1px; background:var(--b); margin:12px 0; }
  .pill{ border:1px solid var(--b); padding:6px 10px; border-radius:999px; background:#03180a; }
  .danger{ border-color:#ff3355; color:#ff88aa; }
  .ok{ border-color:#00ff66; }
</style>
</head>
<body>
  <div class="top">
    <button class="btn tabbtn active" id="tRadar" onclick="showTab('radar')">Радар</button>
    <button class="btn tabbtn" id="tServo" onclick="showTab('servo')">Сервопривод</button>
    <span class="pill" id="wifiPill">AP: 192.168.4.1</span>
    <span class="pill" id="batPill">Батарея: —</span>
  </div>

  <div class="wrap">

    <div id="tab-radar">
      <div class="card">
        <canvas id="cv" width="700" height="420"></canvas>
      </div>

      <div class="card">
        <div class="stat" id="statusTitle">Статус: …</div>
        <div class="sub" id="statusLine">…</div>
        <div class="hr"></div>

        <div class="row">
          <button class="btn" onclick="toggleServo()" id="btnServo">Сканирование: …</button>
          <button class="btn" onclick="toggleRadar()" id="btnRadar">Радар: …</button>
          <button class="btn" onclick="toggleVoice()" id="btnVoice">Озвучка: …</button>
        </div>

        <div class="hr"></div>
        <div class="sub" id="hint">
          • Точки: цвет/размер зависят от скорости и направления.<br>
          • Если озвучка не говорит — нажми любую кнопку (некоторые браузеры требуют жест пользователя).
        </div>
      </div>
    </div>

    <div id="tab-servo" style="display:none">
      <div class="card">
        <div class="stat">Настройки сервопривода</div>

        <label>Center (µs): <span id="vCenter">—</span></label>
        <input class="slider" id="center" type="range" min="800" max="2200" step="1" />

        <label>Left span (µs): <span id="vL">—</span></label>
        <input class="slider" id="lspan" type="range" min="0" max="400" step="1" />

        <label>Right span (µs): <span id="vR">—</span></label>
        <input class="slider" id="rspan" type="range" min="0" max="400" step="1" />

        <label>Bias right (µs): <span id="vBR">—</span></label>
        <input class="slider" id="biasR" type="range" min="-60" max="60" step="1" />

        <label>Bias left (µs): <span id="vBL">—</span></label>
        <input class="slider" id="biasL" type="range" min="-60" max="60" step="1" />

        <label>Scan period (ms): <span id="vP">—</span></label>
        <input class="slider" id="period" type="range" min="2000" max="20000" step="100" />

        <label>Tick (ms): <span id="vT">—</span></label>
        <input class="slider" id="tick" type="range" min="10" max="120" step="1" />

        <div class="hr"></div>
        <div class="row">
          <button class="btn" onclick="centerNow()">Center (встать в центр)</button>
          <button class="btn" onclick="testLeft()">Тест Лево</button>
          <button class="btn" onclick="testRight()">Тест Право</button>
        </div>

        <div class="row" style="margin-top:10px">
          <button class="btn ok" onclick="apply()">Применить</button>
          <button class="btn ok" onclick="save()">Save (сохранить)</button>
          <button class="btn danger" onclick="resetPrefs()">Сброс настроек</button>
        </div>

        <div class="hr"></div>
        <div class="sub" id="diag">Диагностика: …</div>
      </div>
    </div>

  </div>

<script>
let S = null;
let inited=false;
let blips=[];
const BLIP_LIFE=4500;

let voiceEnabled = JSON.parse(localStorage.getItem('voiceEnabled') ?? 'true');
let lastSpeak=0;
let lastClear=0;
let lastAnyDet=0;
const SPEAK_COOLDOWN=2500;
const CLEAR_AFTER=3500;
const CLEAR_COOLDOWN=15000;

function showTab(name){
  document.getElementById('tab-radar').style.display = (name==='radar')?'block':'none';
  document.getElementById('tab-servo').style.display = (name==='servo')?'block':'none';
  document.getElementById('tRadar').classList.toggle('active', name==='radar');
  document.getElementById('tServo').classList.toggle('active', name==='servo');
}

function speak(txt){
  if(!voiceEnabled) return;
  const now=Date.now();
  if(now-lastSpeak < SPEAK_COOLDOWN) return;
  lastSpeak=now;
  try{
    window.speechSynthesis.cancel();
    const u=new SpeechSynthesisUtterance(txt);
    u.lang='ru-RU';
    window.speechSynthesis.speak(u);
  }catch(e){}
}

function toggleVoice(){
  voiceEnabled=!voiceEnabled;
  localStorage.setItem('voiceEnabled', JSON.stringify(voiceEnabled));
  updateButtons();
}

function clamp(v,a,b){ return Math.max(a, Math.min(b, v)); }

function classify(speed){
  // speed < 0 => к нам, >0 => от нас
  const a=Math.abs(speed);
  let size=2, color='#ffff33';
  if(a < 1.2){ size=2; color='#ffff33'; }
  else if(a < 6){ size=3; color=(speed<0)?'#ff8800':'#00aaff'; }      // “пешком/медленно”
  else if(a < 18){ size=4; color=(speed<0)?'#ff2222':'#22ffff'; }     // “быстро”
  else { size=6; color=(speed<0)?'#ff00ff':'#8888ff'; }               // “транспорт/очень быстро”
  return {size,color};
}

function addBlip(angle, dist_cm, speed){
  const now=Date.now();
  blips.push({angle, dist_cm, speed, t:now});
  // чистим старые
  blips = blips.filter(b => now-b.t < BLIP_LIFE);
}

function draw(){
  const cv=document.getElementById('cv');
  const ctx=cv.getContext('2d');

  const W=cv.width, H=cv.height;
  ctx.clearRect(0,0,W,H);

  if(!S || !S.radarEnabled){
    // радар выключен — просто надпись
    ctx.fillStyle='#00ff66';
    ctx.font='28px system-ui';
    ctx.fillText('РАДАР ВЫКЛЮЧЕН', 40, 80);
    return;
  }

  // grid
  const ox=W/2, oy=H-10;
  const R=Math.min(W/2-20, H-20);

  ctx.strokeStyle='#00aa44';
  ctx.lineWidth=2;

  // дуги
  for(let k=1;k<=4;k++){
    ctx.beginPath();
    ctx.arc(ox, oy, R*(k/4), Math.PI, 0);
    ctx.stroke();
  }
  // лучи
  for(let a=0;a<=180;a+=30){
    const rad=(180-a)*Math.PI/180;
    const x=ox+Math.cos(rad)*R;
    const y=oy-Math.sin(rad)*R;
    ctx.beginPath();
    ctx.moveTo(ox,oy);
    ctx.lineTo(x,y);
    ctx.stroke();
  }
  // базовая линия
  ctx.beginPath();
  ctx.moveTo(ox-R, oy);
  ctx.lineTo(ox+R, oy);
  ctx.stroke();

  // blips
  const maxRange = 600; // см (можно потом сделать настройкой)
  const now=Date.now();
  for(const b of blips){
    const age=now-b.t;
    const alpha=clamp(1 - age/BLIP_LIFE, 0, 1);
    const rr=clamp(b.dist_cm/maxRange, 0, 1);
    const rad=(180-b.angle)*Math.PI/180;
    const x=ox+Math.cos(rad)*R*rr;
    const y=oy-Math.sin(rad)*R*rr;

    const cs=classify(b.speed);
    ctx.globalAlpha=alpha;
    ctx.fillStyle=cs.color;
    ctx.beginPath();
    ctx.arc(x,y, cs.size+2, 0, Math.PI*2);
    ctx.fill();
  }
  ctx.globalAlpha=1;

  // sweep
  const rad=(180-S.angle)*Math.PI/180;
  const sx=ox+Math.cos(rad)*R;
  const sy=oy-Math.sin(rad)*R;
  ctx.strokeStyle='#00ff66';
  ctx.lineWidth=4;
  ctx.beginPath();
  ctx.moveTo(ox,oy);
  ctx.lineTo(sx,sy);
  ctx.stroke();
}

function updateButtons(){
  document.getElementById('btnVoice').textContent = 'Озвучка: ' + (voiceEnabled?'ВКЛ':'ВЫКЛ');
  if(S){
    document.getElementById('btnServo').textContent = 'Сканирование: ' + (S.servoEnabled?'СТОП':'СТАРТ');
    document.getElementById('btnRadar').textContent = 'Радар: ' + (S.radarEnabled?'ВЫКЛ':'ВКЛ');
  }
}

function uiInit(){
  if(inited || !S) return;
  inited=true;

  const set = (id,val)=>{ const el=document.getElementById(id); el.value=val; };
  set('center', S.center);
  set('lspan', S.lspan);
  set('rspan', S.rspan);
  set('biasR', S.biasR);
  set('biasL', S.biasL);
  set('period', S.period);
  set('tick', S.tick);

  bindSliders();
  refreshSliderText();
  updateButtons();
}

function refreshSliderText(){
  const g=(id)=>document.getElementById(id);
  document.getElementById('vCenter').textContent = g('center').value;
  document.getElementById('vL').textContent = g('lspan').value;
  document.getElementById('vR').textContent = g('rspan').value;
  document.getElementById('vBR').textContent = g('biasR').value;
  document.getElementById('vBL').textContent = g('biasL').value;
  document.getElementById('vP').textContent = g('period').value;
  document.getElementById('vT').textContent = g('tick').value;
}

let applyTimer=null;
function bindSliders(){
  const ids=['center','lspan','rspan','biasR','biasL','period','tick'];
  for(const id of ids){
    document.getElementById(id).addEventListener('input', ()=>{
      refreshSliderText();
      // лёгкий debounce
      if(applyTimer) clearTimeout(applyTimer);
      applyTimer=setTimeout(apply, 200);
    });
  }
}

async function apply(){
  const q = new URLSearchParams({
    center:document.getElementById('center').value,
    lspan:document.getElementById('lspan').value,
    rspan:document.getElementById('rspan').value,
    biasR:document.getElementById('biasR').value,
    biasL:document.getElementById('biasL').value,
    period:document.getElementById('period').value,
    tick:document.getElementById('tick').value
  });
  await fetch('/api/set?'+q.toString(), {cache:'no-store'});
}

async function save(){
  await fetch('/api/set?save=1', {cache:'no-store'});
  speak('Настройки сохранены');
}

async function resetPrefs(){
  if(!confirm('Сбросить настройки сервопривода?')) return;
  await fetch('/api/reset', {cache:'no-store'});
}

async function toggleServo(){
  if(!S) return;
  await fetch('/api/set?servo='+(S.servoEnabled?0:1), {cache:'no-store'});
}

async function toggleRadar(){
  if(!S) return;
  await fetch('/api/set?radar='+(S.radarEnabled?0:1), {cache:'no-store'});
}

async function centerNow(){
  await fetch('/api/set?centerNow=1', {cache:'no-store'});
}

async function testLeft(){
  await fetch('/api/set?test=left', {cache:'no-store'});
}
async function testRight(){
  await fetch('/api/set?test=right', {cache:'no-store'});
}

function updateStatus(){
  if(!S) return;

  const st = document.getElementById('statusTitle');
  const ln = document.getElementById('statusLine');
  const diag = document.getElementById('diag');

  const pres = S.presence ? 'Движение' : 'Тихо';
  st.textContent = 'Статус: ' + (S.radarEnabled ? pres : 'Радар выключен');

  ln.textContent = `angle=${S.angle}°, us=${S.us}, dir=${S.dir}, OUT=${S.out}, range=${S.range_cm}cm, v=${S.speed_kmh.toFixed(1)}km/h`;

  diag.innerHTML =
    `Диагностика:<br>`+
    `• servoEnabled=${S.servoEnabled}, servoAttached=${S.servoAttached}<br>`+
    `• center=${S.center}, L=${S.lspan}, R=${S.rspan}, biasR=${S.biasR}, biasL=${S.biasL}<br>`+
    `• period=${S.period}ms, tick=${S.tick}ms<br>`+
    `• range=${S.range_cm}cm, speed=${S.speed_kmh.toFixed(1)}km/h, moveDir=${S.moveDir}`;

  // батарея
  const bp = document.getElementById('batPill');
  if(S.batt_pct >= 0){
    bp.textContent = `Батарея: ${S.batt_pct}% (${S.batt_v.toFixed(2)}V)`;
  }else{
    bp.textContent = 'Батарея: —';
  }

  updateButtons();
}

let lastAnnouncedMove = 0; // -1/+1/0
function handleVoiceAndBlips(){
  if(!S || !S.radarEnabled) return;

  const now=Date.now();

  if(S.presence && S.range_cm > 0){
    addBlip(S.angle, S.range_cm, S.speed_kmh);
    lastAnyDet = now;

    // озвучка только когда реально "движется" и меняется направление
    if(S.moveDir !== 0 && S.moveDir !== lastAnnouncedMove){
      lastAnnouncedMove = S.moveDir;
      const sp = Math.abs(S.speed_kmh).toFixed(0);
      if(S.moveDir < 0) speak(`Объект движется к нам. Скорость ${sp} километров в час`);
      else speak(`Объект удаляется. Скорость ${sp} километров в час`);
    }
  }else{
    // если долго никого
    if(now-lastAnyDet > CLEAR_AFTER && now-lastClear > CLEAR_COOLDOWN){
      lastClear = now;
      lastAnnouncedMove = 0;
      speak('Пространство чисто');
    }
  }
}

async function poll(){
  try{
    const r = await fetch('/api/state', {cache:'no-store'});
    S = await r.json();
    uiInit();
    updateStatus();
    handleVoiceAndBlips();
    draw();
  }catch(e){
    // молча
  }
  const d = (S && S.tick) ? Math.max(80, Math.min(400, S.tick*3)) : 200;
  setTimeout(poll, d);
}

updateButtons();
poll();
</script>
</body>
</html>
)HTML";

// -------------------- API handlers --------------------
static void handleIndex() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleState() {
  // OUT пин (если есть)
  int outv = -1;
  if (RADAR_OUT_PIN >= 0) outv = digitalRead(RADAR_OUT_PIN);

  int angle = servoAngleDegFromUs(servo_us_cmd);

  // если UART давно молчит — можно fallback на OUT
  bool pres = radar_presence;
  if (millis() - last_radar_line_ms > 2500 && outv != -1) pres = (outv == HIGH);

  String json = "{";
  json += "\"servoEnabled\":" + String(servo_enabled ? "true" : "false") + ",";
  json += "\"servoAttached\":" + String(servo_attached ? "true" : "false") + ",";
  json += "\"radarEnabled\":" + String(radar_enabled ? "true" : "false") + ",";
  json += "\"angle\":" + String(angle) + ",";
  json += "\"us\":" + String(servo_us_cmd) + ",";
  json += "\"dir\":\"" + String(servo_dir > 0 ? "R" : "L") + "\",";
  json += "\"out\":" + String(outv) + ",";
  json += "\"presence\":" + String(pres ? "true" : "false") + ",";
  json += "\"range_cm\":" + String(radar_range_cm) + ",";
  json += "\"speed_kmh\":" + String(speed_kmh, 2) + ",";
  json += "\"moveDir\":" + String(move_dir) + ",";

  json += "\"center\":" + String(cfg.center_us) + ",";
  json += "\"lspan\":" + String(cfg.left_span_us) + ",";
  json += "\"rspan\":" + String(cfg.right_span_us) + ",";
  json += "\"biasR\":" + String(cfg.bias_right_us) + ",";
  json += "\"biasL\":" + String(cfg.bias_left_us) + ",";
  json += "\"period\":" + String(cfg.scan_period_ms) + ",";
  json += "\"tick\":" + String(cfg.tick_ms) + ",";

  if (BATTERY_ADC_PIN >= 0 && !isnan(battery_v) && battery_pct >= 0) {
    json += "\"batt_v\":" + String(battery_v, 3) + ",";
    json += "\"batt_pct\":" + String(battery_pct);
  } else {
    json += "\"batt_v\":0,";
    json += "\"batt_pct\":-1";
  }

  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

static void handleSet() {
  // настройки
  if (server.hasArg("center")) cfg.center_us = clampi(server.arg("center").toInt(), 800, 2200);
  if (server.hasArg("lspan"))  cfg.left_span_us = clampi(server.arg("lspan").toInt(), 0, 600);
  if (server.hasArg("rspan"))  cfg.right_span_us = clampi(server.arg("rspan").toInt(), 0, 600);
  if (server.hasArg("biasR"))  cfg.bias_right_us = clampi(server.arg("biasR").toInt(), -200, 200);
  if (server.hasArg("biasL"))  cfg.bias_left_us = clampi(server.arg("biasL").toInt(), -200, 200);
  if (server.hasArg("period")) cfg.scan_period_ms = clampi(server.arg("period").toInt(), 1000, 30000);
  if (server.hasArg("tick"))   cfg.tick_ms = clampi(server.arg("tick").toInt(), 10, 200);

  // команды
  if (server.hasArg("servo")) {
    int v = server.arg("servo").toInt();
    if (v == 1) {
      servo_enabled = true;
      scan_start_ms = millis();
      // чтобы серво не “дёрнулось”, ставим центр и только потом начинаем скан
      servoWriteUs(cfg.center_us);
    } else {
      servo_enabled = false;
      if (cfg.servo_hold) {
        // остаёмся удерживать текущую позицию
        servoWriteUs(servo_us_cmd);
      } else {
        // полностью отключаем PWM
        servoDetachIfNeeded();
      }
    }
  }

  if (server.hasArg("radar")) {
    int v = server.arg("radar").toInt();
    radar_enabled = (v == 1);
    if (!radar_enabled) {
      move_dir = 0;
      speed_kmh = 0;
      radar_presence = false;
      radar_range_cm = 0;
      if (RADAR_PWR_PIN >= 0) setRadarPower(false);
    } else {
      if (RADAR_PWR_PIN >= 0) setRadarPower(true);
    }
  }

  if (server.hasArg("centerNow")) {
    servoWriteUs(cfg.center_us);
  }

  if (server.hasArg("test")) {
    String t = server.arg("test");
    int leftUs = cfg.center_us - cfg.left_span_us;
    int rightUs = cfg.center_us + cfg.right_span_us;
    if (t == "left") servoWriteUs(leftUs);
    if (t == "right") servoWriteUs(rightUs);
  }

  if (server.hasArg("save")) {
    saveSettings();
  }

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "OK");
}

static void handleResetPrefs() {
  prefs.begin("rls", false);
  prefs.clear();
  prefs.end();
  server.send(200, "text/plain", "RESET");
  delay(200);
  ESP.restart();
}

// -------------------- Setup/Loop --------------------
void setup() {
#if !RADAR_USE_SERIAL0
  Serial.begin(115200);
  delay(200);
  DBGLN("Boot...");
#endif

  loadSettings();

  // По умолчанию: серво НЕ стартует
  servo_enabled = false;
  servo_attached = false;
  servo_us_cmd = cfg.center_us;

  if (RADAR_OUT_PIN >= 0) {
    pinMode(RADAR_OUT_PIN, INPUT);
  }

  if (RADAR_PWR_PIN >= 0) setRadarPower(true);

  // UART радара
#if RADAR_USE_SERIAL0
  Serial.begin(RADAR_BAUD);
#else
  RadarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#endif

  // Пытаемся перевести LD2420 в Basic text mode (ON/OFF + Range XXXX)
  if (radar_enabled) {
    delay(250);
    ld2420SetBasicStatusMode();
#if !RADAR_USE_SERIAL0
    DBGLN("LD2420: set basic status mode");
#endif
  }

  // Wi‑Fi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
#if !RADAR_USE_SERIAL0
  DBGLN("AP started: " + WiFi.softAPIP().toString());
#endif

  // routes
  server.on("/", handleIndex);
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/set", HTTP_GET, handleSet);
  server.on("/api/reset", HTTP_GET, handleResetPrefs);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
}

void loop() {
  server.handleClient();
  radarPollSerial();
  updateServo();
  batteryUpdate();
}
