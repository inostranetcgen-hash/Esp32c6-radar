#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>

// -------------------- ВЕРСИЯ --------------------
static const char* APP_VERSION = "RLS-C6 v2.0 (servo stable + power logic + battery)";

// -------------------- WIFI AP --------------------
static const char* AP_SSID = "RLS-ESP32C6";
static const char* AP_PASS = "12345678"; // минимум 8 символов

WebServer server(80);
Preferences prefs;

// -------------------- ПИНЫ --------------------
// Сервопривод (как у тебя)
static const int SERVO_PIN = 18;

// Опционально: если поставишь MOSFET на питание сервопривода 5V,
// то укажи пин управления тут. -1 = нет ключа питания.
static const int SERVO_PWR_PIN = -1;   // например 5, 6, 7... (любой свободный GPIO)

// Радар OUT (если подключён) — у тебя часто это GPIO0
static const int RADAR_OUT_PIN = 0;

// Опционально: если поставишь MOSFET/ключ питания 3.3V на радар
static const int RADAR_PWR_PIN = -1;   // любой свободный GPIO, через MOSFET/LoadSwitch

// Опционально: батарея через делитель на ADC
static const int BAT_ADC_PIN = -1;     // например 1,2,3... (любой ADC GPIO)
static const float BAT_DIVIDER = 2.0f; // 100k/100k => 2.0
static const float BAT_VMIN = 3.20f;   // 0%
static const float BAT_VMAX = 4.20f;   // 100%

// -------------------- УТИЛИТЫ --------------------
static int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

static String jsonEscape(const String& s) {
  String out; out.reserve(s.length()+8);
  for (size_t i=0;i<s.length();i++){
    char c = s[i];
    if (c=='\\') out += "\\\\";
    else if (c=='"') out += "\\\"";
    else if (c=='\n') out += "\\n";
    else if (c=='\r') out += "\\r";
    else out += c;
  }
  return out;
}

// -------------------- КОНФИГ --------------------
struct Config {
  int center_us = 1500;
  int left_span_us = 100;
  int right_span_us = 100;
  int bias_right_us = 0;
  int bias_left_us = 0;

  int scan_period_ms = 9000;   // полный цикл туда-обратно
  int tick_ms = 120;           // обновление UI (мс)
  int ease_pct = 35;           // 0..100 (плавное замедление к краям)
  int end_hold_ms = 0;         // пауза в краях (лучше 0)

  bool servo_hold_when_stop = false; // если true — держать последнюю позицию (PWM остается). Если false — PWM OFF.
  bool failsafe_stop = false;        // если true — без активности в UI остановит серво и радар
  int  failsafe_sec = 60;            // через сколько секунд

  // радар
  bool radar_enabled_default = false; // при старте выключен
};

Config cfg;

// -------------------- СОСТОЯНИЯ --------------------
static bool servo_scan_on = false;       // “Сканирование: СТАРТ/СТОП”
static bool servo_pwm_on  = false;       // реально подаём импульсы или нет
static bool radar_on      = false;       // логика радара вкл/выкл
static bool radar_powered = false;       // питание через ключ (если есть)

static uint32_t scan_start_ms = 0;
static int servo_us_cmd = 1500;
static char servo_dir = 'R';

static uint32_t last_client_ms = 0;

// -------------------- SPEAK EVENT (для браузера) --------------------
static uint32_t speak_seq = 0;
static String speak_text = "";

// -------------------- BAT --------------------
static uint32_t last_bat_ms = 0;
static float bat_v = NAN;
static int bat_pct = -1;

// -------------------- RADAR “движение/тихо” --------------------
static bool motion_now = false;
static bool motion_prev = false;
static uint32_t last_motion_ms = 0;
static bool said_clear = false;

// -------------------- HIT POINTS --------------------
struct Hit {
  float angle_deg;
  float range_cm;
  float v_kmh;
  int dir;              // +1 к нам, -1 от нас, 0 неизвестно
  uint32_t ts;
};
static const int MAX_HITS = 32;
static Hit hits[MAX_HITS];
static int hit_head = 0;

static void addHit(float angle, float range_cm, float v_kmh, int dir) {
  hits[hit_head] = {angle, range_cm, v_kmh, dir, millis()};
  hit_head = (hit_head + 1) % MAX_HITS;
}

// -------------------- ПИТАНИЕ КЛЮЧЕЙ --------------------
static void servoPower(bool on) {
  if (SERVO_PWR_PIN >= 0) {
    pinMode(SERVO_PWR_PIN, OUTPUT);
    digitalWrite(SERVO_PWR_PIN, on ? HIGH : LOW);
  }
}
static void radarPower(bool on) {
  if (RADAR_PWR_PIN >= 0) {
    pinMode(RADAR_PWR_PIN, OUTPUT);
    digitalWrite(RADAR_PWR_PIN, on ? HIGH : LOW);
    radar_powered = on;
  } else {
    radar_powered = on; // логическое состояние (для UI)
  }
}

// -------------------- SERVO: РУЧНОЙ PWM 50Hz --------------------
static uint32_t next_pulse_ms = 0;

static void servoPWM(bool on) {
  servo_pwm_on = on;
  if (!on) {
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
  } else {
    next_pulse_ms = millis();
  }
}

static void servoPulseUs(int us) {
  us = clampi(us, 500, 2500);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
}

static void servoService50Hz() {
  if (!servo_pwm_on) return;
  uint32_t now = millis();
  if ((int32_t)(now - next_pulse_ms) >= 0) {
    servoPulseUs(servo_us_cmd);
    next_pulse_ms = now + 20;
  }
}

// Плавная траектория без накопления ошибок
static float smoothstep(float x) { return x * x * (3.0f - 2.0f * x); }

static void servoScanService() {
  if (!servo_scan_on) return;

  int period = clampi(cfg.scan_period_ms, 800, 60000);
  uint32_t now = millis();

  // треугольник 0..1..0
  uint32_t t = (now - scan_start_ms) % (uint32_t)period;
  float p = (float)t / (float)period; // 0..1

  float x; // 0..1
  if (p < 0.5f) {
    x = p * 2.0f;      // 0..1
    servo_dir = 'R';
  } else {
    x = (1.0f - p) * 2.0f; // 1..0
    servo_dir = 'L';
  }

  // easing
  float mix = clampf(cfg.ease_pct / 100.0f, 0, 1);
  float xe = x * (1.0f - mix) + smoothstep(x) * mix;

  int usL = cfg.center_us - cfg.left_span_us;
  int usR = cfg.center_us + cfg.right_span_us;
  if (usR <= usL) usR = usL + 1;

  int us = (int)roundf(usL + (usR - usL) * xe);

  // направленные bias
  if (servo_dir == 'R') us += cfg.bias_right_us;
  else us += cfg.bias_left_us;

  // крайние ограничения
  int minUs = usL - 300;
  int maxUs = usR + 300;
  servo_us_cmd = clampi(us, minUs, maxUs);
}

// угол для UI (0..180)
static float servoAngleDeg() {
  int usL = cfg.center_us - cfg.left_span_us;
  int usR = cfg.center_us + cfg.right_span_us;
  if (usR <= usL) return 90;
  float pos = (float)(servo_us_cmd - usL) / (float)(usR - usL);
  pos = clampf(pos, 0, 1);
  return pos * 180.0f;
}

// -------------------- BATTERY --------------------
static int batteryPercentFromV(float v) {
  // Литий 1S приблизительно (можно потом уточнить по твоей батарее)
  if (v <= 3.20f) return 0;
  if (v >= 4.20f) return 100;
  // кусочно-линейно (лучше чем просто линейка)
  struct Pt { float v; int p; };
  static const Pt pts[] = {
    {3.20f,0}, {3.30f,3}, {3.40f,8}, {3.50f,15}, {3.60f,25},
    {3.70f,40}, {3.80f,55}, {3.90f,70}, {4.00f,85}, {4.10f,95}, {4.20f,100}
  };
  for (size_t i=1;i<sizeof(pts)/sizeof(pts[0]);i++){
    if (v <= pts[i].v) {
      float v0=pts[i-1].v, v1=pts[i].v;
      int p0=pts[i-1].p, p1=pts[i].p;
      float k=(v-v0)/(v1-v0);
      return (int)roundf(p0 + (p1-p0)*k);
    }
  }
  return 100;
}

static void batteryService() {
  if (BAT_ADC_PIN < 0) { bat_pct = -1; return; }
  uint32_t now = millis();
  if (now - last_bat_ms < 900) return;
  last_bat_ms = now;

  analogReadResolution(12);

  uint32_t mv = analogReadMilliVolts(BAT_ADC_PIN);
  float vadc = (float)mv / 1000.0f;
  float v = vadc * BAT_DIVIDER;
  bat_v = v;
  bat_pct = batteryPercentFromV(v);
}

// -------------------- RADAR SERVICE --------------------
static void radarService() {
  if (!radar_on) return;

  // Самый базовый вариант: OUT-пин = движение/тихо
  bool m = false;
  if (RADAR_OUT_PIN >= 0) {
    pinMode(RADAR_OUT_PIN, INPUT);
    m = (digitalRead(RADAR_OUT_PIN) == HIGH);
  }

  motion_prev = motion_now;
  motion_now = m;

  if (motion_now) last_motion_ms = millis();

  // Если движение есть — добавляем "точку" на текущем угле
  // range и speed — пока базово (если у тебя есть UART-данные, добавим позже)
  if (motion_now) {
    float ang = servoAngleDeg();
    float range_cm = 120.0f; // ПЛЕЙСХОЛДЕР: без UART дальность неизвестна
    float v_kmh = 0.0f;
    int dir = 0;
    addHit(ang, range_cm, v_kmh, dir);
    said_clear = false;
  }

  // речь (события)
  if (motion_now && !motion_prev) {
    speak_seq++;
    speak_text = "Обнаружено движение";
  }

  // “пространство чисто” после тишины
  if (!motion_now) {
    uint32_t now = millis();
    if (!said_clear && (now - last_motion_ms) > 5000) {
      speak_seq++;
      speak_text = "Пространство чисто";
      said_clear = true;
    }
  }
}

// -------------------- FAILSAFE --------------------
static void failsafeService() {
  if (!cfg.failsafe_stop) return;
  uint32_t now = millis();
  if ((now - last_client_ms) > (uint32_t)cfg.failsafe_sec * 1000UL) {
    // выключаем всё
    servo_scan_on = false;
    if (!cfg.servo_hold_when_stop) servoPWM(false);
    radar_on = false;
    radarPower(false);
  }
}

// -------------------- HTML UI --------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="ru"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RLS</title>
<style>
  body{margin:0;background:#060b07;color:#8fffa8;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial}
  .top{display:flex;gap:10px;padding:10px;background:#020402;position:sticky;top:0;z-index:2}
  .btn{border:1px solid #1e6b3a;background:rgba(0,0,0,.4);color:#8fffa8;padding:10px 14px;border-radius:22px;font-weight:650}
  .btn:active{transform:scale(.98)}
  .pill{border:1px solid #1e6b3a;background:rgba(0,0,0,.35);padding:10px 14px;border-radius:22px}
  .wrap{max-width:900px;margin:0 auto;padding:12px}
  .card{border:1px solid #1e6b3a;border-radius:18px;padding:14px;margin:12px 0;background:rgba(0,0,0,.25)}
  canvas{width:100%;height:auto;border-radius:16px;border:1px solid #1e6b3a;background:#000}
  .row{display:flex;gap:10px;flex-wrap:wrap}
  .grow{flex:1}
  .label{opacity:.9}
  input[type=range]{width:100%}
  .small{font-size:13px;opacity:.9}
  .tab{display:none}
  .tab.on{display:block}
  .big{font-size:22px;font-weight:800}
</style>
</head>
<body>
  <div class="top">
    <button class="btn" onclick="showTab('radar')">Радар</button>
    <button class="btn" onclick="showTab('servo')">Сервопривод</button>
    <button class="btn" onclick="showTab('bat')">Батарея</button>
    <div class="pill">AP: <span id="ip">—</span></div>
    <div class="pill">Заряд: <span id="batTop">—</span></div>
  </div>

  <div class="wrap">
    <!-- RADAR TAB -->
    <div id="tab-radar" class="tab on">
      <div class="card">
        <canvas id="cv" width="900" height="420"></canvas>
      </div>
      <div class="card">
        <div class="big">Статус: <span id="status">—</span></div>
        <div class="small" id="dbg">—</div>
        <div class="row" style="margin-top:12px">
          <button class="btn" id="btnScan" onclick="toggleScan()">Сканирование: СТАРТ</button>
          <button class="btn" id="btnRadar" onclick="toggleRadar()">Радар: ВКЛ</button>
          <button class="btn" id="btnVoice" onclick="toggleVoice()">Озвучка: ВКЛ</button>
        </div>
        <div class="small" style="margin-top:10px">
          Если озвучка молчит — нажми любую кнопку (браузеры требуют жест).
        </div>
      </div>
    </div>

    <!-- SERVO TAB -->
    <div id="tab-servo" class="tab">
      <div class="card">
        <div class="big">Настройки сервопривода</div>

        <div class="label">Center (us): <b id="cV">1500</b></div>
        <input id="center" type="range" min="1200" max="1800" value="1500" oninput="cV.textContent=this.value">

        <div class="label">Left span (us): <b id="lV">100</b></div>
        <input id="lspan" type="range" min="10" max="400" value="100" oninput="lV.textContent=this.value">

        <div class="label">Right span (us): <b id="rV">100</b></div>
        <input id="rspan" type="range" min="10" max="400" value="100" oninput="rV.textContent=this.value">

        <div class="label">Bias right (us): <b id="brV">0</b></div>
        <input id="br" type="range" min="-80" max="80" value="0" oninput="brV.textContent=this.value">

        <div class="label">Bias left (us): <b id="blV">0</b></div>
        <input id="bl" type="range" min="-80" max="80" value="0" oninput="blV.textContent=this.value">

        <div class="label">Scan period (ms): <b id="pV">9000</b></div>
        <input id="period" type="range" min="800" max="30000" value="9000" oninput="pV.textContent=this.value">

        <div class="label">Ease (% плавность к краям): <b id="eV">35</b></div>
        <input id="ease" type="range" min="0" max="100" value="35" oninput="eV.textContent=this.value">

        <div class="label">Tick UI (ms): <b id="tV">120</b></div>
        <input id="tick" type="range" min="80" max="600" value="120" oninput="tV.textContent=this.value">

        <div class="row" style="margin-top:12px">
          <button class="btn" onclick="servoCenter()">Center</button>
          <button class="btn" onclick="servoTest('L')">Тест Лево</button>
          <button class="btn" onclick="servoTest('R')">Тест Право</button>
        </div>

        <div class="row" style="margin-top:12px">
          <button class="btn" onclick="applyServo()">Применить</button>
          <button class="btn" onclick="saveCfg()">Save</button>
          <button class="btn" onclick="togglePWM()">PWM: ON/OFF</button>
        </div>

        <div class="card" style="margin-top:14px">
          <div class="big">Диагностика</div>
          <div class="small" id="servoDiag">—</div>
        </div>

      </div>
    </div>

    <!-- BAT TAB -->
    <div id="tab-bat" class="tab">
      <div class="card">
        <div class="big">Батарея</div>
        <div>Напряжение: <b id="batV">—</b></div>
        <div>Процент: <b id="batP">—</b></div>
        <div class="small" style="margin-top:10px">
          Чтобы заработало — нужен делитель напряжения на ADC (иначе будет "—").
        </div>
      </div>
    </div>

  </div>

<script>
let voiceOn = true;
let lastSpeakSeq = 0;
let scanOn = false;
let radarOn = false;
let pwmOn = false;

function showTab(name){
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('on'));
  document.getElementById('tab-'+name).classList.add('on');
}

function say(text){
  if(!voiceOn) return;
  if(!('speechSynthesis' in window)) return;
  const u = new SpeechSynthesisUtterance(text);
  u.lang = 'ru-RU';
  speechSynthesis.cancel();
  speechSynthesis.speak(u);
}

async function api(url){
  const r = await fetch(url);
  return await r.json();
}

function toggleVoice(){
  voiceOn = !voiceOn;
  document.getElementById('btnVoice').textContent = 'Озвучка: ' + (voiceOn?'ВКЛ':'ВЫКЛ');
  if(voiceOn) say('Озвучка включена');
}

async function toggleScan(){
  scanOn = !scanOn;
  await api('/api/cmd?scan='+(scanOn?1:0));
}

async function toggleRadar(){
  radarOn = !radarOn;
  await api('/api/cmd?radar='+(radarOn?1:0));
}

async function togglePWM(){
  pwmOn = !pwmOn;
  await api('/api/cmd?pwm='+(pwmOn?1:0));
}

async function applyServo(){
  const q = new URLSearchParams({
    center:center.value, lspan:lspan.value, rspan:rspan.value,
    br:br.value, bl:bl.value, period:period.value,
    ease:ease.value, tick:tick.value
  });
  await api('/api/set?'+q.toString());
}

async function saveCfg(){ await api('/api/save'); say('Сохранено'); }

async function servoCenter(){ await api('/api/cmd?center=1'); }

async function servoTest(dir){ await api('/api/cmd?test='+dir); }

function draw(state){
  const cv = document.getElementById('cv');
  const ctx = cv.getContext('2d');
  ctx.clearRect(0,0,cv.width,cv.height);

  if(!state.radar_on){
    ctx.fillStyle='#001a08';
    ctx.fillRect(0,0,cv.width,cv.height);
    ctx.fillStyle='#8fffa8';
    ctx.font='28px system-ui';
    ctx.fillText('РАДАР ВЫКЛЮЧЕН', 280, 210);
    return;
  }

  // grid
  ctx.fillStyle='#000';
  ctx.fillRect(0,0,cv.width,cv.height);
  ctx.strokeStyle='#1e6b3a';
  ctx.lineWidth=2;

  const cx = cv.width/2;
  const cy = cv.height-10;
  const R = Math.min(cv.width*0.47, cv.height*0.95);

  // arcs
  for(let i=1;i<=4;i++){
    ctx.beginPath();
    ctx.arc(cx,cy,R*i/4,Math.PI,2*Math.PI);
    ctx.stroke();
  }
  // spokes
  for(let a=0;a<=180;a+=30){
    const rad = Math.PI + (a*Math.PI/180);
    ctx.beginPath();
    ctx.moveTo(cx,cy);
    ctx.lineTo(cx+Math.cos(rad)*R, cy+Math.sin(rad)*R);
    ctx.stroke();
  }

  // sweep line
  const ang = state.servo_angle_deg || 0;
  const rad = Math.PI + (ang*Math.PI/180);
  ctx.strokeStyle='#00ff66';
  ctx.lineWidth=4;
  ctx.beginPath();
  ctx.moveTo(cx,cy);
  ctx.lineTo(cx+Math.cos(rad)*R, cy+Math.sin(rad)*R);
  ctx.stroke();

  // hits
  const hits = state.hits || [];
  const now = Date.now();
  for(const h of hits){
    const age = (now - h.ts);
    if(age > 7000) continue;
    const alpha = Math.max(0, 1 - age/7000);
    let col = 'rgba(0,255,120,'+alpha+')';
    let size = 6;

    if(h.dir > 0){ col = 'rgba(255,80,80,'+alpha+')'; size = 8 + Math.min(10, Math.abs(h.v_kmh||0)); }
    else if(h.dir < 0){ col = 'rgba(80,160,255,'+alpha+')'; size = 7 + Math.min(8, Math.abs(h.v_kmh||0)); }

    const a = Math.PI + (h.angle_deg*Math.PI/180);
    const rr = R * Math.min(1.0, (h.range_cm||120)/400.0); // 400см шкала по умолчанию
    const x = cx + Math.cos(a)*rr;
    const y = cy + Math.sin(a)*rr;

    ctx.fillStyle = col;
    ctx.beginPath();
    ctx.arc(x,y,size,0,Math.PI*2);
    ctx.fill();
  }
}

async function loop(){
  try{
    const state = await api('/api/state');

    ip.textContent = state.ip || '192.168.4.1';
    status.textContent = state.motion ? 'Движение' : 'Тихо';
    dbg.textContent = `angle=${(state.servo_angle_deg||0).toFixed(1)}°, us=${state.servo_us}, dir=${state.servo_dir}, OUT=${state.motion?1:0}`;

    scanOn = !!state.scan_on;
    radarOn = !!state.radar_on;
    pwmOn  = !!state.pwm_on;

    btnScan.textContent  = 'Сканирование: ' + (scanOn?'СТОП':'СТАРТ');
    btnRadar.textContent = 'Радар: ' + (radarOn?'ВЫКЛ':'ВКЛ');

    if(state.bat_pct>=0){
      batTop.textContent = state.bat_pct+'%';
      batV.textContent = (state.bat_v||0).toFixed(2)+'V';
      batP.textContent = state.bat_pct+'%';
      batP.style.opacity = 1;
    }else{
      batTop.textContent = '—';
      batV.textContent = '—';
      batP.textContent = '—';
    }

    // servo tab values (sync once)
    cV.textContent = center.value = state.cfg.center_us;
    lV.textContent = lspan.value  = state.cfg.left_span_us;
    rV.textContent = rspan.value  = state.cfg.right_span_us;
    brV.textContent = br.value    = state.cfg.bias_right_us;
    blV.textContent = bl.value    = state.cfg.bias_left_us;
    pV.textContent = period.value = state.cfg.scan_period_ms;
    eV.textContent = ease.value   = state.cfg.ease_pct;
    tV.textContent = tick.value   = state.cfg.tick_ms;

    // servo diag
    servoDiag.textContent =
      `PWM=${pwmOn?'ON':'OFF'} | Scan=${scanOn?'ON':'OFF'} | us=${state.servo_us} | `+
      `min=${state.min_us} max=${state.max_us} | совет: min>=900, max<=2100`;

    draw(state);

    // speak event
    if(state.speak_seq && state.speak_seq !== lastSpeakSeq){
      lastSpeakSeq = state.speak_seq;
      if(state.speak_text) say(state.speak_text);
    }

    setTimeout(loop, state.cfg.tick_ms || 150);
  }catch(e){
    setTimeout(loop, 600);
  }
}
loop();
</script>
</body></html>
)HTML";

// -------------------- API --------------------
static void markClient() { last_client_ms = millis(); }

static void handleIndex() {
  markClient();
  server.send(200, "text/html; charset=utf-8", FPSTR(INDEX_HTML));
}

static void handleState() {
  markClient();

  // собираем hits (последние 32, с ts)
  String hitsJson = "[";
  bool first = true;
  uint32_t now = millis();
  for (int i=0;i<MAX_HITS;i++){
    const Hit &h = hits[i];
    if (h.ts == 0) continue;
    if ((now - h.ts) > 8000) continue;
    if (!first) hitsJson += ",";
    first = false;
    hitsJson += "{";
    hitsJson += "\"angle_deg\":" + String(h.angle_deg,1) + ",";
    hitsJson += "\"range_cm\":" + String(h.range_cm,1) + ",";
    hitsJson += "\"v_kmh\":" + String(h.v_kmh,1) + ",";
    hitsJson += "\"dir\":" + String(h.dir) + ",";
    hitsJson += "\"ts\":" + String((uint32_t)h.ts);
    hitsJson += "}";
  }
  hitsJson += "]";

  int usL = cfg.center_us - cfg.left_span_us;
  int usR = cfg.center_us + cfg.right_span_us;

  String json = "{";
  json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"scan_on\":" + String(servo_scan_on ? 1:0) + ",";
  json += "\"pwm_on\":" + String(servo_pwm_on ? 1:0) + ",";
  json += "\"radar_on\":" + String(radar_on ? 1:0) + ",";
  json += "\"radar_powered\":" + String(radar_powered ? 1:0) + ",";
  json += "\"servo_us\":" + String(servo_us_cmd) + ",";
  json += "\"servo_dir\":\"" + String(servo_dir) + "\",";
  json += "\"servo_angle_deg\":" + String(servoAngleDeg(),1) + ",";
  json += "\"motion\":" + String(motion_now ? 1:0) + ",";
  if (bat_pct >= 0) {
    json += "\"bat_v\":" + String(bat_v,2) + ",";
    json += "\"bat_pct\":" + String(bat_pct) + ",";
  } else {
    json += "\"bat_v\":0,\"bat_pct\":-1,";
  }
  json += "\"min_us\":" + String(usL) + ",";
  json += "\"max_us\":" + String(usR) + ",";
  json += "\"hits\":" + hitsJson + ",";
  json += "\"speak_seq\":" + String(speak_seq) + ",";
  json += "\"speak_text\":\"" + jsonEscape(speak_text) + "\",";
  json += "\"cfg\":{";
  json += "\"center_us\":" + String(cfg.center_us) + ",";
  json += "\"left_span_us\":" + String(cfg.left_span_us) + ",";
  json += "\"right_span_us\":" + String(cfg.right_span_us) + ",";
  json += "\"bias_right_us\":" + String(cfg.bias_right_us) + ",";
  json += "\"bias_left_us\":" + String(cfg.bias_left_us) + ",";
  json += "\"scan_period_ms\":" + String(cfg.scan_period_ms) + ",";
  json += "\"tick_ms\":" + String(cfg.tick_ms) + ",";
  json += "\"ease_pct\":" + String(cfg.ease_pct);
  json += "}";
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

static void handleSet() {
  markClient();
  if (server.hasArg("center")) cfg.center_us = clampi(server.arg("center").toInt(), 1200, 1800);
  if (server.hasArg("lspan"))  cfg.left_span_us = clampi(server.arg("lspan").toInt(), 10, 500);
  if (server.hasArg("rspan"))  cfg.right_span_us = clampi(server.arg("rspan").toInt(), 10, 500);
  if (server.hasArg("br"))     cfg.bias_right_us = clampi(server.arg("br").toInt(), -120, 120);
  if (server.hasArg("bl"))     cfg.bias_left_us = clampi(server.arg("bl").toInt(), -120, 120);
  if (server.hasArg("period")) cfg.scan_period_ms = clampi(server.arg("period").toInt(), 800, 60000);
  if (server.hasArg("tick"))   cfg.tick_ms = clampi(server.arg("tick").toInt(), 80, 1200);
  if (server.hasArg("ease"))   cfg.ease_pct = clampi(server.arg("ease").toInt(), 0, 100);

  server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
}

static void handleCmd() {
  markClient();

  if (server.hasArg("scan")) {
    int v = server.arg("scan").toInt();
    if (v == 1) {
      // старт сканирования: включаем питание (если есть ключ) + PWM
      servoPower(true);
      servoPWM(true);
      servo_scan_on = true;

      // старт из центра и вправо
      scan_start_ms = millis() - (uint32_t)clampi(cfg.scan_period_ms,800,60000) / 4;
      speak_seq++; speak_text = "Сканирование запущено";
    } else {
      servo_scan_on = false;
      if (!cfg.servo_hold_when_stop) servoPWM(false);
      speak_seq++; speak_text = "Сканирование остановлено";
    }
  }

  if (server.hasArg("pwm")) {
    int v = server.arg("pwm").toInt();
    if (v == 1) { servoPower(true); servoPWM(true); }
    else { servoPWM(false); }
  }

  if (server.hasArg("radar")) {
    int v = server.arg("radar").toInt();
    if (v == 1) {
      radar_on = true;
      radarPower(true);
      said_clear = false;
      speak_seq++; speak_text = "Радар включен";
    } else {
      radar_on = false;
      radarPower(false);
      speak_seq++; speak_text = "Радар выключен";
    }
  }

  if (server.hasArg("center")) {
    // ручной центр без сканирования
    servo_scan_on = false;
    servoPower(true);
    servoPWM(true);
    servo_us_cmd = cfg.center_us;
    speak_seq++; speak_text = "Центр";
  }

  if (server.hasArg("test")) {
    String d = server.arg("test");
    servo_scan_on = false;
    servoPower(true);
    servoPWM(true);
    if (d == "L") servo_us_cmd = cfg.center_us - cfg.left_span_us;
    if (d == "R") servo_us_cmd = cfg.center_us + cfg.right_span_us;
  }

  server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
}

static void cfgLoad() {
  prefs.begin("rls", true);
  cfg.center_us = prefs.getInt("center", cfg.center_us);
  cfg.left_span_us = prefs.getInt("lspan", cfg.left_span_us);
  cfg.right_span_us = prefs.getInt("rspan", cfg.right_span_us);
  cfg.bias_right_us = prefs.getInt("br", cfg.bias_right_us);
  cfg.bias_left_us = prefs.getInt("bl", cfg.bias_left_us);
  cfg.scan_period_ms = prefs.getInt("period", cfg.scan_period_ms);
  cfg.tick_ms = prefs.getInt("tick", cfg.tick_ms);
  cfg.ease_pct = prefs.getInt("ease", cfg.ease_pct);
  prefs.end();
}

static void cfgSave() {
  prefs.begin("rls", false);
  prefs.putInt("center", cfg.center_us);
  prefs.putInt("lspan", cfg.left_span_us);
  prefs.putInt("rspan", cfg.right_span_us);
  prefs.putInt("br", cfg.bias_right_us);
  prefs.putInt("bl", cfg.bias_left_us);
  prefs.putInt("period", cfg.scan_period_ms);
  prefs.putInt("tick", cfg.tick_ms);
  prefs.putInt("ease", cfg.ease_pct);
  prefs.end();
}

static void handleSave() {
  markClient();
  cfgSave();
  server.send(200, "application/json; charset=utf-8", "{\"ok\":1}");
}

// OTA (опционально, удобно с телефона)
static void handleUpdatePage() {
  markClient();
  String html = "<html><body style='background:#000;color:#8fffa8;font-family:system-ui'>"
                "<h2>OTA Update</h2>"
                "<p>Загружай файл <b>Esp32c6-radar.ino.bin</b> (НЕ merged.bin).</p>"
                "<form method='POST' action='/update' enctype='multipart/form-data'>"
                "<input type='file' name='update' accept='.bin'>"
                "<input type='submit' value='Upload'>"
                "</form></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

static void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      server.send(200, "text/plain", "OK. Rebooting...");
      delay(300);
      ESP.restart();
    } else {
      server.send(500, "text/plain", "Update failed");
    }
  }
}

// -------------------- SETUP/LOOP --------------------
void setup() {
  // Серво на старте НЕ должно крутиться
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  servoPWM(false);
  servoPower(false);

  // Радар по умолчанию выключен
  radar_on = cfg.radar_enabled_default;
  radarPower(false);

  cfgLoad();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);

  server.on("/", handleIndex);
  server.on("/api/state", handleState);
  server.on("/api/set", handleSet);
  server.on("/api/cmd", handleCmd);
  server.on("/api/save", handleSave);

  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/update", HTTP_POST, [](){}, handleUpdateUpload);

  server.begin();

  last_client_ms = millis();
  last_motion_ms = millis();
}

void loop() {
  server.handleClient();

  // 1) сервис сканирования (не накапливает ошибки)
  servoScanService();

  // 2) сервис импульсов (строго 50Hz, когда PWM включен)
  servoService50Hz();

  // 3) радар
  radarService();

  // 4) батарея
  batteryService();

  // 5) failsafe
  failsafeService();
}
