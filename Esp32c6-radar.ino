#define SERVO_PIN 18

int CENTER_US = 1498.8;   // стартовое, будем подбирать потом
int SPAN_US   = 60;     // начни с 60, потом вернём 70
int STEP_US   = 2;      // 2..4

inline int minPulse() { return CENTER_US - SPAN_US; }
inline int maxPulse() { return CENTER_US + SPAN_US; }

int cur;
int dir = +1;
int turnCount = 0;

void pulse50(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(20000 - us);
}

void hold(int us, int frames) {  // frames * 20ms
  for (int i = 0; i < frames; i++) pulse50(us);
}

void moveSmooth(int from, int to) {
  int step = (from < to) ? STEP_US : -STEP_US;
  int v = from;
  while (v != to) {
    pulse50(v);
    int next = v + step;
    if ((step > 0 && next > to) || (step < 0 && next < to)) next = to;
    v = next;
  }
}

void recenterSoft() {
  // мягко вернуться в центр и чуть подержать, чтобы сбросить накопление
  moveSmooth(cur, CENTER_US);
  hold(CENTER_US, 6);   // 6 кадров = 120мс (почти не видно как пауза)
  cur = CENTER_US;
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  cur = CENTER_US;
  hold(CENTER_US, 10);  // стабилизация старта
}

void loop() {
  pulse50(cur);

  cur += dir * STEP_US;

  if (cur >= maxPulse()) {
    cur = maxPulse();
    dir = -1;
    turnCount++;
    recenterSoft();     // ключ: сброс дрейфа на каждом развороте
  } 
  else if (cur <= minPulse()) {
    cur = minPulse();
    dir = +1;
    turnCount++;
    recenterSoft();     // ключ: сброс дрейфа на каждом развороте
  }
}
