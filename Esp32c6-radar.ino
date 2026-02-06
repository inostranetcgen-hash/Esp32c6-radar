#define SERVO_PIN 18

int CENTER_US = 1497;
int SPAN_US   = 45;

int STEP_US   = 1;     // скорость
int cur;
int dir = +1;

inline int minPulse() { return CENTER_US - SPAN_US; }
inline int maxPulse() { return CENTER_US + SPAN_US; }

void pulse_50hz(int us) {
  // Формируем строго 20ms период: HIGH us, затем LOW (20000-us)
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(20000 - us);
}

void goCenter() {
  // “сброс” в центр: 0.3 сек удерживаем центр
  for (int i = 0; i < 15; i++) pulse_50hz(CENTER_US);
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  cur = CENTER_US;
  goCenter();
}

void loop() {
  pulse_50hz(cur);

  cur += dir * STEP_US;

  if (cur >= maxPulse()) {
    cur = maxPulse();
    dir = -1;
    goCenter();          // <-- ключ: сбрасываем ошибку
  } 
  else if (cur <= minPulse()) {
    cur = minPulse();
    dir = +1;
    goCenter();          // <-- ключ: сбрасываем ошибку
  }
}
