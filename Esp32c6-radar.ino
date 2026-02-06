#define SERVO_PIN 18

// 1) ПОДСТРОЙКА ЦЕНТРА (главное против перекоса)
// Было 1500. Если вправо меньше — чаще нужно чуть УВЕЛИЧИТЬ центр: 1510..1540
int CENTER_US = 1480;

// 2) ДИАПАЗОН "в каждую сторону"
int SPAN_US = 45;   // 60 => 1440..1560, 50 => 1450..1550, 30 => 1470..1530

// 3) СКОРОСТЬ
int STEP_US = 1;    // 1 медленнее, 2..4 быстрее

int cur;
int dir = +1;

inline int minPulse() { return CENTER_US - SPAN_US; }
inline int maxPulse() { return CENTER_US + SPAN_US; }

void pulse(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  cur = CENTER_US;
}

void loop() {
  // всегда 50 Гц
  pulse(cur);
  delay(20);

  // шаг
  cur += dir * STEP_US;

  // разворот БЕЗ "залипания" на краях
  if (cur > maxPulse()) { cur = maxPulse(); dir = -1; }
  else if (cur < minPulse()) { cur = minPulse(); dir = +1; }
}
