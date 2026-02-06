#define SERVO_PIN 18

// === НАСТРОЙ ДИАПАЗОНА ===
// Центр почти всегда 1500 мкс.
// Поставь безопасно узко, потом расширишь.
int minPulse = 1450;  // левый предел (меньше -> больше поворот)
int maxPulse = 1550;  // правый предел (больше -> больше поворот)

// === СКОРОСТЬ ===
int stepUs = 1;       // 1 = медленно, 2..5 быстрее

int cur = 1500;
int dir = +1;

void pulse(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
}

void loop() {
  // 50 Гц — всегда выдаём импульс раз в 20 мс
  pulse(cur);
  delay(20);

  // постоянно изменяем позицию в пределах диапазона
  cur += dir * stepUs;

  if (cur >= maxPulse) { cur = maxPulse; dir = -1; }
  if (cur <= minPulse) { cur = minPulse; dir = +1; }
}
