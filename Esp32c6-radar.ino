#define SERVO_PIN 18

// Настройки диапазона (сузь, если всё ещё натягивает провода)
const int CENTER = 1500;     // центр
const int LEFT   = 1350;     // левее (уменьши к 1400 если нужно меньше)
const int RIGHT  = 1650;     // правее (уменьши к 1600 если нужно меньше)

// Скорость
const int STEP_US = 1;       // 1 = очень медленно, 2..5 быстрее
const int HOLD_MS = 400;     // пауза на краях

int cur = CENTER;
int target = RIGHT;

void pulse(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
}

unsigned long lastEdge = 0;

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
}

void loop() {
  // Всегда держим 50 Гц: 20 мс период
  pulse(cur);
  delay(20);

  // Медленно двигаем к цели
  if (cur < target) cur += STEP_US;
  else if (cur > target) cur -= STEP_US;

  // Если дошли до края — держим и меняем направление
  if (cur == target) {
    if (millis() - lastEdge > (unsigned long)HOLD_MS) {
      lastEdge = millis();
      target = (target == RIGHT) ? LEFT : RIGHT;
    }
  }
}
