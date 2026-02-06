#define SERVO_PIN 18

int currentPulse = 1500;   // старт с центра
int minPulse = 1200;       // -90°
int maxPulse = 1800;       // +90°
int step = 5;              // МЕНЬШЕ = МЕДЛЕННЕЕ
int stepDelay = 25;        // МЕНЬШЕ = БЫСТРЕЕ

void pulse(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
  delay(20);
}

void moveTo(int target) {
  while (currentPulse != target) {
    if (currentPulse < target) currentPulse += step;
    if (currentPulse > target) currentPulse -= step;

    pulse(currentPulse);
    delay(stepDelay);
  }
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
}

void loop() {
  moveTo(minPulse);  // влево
  delay(500);

  moveTo(maxPulse);  // вправо
  delay(500);
}
