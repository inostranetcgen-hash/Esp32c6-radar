#define SERVO_PIN 18

void pulse(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
}

void loop() {
  pulse(1500);     // центр / стоп для continuous
  delay(20);       // 50 Гц
}
