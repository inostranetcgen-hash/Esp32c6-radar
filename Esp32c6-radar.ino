#define SERVO_PIN 18
int CENTER_US = 1498;

void pulse_50hz(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(20000 - us);
}

void setup() { pinMode(SERVO_PIN, OUTPUT); }

void loop() {
  pulse_50hz(CENTER_US);   // всё время один и тот же импульс
}
