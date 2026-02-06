#define SERVO_PIN 18
#define SERVO_CHANNEL 0
#define SERVO_FREQ 50
#define SERVO_RESOLUTION 16

static const uint32_t dutyMin = 3277; // ~0.5ms
static const uint32_t dutyMax = 6554; // ~2.5ms

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-C6 servo test (LEDC)");

  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
}

void loop() {
  for (uint32_t duty = dutyMin; duty <= dutyMax; duty += 40) {
    ledcWrite(SERVO_CHANNEL, duty);
    delay(20);
  }
  delay(300);

  for (uint32_t duty = dutyMax; duty >= dutyMin; duty -= 40) {
    ledcWrite(SERVO_CHANNEL, duty);
    delay(20);
    if (duty < dutyMin + 40) break;
  }
  delay(300);
}
