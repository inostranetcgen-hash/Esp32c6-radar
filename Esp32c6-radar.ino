// ===== НАСТРОЙКИ =====
#define SERVO_PIN 18
#define SERVO_CHANNEL 0
#define SERVO_FREQ 50       // 50 Гц для сервопривода
#define SERVO_RESOLUTION 16 // 16 бит PWM

uint32_t dutyMin = 3277;   // ~0.5 ms
uint32_t dutyMax = 6554;   // ~2.5 ms

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-C6 Radar boot OK");

  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
}

// ===== LOOP =====
void loop() {
  // от 0 до 180 градусов
  for (uint32_t duty = dutyMin; duty <= dutyMax; duty += 50) {
    ledcWrite(SERVO_CHANNEL, duty);
    delay(20);
  }

  delay(500);

  // обратно
  for (uint32_t duty = dutyMax; duty >= dutyMin; duty -= 50) {
    ledcWrite(SERVO_CHANNEL, duty);
    delay(20);
  }

  delay(500);
}
