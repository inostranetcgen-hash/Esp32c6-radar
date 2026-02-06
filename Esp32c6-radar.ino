#include <Arduino.h>

#define SERVO_PIN 18

// Для сервопривода нам нужно 50 Гц.
// В новом core (3.x) ledcSetup/ledcAttachPin могут отсутствовать.
// Используем новый API: ledcAttach + ledcWrite / ledcWriteTone.

static const int SERVO_FREQ = 50;         // 50 Hz
static const int SERVO_RES_BITS = 16;     // 16-bit
static const int SERVO_CHANNEL = 0;

// 16 бит: 0..65535. Период 20ms.
// 0.5ms = 2.5% => 65535*0.025 ≈ 1638
// 2.5ms = 12.5% => 65535*0.125 ≈ 8192
static const uint32_t dutyMin = 1638;
static const uint32_t dutyMax = 8192;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32-C6 Servo test (new LEDC)");

  // Новый вызов: ledcAttach(pin, freq, resolution)
  // Если на твоём core сигнатура другая, лог скажет — я подстрою.
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES_BITS);

  // Некоторые сборки используют канал неявно.
  // Если потребуется канал — будет видно по ошибке.
}

void loop() {
  // sweep вперед
  for (uint32_t duty = dutyMin; duty <= dutyMax; duty += 60) {
    ledcWrite(SERVO_PIN, duty);  // в новом API иногда пишут по pin
    delay(20);
  }
  delay(300);

  // sweep назад
  for (uint32_t duty = dutyMax; duty >= dutyMin; duty -= 60) {
    ledcWrite(SERVO_PIN, duty);
    delay(20);
    if (duty < dutyMin + 60) break;
  }
  delay(300);
}
