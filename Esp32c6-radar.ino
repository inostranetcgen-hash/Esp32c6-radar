void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-C6 Radar boot OK");
}

void loop() {
  delay(1000);
  Serial.println("Radar running...");
}
