#define SERVO_PIN 18

int CENTER_US = 1498.5;
int SPAN_US   = 70;

int STEP_US   = 3;   // 2..4 обычно идеально; 1 часто даёт дрейф
int cur;
int dir = +1;

inline int minPulse() { return CENTER_US - SPAN_US; }
inline int maxPulse() { return CENTER_US + SPAN_US; }

void pulse_50hz(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(20000 - us);
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  cur = CENTER_US;
  // короткая стабилизация
  for (int i = 0; i < 10; i++) pulse_50hz(cur);
}

void loop() {
  pulse_50hz(cur);

  cur += dir * STEP_US;

  if (cur >= maxPulse()) {
    cur = maxPulse();
    dir = -1;
    // микродемпфер: 2 кадра (~40мс) без заметной паузы
    pulse_50hz(cur);
    pulse_50hz(cur);
  } 
  else if (cur <= minPulse()) {
    cur = minPulse();
    dir = +1;
    pulse_50hz(cur);
    pulse_50hz(cur);
  }
}
