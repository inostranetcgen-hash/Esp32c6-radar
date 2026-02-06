#define SERVO_PIN 18

// БАЗА
int CENTER_US = 1499;

// Пределы отдельно (если уезжает влево — делаем вправо ЧУТЬ больше или влево ЧУТЬ меньше)
int LEFT_SPAN_US  = 60;   // левый предел = CENTER - LEFT_SPAN_US
int RIGHT_SPAN_US = 65;   // правый предел = CENTER + RIGHT_SPAN_US

// Плавность/скорость (важно: серво должно успевать)
int STEP_US = 1;          // шаг
int MOVE_EVERY_FRAMES = 2; // двигаем позицию раз в N кадров (2 = медленнее и плавнее), но импульсы идут каждый кадр

// Компенсация люфта по направлению (подбирается)
// Если диапазон уезжает ВЛЕВО -> обычно помогает чуть "подтолкнуть" вправо при движении вправо
int BIAS_WHEN_MOVING_RIGHT = +3; // +2..+6
int BIAS_WHEN_MOVING_LEFT  = 0;  // обычно 0..+2

int cur = 1499;
int dir = +1;   // +1 вправо, -1 влево
int frame = 0;

inline int minPulse() { return CENTER_US - LEFT_SPAN_US; }
inline int maxPulse() { return CENTER_US + RIGHT_SPAN_US; }

void pulse50(int us) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(20000 - us); // ровно 20мс период
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  cur = CENTER_US;

  // стабилизация на старте
  for (int i = 0; i < 20; i++) pulse50(CENTER_US);
}

void loop() {
  // Команда с компенсацией люфта
  int cmd = cur + (dir > 0 ? BIAS_WHEN_MOVING_RIGHT : BIAS_WHEN_MOVING_LEFT);

  // защитим от выхода за пределы
  if (cmd < minPulse()) cmd = minPulse();
  if (cmd > maxPulse()) cmd = maxPulse();

  // подаём импульс каждый кадр
  pulse50(cmd);

  // а позицию меняем реже -> серво успевает, движение выглядит плавнее
  frame++;
  if (frame % MOVE_EVERY_FRAMES != 0) return;

  cur += dir * STEP_US;

  if (cur >= maxPulse()) { cur = maxPulse(); dir = -1; }
  if (cur <= minPulse()) { cur = minPulse(); dir = +1; }
}
