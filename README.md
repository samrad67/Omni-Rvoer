// Motor 1 (L298N #1)
#define M1_IN1 4
#define M1_IN2 5
#define M1_ENA 6

// Motor 2 (L298N #1)
#define M2_IN1 7
#define M2_IN2 8
#define M2_ENB 9

// Motor 3 (L298N #2)
#define M3_IN1 10
#define M3_IN2 11
#define M3_ENA 12

// Motor 4 (L298N #2)
#define M4_IN1 13
#define M4_IN2 14
#define M4_ENB 15

// URM09 Analog Pins
#define FRONT_SIG A0
#define REAR_SIG  A1

// URM09 analog: 0.3V–2.5V → 10–80 cm
const float analog_min_v = 0.3;
const float analog_max_v = 2.5;
const float dist_min_cm = 10;
const float dist_max_cm = 80;

const int analog_res = 1023;
const float ref_voltage = 5.0;

// Threshold distance
const float trigger_cm = 15;

void setup() {
  Serial.begin(9600);

  // Motor pins
  int pins[] = {
    M1_IN1, M1_IN2, M1_ENA,
    M2_IN1, M2_IN2, M2_ENB,
    M3_IN1, M3_IN2, M3_ENA,
    M4_IN1, M4_IN2, M4_ENB
  };
  for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);

  // Sensor pins
  pinMode(FRONT_SIG, INPUT);
  pinMode(REAR_SIG, INPUT);

  Serial.println("Front(cm)\tRear(cm)");
}

void loop() {
  float front_cm = readURM09Analog(FRONT_SIG);
  float rear_cm  = readURM09Analog(REAR_SIG);

  // Serial Plotter output
  Serial.print(front_cm);
  Serial.print("\t");
  Serial.println(rear_cm);

  if (front_cm > 0 && front_cm < trigger_cm) {
    moveBackward();
  } else if (rear_cm > 0 && rear_cm < trigger_cm) {
    moveForward();
  } else {
    stopMotors();
  }

  delay(100);
}

// === Read analog voltage and convert to cm ===
float readURM09Analog(int pin) {
  int raw = analogRead(pin);
  float volts = raw * ref_voltage / analog_res;

  if (volts < analog_min_v || volts > analog_max_v) return -1;

  // Map voltage to distance range
  return mapFloat(volts, analog_min_v, analog_max_v, dist_min_cm, dist_max_cm);
}

// === Float map like Arduino's map() ===
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// === Motor control ===
void moveForward() {
  setMotor(M1_IN1, M1_IN2, M1_ENA, true);
  setMotor(M2_IN1, M2_IN2, M2_ENB, true);
  setMotor(M3_IN1, M3_IN2, M3_ENA, true);
  setMotor(M4_IN1, M4_IN2, M4_ENB, true);
}

void moveBackward() {
  setMotor(M1_IN1, M1_IN2, M1_ENA, false);
  setMotor(M2_IN1, M2_IN2, M2_ENB, false);
  setMotor(M3_IN1, M3_IN2, M3_ENA, false);
  setMotor(M4_IN1, M4_IN2, M4_ENB, false);
}

void stopMotors() {
  analogWrite(M1_ENA, 0);
  analogWrite(M2_ENB, 0);
  analogWrite(M3_ENA, 0);
  analogWrite(M4_ENB, 0);
}

void setMotor(int in1, int in2, int en, bool forward) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(en, 255);
}
