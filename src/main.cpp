#include <Arduino.h>
#include <geeWhiz.h>
#include <math.h>

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// ================== Configuration ==================
#define MOTOR_ENCODER_M -0.01377891515
#define MOTOR_ENCODER_OFFSET 7.05480455543
#define MOTOR_VOLTAGE_OFFSET 0.2f

#define SAMPLING_TIME_MS 2

#define MAX_ANGLE PI / 4.0f
#define MIN_ANGLE -PI / 4.0f

#define MIN_VOLTAGE -6.0f
#define MAX_VOLTAGE 6.0f

#define CONTROL_POLES 14
const float CONTROL_NUMERATOR[CONTROL_POLES] = {};
const float CONTROL_DENOMINATOR[CONTROL_POLES + 1] = {};

// ================== Function Declarations ==================
float getMotorAngle();
float frictionOffset(float voltage);
void control(float target, float kp);
float square(float period, float max = 1, float min = 0);

// ================== Setup ==================
void setup() {
  pinMode(A5, OUTPUT);
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();
  set_control_interval_ms(SAMPLING_TIME_MS);
  setMotorVoltage(0.0f);
}

// ================== Main Loop ==================
void loop() {
  float target = square(1.0f, 0.7f, -0.7f);
  
  Serial.println(target);

  delay(2);
}

// ================== Control Functions ==================
float getMotorAngle() {
  int motor = analogRead(MOT_PIN);
  return MOTOR_ENCODER_M * motor + MOTOR_ENCODER_OFFSET;
}

float frictionOffset(float voltage) {
  return (voltage / fabs(voltage)) * fmax(fabs(voltage), MOTOR_VOLTAGE_OFFSET);
}

void control(float target) {
  float error = constrain(target, MIN_ANGLE, MAX_ANGLE) - getMotorAngle();

  float voltage = error;

  voltage = constrain(voltage, MIN_VOLTAGE, MAX_VOLTAGE);

  setMotorVoltage(frictionOffset(voltage));
}

// ================== Square Wave Generator ==================
float square(float period, float max, float min) {
  float current_time = millis() / 1000.0f;  // Convert to seconds
  float position_in_period = fmod(current_time, period);
  
  // First half of period: return max, second half: return min
  if (position_in_period < period / 2.0f) {
    return max;
  } else {
    return min;
  }
}

// ================== Control ISR ==================
void interval_control_code(void) {
  float seconds = millis() / 1000.0f;
  int motor = analogRead(MOT_PIN);
  int ball = analogRead(BAL_PIN);
  float angle = getMotorAngle();
  
  digitalWrite(A5, HIGH);
  
  digitalWrite(A5, LOW);
}
