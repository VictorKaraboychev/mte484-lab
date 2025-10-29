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

// Transfer function history arrays
static float error_history[CONTROL_POLES] = {0.0f};
static float voltage_history[CONTROL_POLES + 1] = {0.0f};

// ================== Function Declarations ==================
float getMotorAngle();
float frictionOffset(float voltage);
void control(float target);
float square(float period, float max = 1, float min = 0);

// ================== Setup ==================
void setup() {
  pinMode(A5, OUTPUT);
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();
  set_control_interval_ms(SAMPLING_TIME_MS);
  setMotorVoltage(0.0f);
  
  // Initialize transfer function history arrays
  for (int i = 0; i < CONTROL_POLES; i++) {
    error_history[i] = 0.0f;
    voltage_history[i] = 0.0f;
  }
  voltage_history[CONTROL_POLES] = 0.0f;
}

// ================== Main Loop ==================
void loop() {
  float target = square(0.5f, 0.7f, -0.7f);

  control(target);

  delay(2);
}

// ================== Control Functions ==================
float getMotorAngle() {
  int motor = analogRead(MOT_PIN);
  return MOTOR_ENCODER_M * motor + MOTOR_ENCODER_OFFSET;
}

float offset(float value, float offset) {
  return (value / fabs(value)) * fmax(fabs(value), offset);
}

void control(float target) {
  float error = constrain(target, MIN_ANGLE, MAX_ANGLE) - getMotorAngle();

  // Compute voltage using transfer function difference equation
  // y[k] = (b[0]*u[k] + b[1]*u[k-1] + ... - a[1]*y[k-1] - a[2]*y[k-2] - ...) / a[0]
  
  float voltage = 0.0f;
  
  // Numerator (feedforward) terms: b[0]*u[k] + b[1]*u[k-1] + ... + b[n-1]*u[k-n+1]
  for (int i = 0; i < CONTROL_POLES; i++) {
    if (i == 0) {
      voltage += CONTROL_NUMERATOR[i] * error;
    } else {
      voltage += CONTROL_NUMERATOR[i] * error_history[i - 1];
    }
  }
  
  // Denominator (feedback) terms: -a[1]*y[k-1] - a[2]*y[k-2] - ... - a[n]*y[k-n]
  // Skip a[0] as it's the leading coefficient (used for normalization)
  for (int i = 1; i <= CONTROL_POLES; i++) {
    voltage -= CONTROL_DENOMINATOR[i] * voltage_history[i - 1];
  }
  
  // Normalize by leading denominator coefficient (typically 1.0)
  if (CONTROL_DENOMINATOR[0] != 0.0f) {
    voltage /= CONTROL_DENOMINATOR[0];
  }
  
  // Update history arrays (shift right, insert new values at index 0)
  // Shift error history: u[k-n+2] -> u[k-n+1], ..., u[k-1] -> u[k-2], u[k] -> u[k-1]
  for (int i = CONTROL_POLES - 1; i > 0; i--) {
    error_history[i] = error_history[i - 1];
  }
  error_history[0] = error;
  
  // Shift voltage history: y[k-n] -> y[k-n-1], ..., y[k-1] -> y[k-2]
  for (int i = CONTROL_POLES; i > 0; i--) {
    voltage_history[i] = voltage_history[i - 1];
  }
  // voltage_history[0] will be updated after saturation and before applying to motor

  voltage = constrain(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
  
  // Update voltage history with the constrained value
  voltage_history[0] = voltage;

  setMotorVoltage(offset(voltage, MOTOR_VOLTAGE_OFFSET));
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
