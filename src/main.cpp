#include <Arduino.h>
#include <geeWhiz.h>
#include <math.h>

#include "transfer_function.h"
#include "filters.h"
#include "pid.h"

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// ================== Data Variables ==================
volatile int motor_angle_raw;
volatile int ball_position_raw; 

// ================== Configuration ==================
#define MOTOR_ENCODER_M -0.01377891515
#define MOTOR_ENCODER_OFFSET 7.05480455543

#define BALL_POSITION_M 0.001031377
#define BALL_POSITION_OFFSET -0.3197270408

#define MAX_ANGLE 0.7f
#define MIN_ANGLE -0.7f

#define MIN_VOLTAGE -6.0f
#define MAX_VOLTAGE 6.0f

#define SENSOR_SAMPLING_TIME_MS 2

LowPassFilter ball_position_low_pass_filter(
  4.0f, 
  1000.0f / SENSOR_SAMPLING_TIME_MS
);
LowPassFilter motor_angle_low_pass_filter(
  150.0f,
  1000.0f / SENSOR_SAMPLING_TIME_MS
);

PID pid_outer(-1.920872f, -0.2f, -3.117162f, 394, MIN_ANGLE, MAX_ANGLE);
PID pid_inner(-7.892344f, -0.998973, -0.177016f, 15, MIN_VOLTAGE, MAX_VOLTAGE);

// ================== Function Declarations ==================
float getMotorAngle();
float getBallPosition();
float offset(float value, float offset_up, float offset_down);
float square(float period, float max = 1, float min = 0);

// ================== Setup ==================
void setup() {
  pinMode(A5, OUTPUT);
  Serial.begin(115200);

  delay(300);

  geeWhizBegin();

  set_control_interval_ms(2);

  setMotorVoltage(0.0f);
}

// ================== Main Loop ==================
void loop() {
  float t = millis() / 1000.0f;

  float r1 = square(20.0f, 0.25f, 0.1f);
  float y1 = getBallPosition();

  // Error (reference ball position - output ball position)
  float e1 = r1 - y1;

  // Compute the angle using the transfer function (already constrained)
  float u1 = pid_outer.compute(e1, millis());
  u1 = constrain(u1, MIN_ANGLE, MAX_ANGLE);
  float r2 = u1;
  float y2 = getMotorAngle();

  // Error (reference motor angle - output motor angle)
  float e2 = r2 - y2;

  // Compute the voltage using the transfer function (already constrained)
  float u2 = pid_inner.compute(e2, millis());
  u2 = constrain(u2, MIN_VOLTAGE, MAX_VOLTAGE);

  // Set the motor voltage
  setMotorVoltage(u2);

  // Print in CSV format 4 decimal places
  Serial.print(t, 4);
  Serial.print(",");
  Serial.print(r1, 4);
  Serial.print(",");
  Serial.print(y1, 4);
  Serial.print(",");
  Serial.print(e1, 4);
  Serial.print(",");
  Serial.print(u1, 4);
  Serial.print(",");
  Serial.print(r2, 4);
  Serial.print(",");
  Serial.print(y2, 4);
  Serial.print(",");
  Serial.print(e2, 4);
  Serial.print(",");
  Serial.print(u2, 4);
  Serial.println();
}

// ================== Control Functions ==================
float getMotorAngle() {
  return MOTOR_ENCODER_M * motor_angle_raw + MOTOR_ENCODER_OFFSET;
}

float getBallPosition() {
  return BALL_POSITION_M * ball_position_raw + BALL_POSITION_OFFSET;
}

float offset(float value, float offset_up, float offset_down) {
  if (value > 0) {
    return value + offset_up;
  } else  {
    return value + offset_down;
  }
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
  motor_angle_raw = motor_angle_low_pass_filter.process(analogRead(MOT_PIN));
  ball_position_raw = ball_position_low_pass_filter.process(analogRead(BAL_PIN));

  digitalWrite(A5, HIGH);
  digitalWrite(A5, LOW);
}
