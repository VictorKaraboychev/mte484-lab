#include <Arduino.h>
#include <math.h>

#include "geeWhiz.h"

#include "transfer_function.h"
#include "filters.h"

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

#define BALL_ANGLE_OFFSET_UP 0.07f
#define BALL_ANGLE_OFFSET_DOWN -0.03f

#define MOTOR_VOLTAGE_OFFSET_UP_0 -0.1f
#define MOTOR_VOLTAGE_OFFSET_DOWN_0 -0.75f
#define MOTOR_VOLTAGE_OFFSET_UP_40 -0.2f
#define MOTOR_VOLTAGE_OFFSET_DOWN_40 -1.00f

#define MAX_ANGLE 0.7f
#define MIN_ANGLE -0.7f

#define MIN_VOLTAGE -6.0f
#define MAX_VOLTAGE 6.0f

#define D1_ZEROS 2
#define D1_POLES 3
#define D1_SAMPLING_TIME_MS 500
const float D1_NUMERATOR[D1_ZEROS] = {-1.950542,1.950542}; // {-1.967128,1.994930};
const float D1_DENOMINATOR[D1_POLES] = {1.000000,-0.714464,0.313687}; //{1.0,-0.791685,0.439943}; 

TransferFunction d1(D1_NUMERATOR, D1_DENOMINATOR, D1_ZEROS, D1_POLES, D1_SAMPLING_TIME_MS, MIN_ANGLE, MAX_ANGLE);

#define D2_ZEROS 2
#define D2_POLES 3
#define D2_SAMPLING_TIME_MS 15
const float D2_NUMERATOR[D2_ZEROS] = {-1.736621,-0.139124};
const float D2_DENOMINATOR[D2_POLES] = {1.000000,-1.180573,0.609533};

TransferFunction d2(D2_NUMERATOR, D2_DENOMINATOR, D2_ZEROS, D2_POLES, D2_SAMPLING_TIME_MS, MIN_VOLTAGE, MAX_VOLTAGE);

#define SENSOR_SAMPLING_TIME_MS 2

LowPassFilter ball_position_low_pass_filter(
  10.0f, 
  1000.0f / SENSOR_SAMPLING_TIME_MS
);
LowPassFilter motor_angle_low_pass_filter(
  150.0f,
  1000.0f / SENSOR_SAMPLING_TIME_MS
);

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
  float u1 = d1.compute(e1, millis());
  float r2 = u1;
  // r2 = offset(u1, BALL_ANGLE_OFFSET_UP, BALL_ANGLE_OFFSET_DOWN);
  float y2 = getMotorAngle();

  // Error (reference motor angle - output motor angle)
  float e2 = r2 - y2;

  // Compute the voltage using the transfer function (already constrained)
  float u2 = d2.compute(e2, millis());

  // Offset the voltage (constraints already applied in transfer function)

  float offset_ball_effect = y1 / 0.4043f;
  float voltage_offset_up = offset_ball_effect * MOTOR_VOLTAGE_OFFSET_UP_40 + MOTOR_VOLTAGE_OFFSET_UP_0 * (1 - offset_ball_effect);
  float voltage_offset_down = offset_ball_effect * MOTOR_VOLTAGE_OFFSET_DOWN_40 + MOTOR_VOLTAGE_OFFSET_DOWN_0 * (1 - offset_ball_effect);

  u2 = offset(u2, voltage_offset_up, voltage_offset_down);

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
