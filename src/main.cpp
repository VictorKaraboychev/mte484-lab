#include <Arduino.h>
#include <geeWhiz.h>
#include <math.h>

#include "transfer_function.h"

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

#define MOTOR_VOLTAGE_OFFSET_UP 0.2f
#define MOTOR_VOLTAGE_OFFSET_DOWN -0.55f


#define MAX_ANGLE 0.7f
#define MIN_ANGLE -0.7f

#define MIN_VOLTAGE -6.0f
#define MAX_VOLTAGE 6.0f

#define D1_POLES 1
#define D1_SAMPLING_TIME_MS 15
const float D1_NUMERATOR[D1_POLES] = {-5.0f};
const float D1_DENOMINATOR[D1_POLES + 1] = {1.0f, 0.0f};

TransferFunction d1(D1_NUMERATOR, D1_DENOMINATOR, D1_POLES, D1_SAMPLING_TIME_MS);

#define D2_POLES 6
#define D2_SAMPLING_TIME_MS 15
const float D2_NUMERATOR[D2_POLES] = {-3.077769438503281,10.138128350102296,-14.855920046134203,11.987988761412678,-5.124585949359640,0.889457904200626};
const float D2_DENOMINATOR[D2_POLES + 1] = {1.0,-2.953909355670471,3.663745867407955,-2.446303177829365,0.934801603263164,-0.216826988359841,0.032790888211790};

TransferFunction d2(D2_NUMERATOR, D2_DENOMINATOR, D2_POLES, D2_SAMPLING_TIME_MS);

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

  uint16_t min_sampling_time_ms = min(D1_SAMPLING_TIME_MS, D2_SAMPLING_TIME_MS);
  set_control_interval_ms(min_sampling_time_ms);
  
  setMotorVoltage(0.0f);
}

// ================== Main Loop ==================
void loop() {
  float r1 = square(15.0f, 0.25f, 0.1f);

  // Error (reference ball position - output ball position)
  float e1 = r1 - getBallPosition();

  // Compute the angle using the transfer function
  float u1 = d1.compute(e1);

  // Constrain the target angle
  float r2 = constrain(u1, MIN_ANGLE, MAX_ANGLE);

  // Error (reference motor angle - output motor angle)
  float e2 = r2 - getMotorAngle();

  // Compute the voltage using the transfer function
  float u2 = d2.compute(e2);

  // Offset and constrain the voltage
  u2 = offset(u2, MOTOR_VOLTAGE_OFFSET_UP, MOTOR_VOLTAGE_OFFSET_DOWN);
  u2 = constrain(u2, MIN_VOLTAGE, MAX_VOLTAGE);

  // Set the motor voltage
  setMotorVoltage(u2);
}

// ================== Control Functions ==================
float getMotorAngle() {
  return MOTOR_ENCODER_M * motor_angle_raw + MOTOR_ENCODER_OFFSET;
}

float getBallPosition() {
  return BALL_POSITION_M * ball_position_raw + BALL_POSITION_OFFSET;
}

// If the value is less than the center but greater than the offset_down, return the offset_down.
// If the value is greater than the center but less than the offset_up, return the offset_up.
// Otherwise, return the value.
float offset(float value, float offset_up, float offset_down) {
  float center = (offset_up + offset_down) / 2.0f;
  if (value < center && value > offset_down) return offset_down;
  if (value > center && value < offset_up) return offset_up;
  return value;
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
  motor_angle_raw = analogRead(MOT_PIN);
  ball_position_raw = analogRead(BAL_PIN);
  
  digitalWrite(A5, HIGH);
  digitalWrite(A5, LOW);
}
