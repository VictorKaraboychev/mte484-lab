#include <Arduino.h>
#include <geeWhiz.h>
#include <math.h>

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

#define MOTOR_VOLTAGE_OFFSET_UP 0.1f
#define MOTOR_VOLTAGE_OFFSET_DOWN -0.4f

#define SAMPLING_TIME_MS 15

#define MAX_ANGLE 0.7f
#define MIN_ANGLE -0.7f

#define MIN_VOLTAGE -6.0f
#define MAX_VOLTAGE 6.0f

#define CONTROL_POLES 6
const float CONTROL_NUMERATOR[CONTROL_POLES] = {-2.553,7.607134211501692,-9.306292203923523,5.549959653328761,-1.4398647156921363,0.0863967655378138};
const float CONTROL_DENOMINATOR[CONTROL_POLES + 1] = {1,-3.0378212871871377,3.885912478932794,-2.6750909423578797,1.0484404482158423,-0.22543761513395377,0.019081635608925465};

// Transfer function history arrays
static float error_history[CONTROL_POLES] = {0.0f};
static float voltage_history[CONTROL_POLES + 1] = {0.0f};

// ================== Function Declarations ==================
float getMotorAngle();
float getBallPosition();
float offset(float value, float offset);
float control(float target);
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
  float target = PI / 4.0f; //square(3.0f, 0.7f, -0.7f);

  float voltage = control(target);

  setMotorVoltage(voltage);

  delay(SAMPLING_TIME_MS);
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
    return fmax(value, offset_up);
  } else {
    return fmin(value, offset_down);
  }
}

float control(float target) {
  float y = getMotorAngle();  // Output (motor angle)
  float r = constrain(target, MIN_ANGLE, MAX_ANGLE);  // Input (reference/target)
  float e = r - y;  // Error

  // Update error history first (shift right, then insert new error at index 0)
  for (int i = CONTROL_POLES - 1; i > 0; i--) {
    error_history[i] = error_history[i - 1];
  }
  error_history[0] = e;

  // Compute voltage using transfer function difference equation
  // y[k] = (b[0]*u[k] + b[1]*u[k-1] + ... - a[1]*y[k-1] - a[2]*y[k-2] - ...) / a[0]
  
  float u = 0.0f;  // Control voltage (before offset)
  
  // Numerator (feedforward) terms: b[0]*u[k] + b[1]*u[k-1] + ... + b[n-1]*u[k-n+1]
  for (int i = 0; i < CONTROL_POLES; i++) {
    u += CONTROL_NUMERATOR[i] * error_history[i];
  }
  
  // Denominator (feedback) terms: -a[1]*y[k-1] - a[2]*y[k-2] - ... - a[n]*y[k-n]
  // Skip a[0] as it's the leading coefficient (used for normalization)
  for (int i = 0; i < CONTROL_POLES; i++) {
    u -= CONTROL_DENOMINATOR[i + 1] * voltage_history[i];
  }
  
  // Normalize by leading denominator coefficient (typically 1.0)
  if (CONTROL_DENOMINATOR[0] != 0.0f) {
    u /= CONTROL_DENOMINATOR[0];
  }

  u = constrain(u, MIN_VOLTAGE, MAX_VOLTAGE);
  
  // Update voltage history (shift right, then insert new voltage at index 0)
  for (int i = CONTROL_POLES; i > 0; i--) {
    voltage_history[i] = voltage_history[i - 1];
  }
  voltage_history[0] = u;

  // Output CSV: (time[seconds], y, u, r, e)
  float time = millis() / 1000.0f;
  // Serial.print(time, 4);
  // Serial.print(",");
  // Serial.print(y, 4);
  // Serial.print(",");
  // Serial.print(u, 4);
  // Serial.print(",");
  // Serial.print(r, 4);
  // Serial.print(",");
  // Serial.println(e, 4);

  Serial.print(time, 4);
  Serial.print(",");
  Serial.print(getBallPosition(), 4);
  Serial.print(",");
  Serial.println(getMotorAngle(), 4);

  float voltage = offset(u, MOTOR_VOLTAGE_OFFSET_UP, MOTOR_VOLTAGE_OFFSET_DOWN);
  
  return voltage;
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
  motor_angle_raw = analogRead(MOT_PIN);
  ball_position_raw = analogRead(BAL_PIN);
  
  digitalWrite(A5, HIGH);
  digitalWrite(A5, LOW);
}
