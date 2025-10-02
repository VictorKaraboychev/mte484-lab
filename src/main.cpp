#include <Arduino.h>
#include <geeWhiz.h>
#include <math.h>

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

#define MOTOR_ENCODER_M -0.01377891515
#define MOTOR_ENCODER_OFFSET 7.05480455543
#define MOTOR_VOLTAGE_OFFSET 0.2f

// ================== Calibration Parameters ==================
const int NUM_KP_TESTS = 8;  // Number of Kp values to test
const int NUM_SETPOINT_TESTS = 10;  // Number of setpoints to test per Kp
const float KP_MIN = -18.0f;
const float KP_MAX = -24.0f;
const float SETPOINT_MIN = PI/4.0f;
const float SETPOINT_MAX = PI/8.0f;

// ================== Test Data Structure ==================
struct TestData {
  float kp;
  float setpoint;
  float percentage_overshoot;
  float peak_time;
  float settling_time;
  float steady_state_error;
  bool valid_test;
  float zeta;
  float omega_n;
  float tau;
  float K1;
};

// ================== Global Variables ==================
TestData test_results[NUM_KP_TESTS * NUM_SETPOINT_TESTS];
int current_test = 0;
int current_setpoint_test = 0;
float current_kp = 0;
float current_setpoint = 0;
bool test_in_progress = false;
bool calibration_complete = false;
unsigned long test_start_time = 0;
unsigned long step_start_time = 0;
const unsigned long TEST_DURATION = 750;
const unsigned long SETTLING_TIME = 250;  

// Response analysis variables
float max_overshoot = 0;
float peak_time_value = 0;
float steady_state_value = 0;
bool peak_found = false;
float initial_setpoint = 0;

// ================== Function Declarations ==================
float getMotorAngle();
float frictionOffset(float voltage);
void control(float target, float kp);
void startNewTest();
void analyzeResponse();
void calculateSystemParameters();
void printResults();
void resetTestVariables();

// ================== Setup ==================
void setup() {
  pinMode(A5, OUTPUT);
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();
  set_control_interval_ms(2);
  setMotorVoltage(0.0f + MOTOR_VOLTAGE_OFFSET);

  Serial.println("=== System Calibration Started ===");
  Serial.println("Testing Kp values from -20 to -100");
  Serial.println("Testing setpoints from 0 to π/4");
  Serial.println("Only final test results will be printed");
  Serial.println("=== Starting Tests ===");
  
  // Start calibration immediately
  delay(1000); // Brief delay to ensure system is ready
  startNewTest();

}

// ================== Main Loop ==================
void loop() {
  if (calibration_complete) {
    if (millis() - test_start_time > 2000) { // Wait 2 seconds after completion
      calculateSystemParameters();
      printResults();
      while(true) { delay(1000); } // Stop execution
    }
    return;
  }

  unsigned long current_time = millis();
  
  if (!test_in_progress) {
    // Settling phase between tests
    if (current_time - step_start_time > SETTLING_TIME) {
      startNewTest();
    }
  } else {
    // Test in progress
    if (current_time - test_start_time > TEST_DURATION) {
      // Test completed, analyze response
      analyzeResponse();
      test_in_progress = false;
      step_start_time = current_time;
      
      // Move to next test
      current_setpoint_test++;
      if (current_setpoint_test >= NUM_SETPOINT_TESTS) {
        current_setpoint_test = 0;
        current_test++;
        if (current_test >= NUM_KP_TESTS) {
          calibration_complete = true;
          test_start_time = current_time;
          return;
        }
      }
    }
  }
  
  // Control the system
  if (test_in_progress) {
    control(current_setpoint, current_kp);
  } else {
    // Reset position to 0 during settling
    control(0, -30); // Use negative gain to drive to zero
  }
}

// ================== Control Functions ==================
float getMotorAngle() {
  int motor = analogRead(MOT_PIN);
  return MOTOR_ENCODER_M * motor + MOTOR_ENCODER_OFFSET;
}

float frictionOffset(float voltage) {
  return (voltage / abs(voltage)) * max(abs(voltage), MOTOR_VOLTAGE_OFFSET);
}

void control(float target, float kp) {
  float error = target - getMotorAngle();
  float voltage = kp * error;
  setMotorVoltage(frictionOffset(voltage));
}

// ================== Test Management ==================
void startNewTest() {
  // Calculate current Kp and setpoint
  float kp_step = (KP_MAX - KP_MIN) / (NUM_KP_TESTS - 1);
  float setpoint_step = (SETPOINT_MAX - SETPOINT_MIN) / (NUM_SETPOINT_TESTS - 1);
  
  current_kp = KP_MIN + current_test * kp_step;
  current_setpoint = SETPOINT_MIN + current_setpoint_test * setpoint_step;
  
  // Reset test variables
  resetTestVariables();
  
  test_in_progress = true;
  test_start_time = millis();
  step_start_time = millis();
  
  Serial.print("Starting test ");
  Serial.print(current_test * NUM_SETPOINT_TESTS + current_setpoint_test + 1);
  Serial.print("/");
  Serial.print(NUM_KP_TESTS * NUM_SETPOINT_TESTS);
  Serial.print(" - Kp: ");
  Serial.print(current_kp, 1);
  Serial.print(", Setpoint: ");
  Serial.print(current_setpoint, 3);
  Serial.println();
}

void resetTestVariables() {
  max_overshoot = 0;
  peak_time_value = 0;
  steady_state_value = 0;
  peak_found = false;
  initial_setpoint = current_setpoint;
}

// ================== Response Analysis ==================
void analyzeResponse() {
  // Calculate percentage overshoot
  float overshoot = max_overshoot - current_setpoint;
  float percentage_overshoot = (overshoot / current_setpoint) * 100.0f;
  
  // Store basic test results
  int result_index = current_test * NUM_SETPOINT_TESTS + current_setpoint_test;
  test_results[result_index].kp = current_kp;
  test_results[result_index].setpoint = current_setpoint;
  test_results[result_index].percentage_overshoot = percentage_overshoot;
  test_results[result_index].peak_time = peak_time_value;
  test_results[result_index].valid_test = (current_setpoint > 0.01f); // Only valid for non-zero setpoints
  
  // Calculate system parameters for this test
  if (test_results[result_index].valid_test && peak_time_value > 0) {
    float po = percentage_overshoot / 100.0f;
    float tp = peak_time_value;
    
    // Calculate damping ratio from percentage overshoot
    float ln_po_over_pi = log(po) / PI;
    float zeta = (-ln_po_over_pi) / sqrt(1 + pow((-ln_po_over_pi), 2));
    
    // Calculate natural frequency from peak time
    float omega_n = PI / (tp * sqrt(1 - zeta*zeta));
    
    // Calculate tau and K1 for this test
    float tau = 1.0f / (2.0f * zeta * omega_n);
    float K1 = (omega_n * omega_n * tau) / current_kp;
    
    // Store calculated parameters
    test_results[result_index].zeta = zeta;
    test_results[result_index].omega_n = omega_n;
    test_results[result_index].tau = tau;
    test_results[result_index].K1 = K1;
    
    Serial.print("Test completed - PO: ");
    Serial.print(percentage_overshoot, 2);
    Serial.print("%, Tp: ");
    Serial.print(peak_time_value, 3);
    Serial.print("s, ζ: ");
    Serial.print(zeta, 4);
    Serial.print(", ωn: ");
    Serial.print(omega_n, 4);
    Serial.print(", τ: ");
    Serial.print(tau, 4);
    Serial.print(", K1: ");
    Serial.println(K1, 4);
  } else {
    // Invalid test - set parameters to zero
    test_results[result_index].zeta = 0;
    test_results[result_index].omega_n = 0;
    test_results[result_index].tau = 0;
    test_results[result_index].K1 = 0;
    
    Serial.print("Test completed - PO: ");
    Serial.print(percentage_overshoot, 2);
    Serial.print("%, Tp: ");
    Serial.print(peak_time_value, 3);
    Serial.println("s (Invalid test)");
  }
}

// ================== Parameter Estimation ==================
void calculateSystemParameters() {
  Serial.println("\n=== Calculating System Parameters ===");
  
  // Calculate averages from individual test results
  float sum_zeta = 0;
  float sum_omega_n = 0;
  float sum_tau = 0;
  float sum_K1 = 0;
  int valid_count = 0;
  
  for (int i = 0; i < NUM_KP_TESTS * NUM_SETPOINT_TESTS; i++) {
    if (test_results[i].valid_test && test_results[i].tau > 0) {
      sum_zeta += test_results[i].zeta;
      sum_omega_n += test_results[i].omega_n;
      sum_tau += test_results[i].tau;
      sum_K1 += test_results[i].K1;
      valid_count++;
    }
  }
  
  if (valid_count == 0) {
    Serial.println("ERROR: No valid test data for parameter estimation");
    return;
  }
  
  // Calculate averages
  float avg_zeta = sum_zeta / valid_count;
  float avg_omega_n = sum_omega_n / valid_count;
  float avg_tau = sum_tau / valid_count;
  float avg_K1 = sum_K1 / valid_count;
  
  Serial.print("Valid tests used: ");
  Serial.println(valid_count);
  Serial.print("Average damping ratio (ζ): ");
  Serial.println(avg_zeta, 4);
  Serial.print("Average natural frequency (ωn): ");
  Serial.println(avg_omega_n, 4);
  Serial.print("Average τ (tau): ");
  Serial.println(avg_tau, 4);
  Serial.print("Average K1: ");
  Serial.println(avg_K1, 4);
}

// ================== Results Output ==================
void printResults() {
  Serial.println("\n=== Calibration Results ===");
  Serial.println("Kp,Setpoint,PO%,PeakTime(s),zeta,omega_n,tau,K1,Valid");
  
  for (int i = 0; i < NUM_KP_TESTS * NUM_SETPOINT_TESTS; i++) {
    Serial.print(test_results[i].kp, 1);
    Serial.print(",");
    Serial.print(test_results[i].setpoint, 3);
    Serial.print(",");
    Serial.print(test_results[i].percentage_overshoot, 2);
    Serial.print(",");
    Serial.print(test_results[i].peak_time, 3);
    Serial.print(",");
    Serial.print(test_results[i].zeta, 4);
    Serial.print(",");
    Serial.print(test_results[i].omega_n, 4);
    Serial.print(",");
    Serial.print(test_results[i].tau, 4);
    Serial.print(",");
    Serial.print(test_results[i].K1, 4);
    Serial.print(",");
    Serial.println(test_results[i].valid_test ? "1" : "0");
  }
}

// ================== Control ISR ==================
void interval_control_code(void) {
  float seconds = millis() / 1000.0f;
  int motor = analogRead(MOT_PIN);
  int ball = analogRead(BAL_PIN);
  float angle = getMotorAngle();
  
  digitalWrite(A5, HIGH);
  
  // Analyze response in real-time (no printing during tests)
  if (test_in_progress && current_setpoint > 0.01f) { // Only for non-zero setpoints
    float elapsed_time = (millis() - test_start_time) / 1000.0f;
    
    // Track maximum overshoot
    if (angle > current_setpoint && angle > max_overshoot) {
      max_overshoot = angle;
      if (!peak_found) {
        peak_time_value = elapsed_time;
        peak_found = true;
      }
    }
    
    // Track steady state (last 200ms of test)
    if (elapsed_time > TEST_DURATION/1000.0f - 0.2f) {
      steady_state_value = angle;
    }
  }
  
  digitalWrite(A5, LOW);
}
