#ifndef __TRANSFER_FUNCTION_H__
#define __TRANSFER_FUNCTION_H__

#include <Arduino.h>

class TransferFunction {
private:
  int num_poles;
  const float* numerator;
  const float* denominator;
  float* input_history;
  float* output_history;
  unsigned long sample_time_ms;
  unsigned long last_compute_time;
  float last_output;

public:
  // Constructor: takes numerator, denominator arrays, number of poles, and sample time in milliseconds
  TransferFunction(const float* num, const float* den, int poles, unsigned long sample_time_ms = 0) 
    : num_poles(poles), numerator(num), denominator(den), sample_time_ms(sample_time_ms) {
    // Allocate and initialize input history (size = num_poles)
    input_history = new float[num_poles];
    for (int i = 0; i < num_poles; i++) {
      input_history[i] = 0.0f;
    }
    
    // Allocate and initialize output history (size = num_poles + 1)
    output_history = new float[num_poles + 1];
    for (int i = 0; i < num_poles + 1; i++) {
      output_history[i] = 0.0f;
    }
    
    // Initialize timing variables
    last_compute_time = 0;
    last_output = 0.0f;
  }
  
  // Destructor: free allocated memory
  ~TransferFunction() {
    delete[] input_history;
    delete[] output_history;
  }
  
  // Compute transfer function output given current input value
  // Only recomputes if enough time has passed since last computation (if sample_time_ms > 0)
  float compute(float current_value) {
    unsigned long current_time = millis();
    
    // If sample_time_ms is 0, always compute (backward compatibility)
    // Otherwise, check if enough time has passed
    if (sample_time_ms > 0) {
      // Check if enough time has passed (handle millis() overflow)
      unsigned long time_since_last = (current_time >= last_compute_time) 
        ? (current_time - last_compute_time) 
        : (UINT32_MAX - last_compute_time + current_time + 1);
      
      if (time_since_last < sample_time_ms) {
        // Not enough time has passed, return last output
        return last_output;
      }
    }
    
    // Update input history first (shift right, then insert new value at index 0)
    for (int i = num_poles - 1; i > 0; i--) {
      input_history[i] = input_history[i - 1];
    }
    input_history[0] = current_value;

    // Compute output using transfer function difference equation
    // y[k] = (b[0]*u[k] + b[1]*u[k-1] + ... - a[1]*y[k-1] - a[2]*y[k-2] - ...) / a[0]
    float output = 0.0f;
    
    // Numerator (feedforward) terms: b[0]*u[k] + b[1]*u[k-1] + ... + b[n-1]*u[k-n+1]
    for (int i = 0; i < num_poles; i++) {
      output += numerator[i] * input_history[i];
    }
    
    // Denominator (feedback) terms: -a[1]*y[k-1] - a[2]*y[k-2] - ... - a[n]*y[k-n]
    // Skip a[0] as it's the leading coefficient (used for normalization)
    for (int i = 0; i < num_poles; i++) {
      output -= denominator[i + 1] * output_history[i];
    }
    
    // Normalize by leading denominator coefficient (typically 1.0)
    if (denominator[0] != 0.0f) {
      output /= denominator[0];
    }
    
    // Update output history (shift right, then insert new output at index 0)
    for (int i = num_poles; i > 0; i--) {
      output_history[i] = output_history[i - 1];
    }
    output_history[0] = output;

    // Update timing and last output
    last_compute_time = current_time;
    last_output = output;

    return output;
  }
};

#endif // __TRANSFER_FUNCTION_H__