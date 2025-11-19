#ifndef __TRANSFER_FUNCTION_H__
#define __TRANSFER_FUNCTION_H__

#include <Arduino.h>
#include "circular_queue.h"

class TransferFunction {
private:
  int num_zeros;
  int num_poles;
  const float* numerator;
  const float* denominator;
  CircularQueue* input_history;
  CircularQueue* output_history;
  unsigned long sample_time_ms;
  unsigned long last_compute_time;
  float last_output;

public:
  // Constructor: takes numerator, denominator arrays, number of zeros, number of poles, and sample time in milliseconds
  TransferFunction(const float* num, const float* den, int zeros, int poles, unsigned long sample_time_ms = 0) 
    : num_zeros(zeros), num_poles(poles), numerator(num), denominator(den), sample_time_ms(sample_time_ms) {
    // Create circular queues for history
    // Input history needs to store num_zeros past inputs
    input_history = new CircularQueue(num_zeros);
    // Output history needs to store num_poles past outputs
    output_history = new CircularQueue(num_poles);
    
    // Initialize timing variables
    last_compute_time = 0;
    last_output = 0.0f;
  }
  
  // Destructor: free allocated memory
  ~TransferFunction() {
    delete input_history;
    delete output_history;
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
    
    // Update input history (push new value to front of circular queue)
    input_history->push(current_value);

    // Compute output using transfer function difference equation
    // y[k] = (b[0]*u[k] + b[1]*u[k-1] + ... - a[1]*y[k-1] - a[2]*y[k-2] - ...) / a[0]
    float output = 0.0f;
    
    // Numerator (feedforward) terms: b[0]*u[k] + b[1]*u[k-1] + ... + b[n-1]*u[k-n+1]
    // get(0) is the most recent (u[k]), get(1) is u[k-1], etc.
    for (int i = 0; i < num_zeros; i++) {
      output += numerator[i] * input_history->get(i);
    }
    
    // Denominator (feedback) terms: -a[1]*y[k-1] - a[2]*y[k-2] - ... - a[n]*y[k-n]
    // Skip a[0] as it's the leading coefficient (used for normalization)
    // get(0) is the most recent past output (y[k-1]), get(1) is y[k-2], etc.
    for (int i = 0; i < num_poles; i++) {
      output -= denominator[i + 1] * output_history->get(i);
    }
    
    // Normalize by leading denominator coefficient (typically 1.0)
    if (denominator[0] != 0.0f) {
      output /= denominator[0];
    }
    
    // Update output history (push new output to front of circular queue)
    output_history->push(output);

    // Update timing and last output
    last_compute_time = current_time;
    last_output = output;

    return output;
  }
};

#endif // __TRANSFER_FUNCTION_H__