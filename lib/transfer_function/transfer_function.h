#pragma once

class TransferFunction {
private:
  int num_poles;
  const float* numerator;
  const float* denominator;
  float* input_history;
  float* output_history;

public:
  // Constructor: takes numerator, denominator arrays and number of poles
  TransferFunction(const float* num, const float* den, int poles) 
    : num_poles(poles), numerator(num), denominator(den) {
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
  }
  
  // Destructor: free allocated memory
  ~TransferFunction() {
    delete[] input_history;
    delete[] output_history;
  }
  
  // Compute transfer function output given current input value
  float compute(float current_value) {
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

    return output;
  }
};

