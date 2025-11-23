#ifndef __PID_H__
#define __PID_H__

#include <cmath>
#include <cstdint>

class PID
{
public:
	PID(float kp, float ki = 0, float kd = 0, uint32_t sample_time_ms = 0, float min_output = -INFINITY, float max_output = INFINITY, float min_integral = -INFINITY, float max_integral = INFINITY)
	{
		this->kp = kp;
		this->ki = ki;
		this->kd = kd;

		this->min_output = min_output;
		this->max_output = max_output;

		this->min_integral = min_integral;
		this->max_integral = max_integral;

		this->sample_time_ms = sample_time_ms;
		this->integral = 0;
		this->last_error = 0;
		this->last_compute_time = 0;
		this->last_output = 0.0f;
	}

	~PID() {}

	float compute(float error, uint32_t current_time_ms)
	{
		uint32_t current_time = current_time_ms;
		
		// Convert sampling time from milliseconds to seconds
		float sampling_time_s = this->sample_time_ms / 1000.0f;
		
		// If sample_time_ms is 0, always compute (backward compatibility)
		// Otherwise, check if enough time has passed
		if (this->sample_time_ms > 0) {
			// Check if enough time has passed (handle millis() overflow)
			uint32_t time_since_last = (current_time >= this->last_compute_time) 
				? (current_time - this->last_compute_time) 
				: (UINT32_MAX - this->last_compute_time + current_time + 1);
			
			if (time_since_last < this->sample_time_ms) {
				// Not enough time has passed, return last output
				return this->last_output;
			}
		}
		
		// Time to sample - update PID controller
		
		// Proportional term: u_p = Kp * e
		float u_p = this->kp * error;
		
		// Calculate dt for backward compatibility mode (when sample_time_ms is 0)
		float dt = 0.0f;
		if (this->sample_time_ms == 0) {
			dt = (current_time - this->last_time_ms) / 1000.0f;
			if (dt <= 0.0f || this->last_time_ms == 0) {
				dt = 0.001f;  // Default to 1ms if invalid
			}
		}
		
		// Integral term: u_i[k] = u_i[k-1] + Ki * Ts * e[k]
		// Use fixed sampling time, not actual time difference
		if (this->sample_time_ms > 0) {
			this->integral += this->ki * sampling_time_s * error;
		} else {
			// Backward compatibility: use actual time difference if sampling_time is 0
			this->integral += this->ki * dt * error;
		}
		
		// Apply integral constraints
		this->integral = fminf(fmaxf(this->integral, this->min_integral), this->max_integral);
		float u_i = this->integral;
		
		// Derivative term: u_d[k] = Kd * (e[k] - e[k-1]) / Ts
		// Skip derivative on first sample (when last_compute_time is 0 or invalid)
		float u_d = 0.0f;
		if (this->last_compute_time != 0) {
			float error_diff = error - this->last_error;
			if (this->sample_time_ms > 0) {
				// Use fixed sampling time (consistent with Python implementation)
				u_d = this->kd * (error_diff / sampling_time_s);
			} else {
				// Backward compatibility: use actual time difference (dt calculated above)
				u_d = this->kd * (error_diff / dt);
			}
		}
		
		// Update last_time_ms for backward compatibility mode
		if (this->sample_time_ms == 0) {
			this->last_time_ms = current_time;
		}
		
		// Total output: u = Kp*e + Ki*integral + Kd*derivative
		// Note: integral already includes Ki, so u = Kp*e + integral + Kd*derivative
		float output = u_p + u_i + u_d;
		
		// Apply output constraints
		output = fminf(fmaxf(output, this->min_output), this->max_output);
		
		// Update state
		this->last_error = error;
		this->last_compute_time = current_time;
		this->last_output = output;

		return output;
	}

	void reset()
	{
		this->integral = 0.0f;
		this->last_error = 0.0f;
		this->last_time_ms = 0;
		this->last_compute_time = 0;
		this->last_output = 0.0f;
	}

private:
	float kp;
	float ki;
	float kd;

	float min_output;
	float max_output;

	float min_integral;
	float max_integral;

	uint32_t sample_time_ms;
	float integral;  // Integral accumulator (already includes Ki)
	float last_error;
	uint32_t last_time_ms;  // Only used when sample_time_ms is 0 (backward compatibility)
	uint32_t last_compute_time;
	float last_output;
};

#endif // __PID_H__