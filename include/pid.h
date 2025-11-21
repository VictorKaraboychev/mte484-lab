#ifndef __PID_H__
#define __PID_H__

#include <cmath>

class PID
{
public:
	PID(float kp, float ki = 0, float kd = 0, float min_output = -INFINITY, float max_output = INFINITY, float min_integral = -INFINITY, float max_integral = INFINITY)
	{
		this->kp = kp;
		this->ki = ki;
		this->kd = kd;

		this->min_output = min_output;
		this->max_output = max_output;

		this->min_integral = min_integral;
		this->max_integral = max_integral;

		this->integral = 0;
		this->last_error = 0;
	}

	~PID() {}

	float update(float error, uint32_t current_time_ms)
	{
		float dt = (current_time_ms - this->last_time_ms) / 1000.0f;
		this->last_time_ms = current_time_ms;

		this->integral += error * dt;
		this->integral = fminf(fmaxf(this->integral, this->min_integral), this->max_integral);

		float derivative = (error - this->last_error) / dt;

		float output = this->kp * error + this->ki * this->integral + this->kd * derivative;

		output = fminf(fmaxf(output, this->min_output), this->max_output);

		this->last_error = error;

		return output;
	}

	void reset()
	{
		this->integral = 0;
		this->last_error = 0;
		this->last_time_ms = 0;
	}

private:
	float kp;
	float ki;
	float kd;

	float min_output;
	float max_output;

	float min_integral;
	float max_integral;

	float integral;
	float last_error;
	uint32_t last_time_ms;
};

#endif // __PID_H__