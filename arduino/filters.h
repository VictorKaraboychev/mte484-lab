#ifndef __FILTERS_H__
#define __FILTERS_H__

#include <cmath>

class Filter
{
public:
	Filter() {}
	virtual ~Filter() {}

	virtual float process(float input) = 0;
};

class LowPassFilter : public Filter
{
public:
	// Constructor: cutoffFrequency in Hz, sampleRate in Hz
	LowPassFilter(float cutoffFrequency, float sampleRate)
	{
		float dt = 1.0f / sampleRate;
		float RC = 1.0f / (2.0f * M_PI * cutoffFrequency);
		this->alpha = dt / (RC + dt);
		this->output = 0.0f;
		this->initialized = false;
	}

	virtual ~LowPassFilter() {}

	// Update the filter with a new input sample and return the filtered output.
	virtual float process(float input)
	{
		if (!this->initialized)
		{
			this->output = input; // Initialize with the first sample to avoid startup transients.
			this->initialized = true;
		}

		// Compute the low-pass filter output
		// y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
		this->output = this->alpha * input + (1.0f - this->alpha) * this->output;
		return this->output;
	}

private:
	float alpha;
	float output;
	bool initialized;
};

#endif // __FILTERS_H__