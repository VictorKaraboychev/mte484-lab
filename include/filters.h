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

class HighPassFilter : public Filter
{
public:
	// Constructor: cutoffFrequency in Hz, sampleRate in Hz
	HighPassFilter(float cutoffFrequency, float sampleRate)
	{
		float dt = 1.0f / sampleRate;
		float RC = 1.0f / (2.0f * M_PI * cutoffFrequency);
		this->alpha = RC / (RC + dt);
		this->output = 0.0f;
		this->prevInput = 0.0f;
		this->initialized = false;
	}

	virtual ~HighPassFilter() {}

	// Update the filter with a new input sample and return the filtered output.
	virtual float process(float input)
	{
		if (!this->initialized)
		{
			// On the first sample, initialize previous input to avoid a transient.
			this->prevInput = input;
			this->output = input;
			this->initialized = true;
		}
		else
		{
			// Compute the high-pass filter output
			// y[n] = alpha * (y[n-1] + x[n] - x[n-1])
			this->output = this->alpha * (this->output + input - this->prevInput);
			this->prevInput = input;
		}
		return this->output;
	}

private:
	float alpha;
	float output;
	float prevInput;
	bool initialized;
};

class BandPassFilter : public Filter
{
public:
	// Constructor: lowCutoff (Hz) is the lower bound and highCutoff (Hz) is the upper bound.
	// sampleRate is in Hz.
	BandPassFilter(float lowCutoff, float highCutoff, float sampleRate)
		: hp(lowCutoff, sampleRate), lp(highCutoff, sampleRate)
	{
	}

	virtual ~BandPassFilter() {}

	// The update function cascades the high-pass and low-pass filters.
	virtual float process(float input)
	{
		float highPassed = hp.process(input);
		float bandPassed = lp.process(highPassed);
		return bandPassed;
	}

private:
	HighPassFilter hp;
	LowPassFilter lp;
};

class BandRejectFilter : public Filter
{
public:
	// Constructor: lowCutoff (Hz) and highCutoff (Hz) define the rejection band.
	// sampleRate is in Hz.
	BandRejectFilter(float lowCutoff, float highCutoff, float sampleRate)
		: bp(lowCutoff, highCutoff, sampleRate)
	{
	}

	virtual ~BandRejectFilter() {}

	// The update function subtracts the bandpass output from the input.
	virtual float process(float input)
	{
		float bandComponent = bp.process(input);
		// The rejected signal is the input minus the frequencies in the band.
		float output = input - bandComponent;
		return output;
	}

private:
	BandPassFilter bp;
};

#endif // __FILTERS_H__