#include "wheel_speed_estimator.hpp"

#include "constants.hpp"

int32_t positive_mod(int32_t a, int32_t n) {
	return (a % n + n) % n;
}

double WheelSpeedEstimator::update(uint16_t current_count,
		uint32_t current_time) {
	// If first run, initialize variables
	if (first_run) {
		prev_count = current_count;
		prev_time = current_time;
		first_run = false;
		return x;
	}

	// Calculate time difference in seconds
	double delta_time = (current_time - prev_time) / 1000.0; // Assuming time in milliseconds

	// Handle time wrap-around if necessary
	if (delta_time <= 0.0f) {
		return x;
	}

	int32_t delta_pos = positive_mod(
			(current_count - prev_count + ENCODER_FULL_RANGE / 2),
			ENCODER_FULL_RANGE) - ENCODER_FULL_RANGE / 2;

	// Calculate wheel speed (radians per second)
	double measurement = (delta_pos / delta_time)
			/ ((double) ENCODER_FULL_RANGE) * TAU;

	// Kalman filter update

	// Predict step (state remains the same)
	p = p + q;

	// Update step
	double k = p / (p + r);
	x = x + k * (measurement - x);
	p = (1 - k) * p;

	// Get ready for the next step
	prev_count = current_count;
	prev_time = current_time;

	return x;
}
