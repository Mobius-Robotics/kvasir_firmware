#pragma once
#include <cstdint>

class WheelSpeedEstimator {
private:
    // Kalman filter variables
	double q = 0.1;      // Process noise covariance
    double r = 1.0;      // Measurement noise covariance
    double x = 0.0;      // Estimated state (wheel speed)
    double p = 0.0;      // Estimated error covariance

    int32_t prev_count;
    uint32_t prev_time;
    bool first_run = true;
public:
    double update(uint16_t, uint32_t);
};
