#pragma once
#include <cstdint>

class WheelSpeedEstimator {
private:
    // Kalman filter variables
	static constexpr double q = 0.1;      // Process noise covariance
    static constexpr double r = 1.0;      // Measurement noise covariance
    double p = 0.0;      // Estimated error covariance

    int32_t prev_count_;
    uint32_t prev_time_;
    double speed_ = 0.0;      // Estimated state (wheel speed)
    bool first_run_ = true;
public:
    void update(uint16_t, uint32_t);

    double get_position(void);
    double get_speed(void);
};
