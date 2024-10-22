#include <peripherals/position_estimator.hpp>
#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"

#define CHECK_HAL_STATUS(func_call)           \
    do {                                      \
        HAL_StatusTypeDef status = func_call; \
        if (status != HAL_OK)                 \
            return status;                    \
    } while (0)

HAL_StatusTypeDef PositionEstimator::init(I2C_HandleTypeDef *hi2c) {
	i2c_ = hi2c;

	as5600_ = AS5600_New();
	if (as5600_ == NULL)
		return HAL_ERROR;
	as5600_->i2cHandle = i2c_;
	as5600_->i2cAddr = (0x36 << 1); // AS5600 I2C address shifted for STM32 HAL

	for (uint8_t channel = 0; channel < WHEEL_COUNT; ++channel) {
		CHECK_HAL_STATUS(set_channel(2 + channel));
		CHECK_HAL_STATUS(AS5600_Init(as5600_));
	}

	return HAL_OK;
}

HAL_StatusTypeDef PositionEstimator::read_sensors(uint16_t *buf) {
	for (uint8_t channel = 0; channel < WHEEL_COUNT; ++channel) {
		CHECK_HAL_STATUS(set_channel(2 + channel));
		CHECK_HAL_STATUS(AS5600_GetAngle(as5600_, buf + channel));
	}
	return HAL_OK;
}

HAL_StatusTypeDef PositionEstimator::set_channel(uint8_t channel) {
	if (channel > 7)
		return HAL_ERROR;  // Invalid channel number

	uint8_t cmd = (1 << channel) | 0b01100000;
	auto status = HAL_I2C_Master_Transmit(i2c_, TCA9548A_ADDR, &cmd, 1,
	HAL_MAX_DELAY);
	return status;
}

inline int32_t positive_mod(int32_t a, int32_t n) {
	return (a % n + n) % n;
}

HAL_StatusTypeDef PositionEstimator::update(void) {
	uint32_t current_time = HAL_GetTick();
	if (prev_time_ != 0) {
		uint16_t counts[WHEEL_COUNT];
		CHECK_HAL_STATUS(read_sensors(counts));

		double u1 = wheel1_.update(counts[0], current_time);
		double u2 = wheel2_.update(counts[1], current_time);
		double u3 = wheel3_.update(counts[2], current_time);

		double x_dot = -WHEEL_RADIUS / 3 * (-2 * u1 + u2 + u3);
		double y_dot = -SQRT_3 * WHEEL_RADIUS / 3 * (u2 - u3);
		double psi_dot = -WHEEL_RADIUS / (3 * WHEEL_BASE) * (u1 + u2 + u3);

		double delta_time = (current_time - prev_time_) / 1000.0;
		if (delta_time > 0) {
			x_ += x_dot * delta_time;
			y_ += y_dot * delta_time;
			psi_ += psi_dot * delta_time;
		}
	}
	prev_time_ = current_time;

	return HAL_OK;
}

Position PositionEstimator::get_position(void) {
	return Position { x_, y_, psi_ };
}
