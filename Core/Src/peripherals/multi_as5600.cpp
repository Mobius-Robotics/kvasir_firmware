#include "peripherals/multi_as5600.hpp"

#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"

#define CHECK_HAL_STATUS(func_call)        \
    do {                                   \
        HAL_StatusTypeDef status = func_call; \
        if (status != HAL_OK)              \
            return status;                 \
    } while (0)

MultiAS5600::MultiAS5600() {
}

HAL_StatusTypeDef MultiAS5600::init(I2C_HandleTypeDef *hi2c) {
	i2c_ = hi2c;

	as5600_ = AS5600_New();
	if (as5600_ == NULL)
		return HAL_ERROR;
	as5600_->i2cHandle = i2c_;
	as5600_->i2cAddr = (0x36 << 1); // AS5600 I2C address shifted for STM32 HAL

	for (uint8_t channel = 0; channel < AS5600_COUNT; ++channel) {
		CHECK_HAL_STATUS(set_channel(2 + channel));
		CHECK_HAL_STATUS(AS5600_Init(as5600_));
	}

	uint16_t angles[AS5600_COUNT];
	CHECK_HAL_STATUS(read_sensors(angles));
	for (uint8_t i = 0; i < AS5600_COUNT; i++) {
		prev_angles_[i] = angles[i];
		cumulative_positions_[i] = angles[i];
	}
	return HAL_OK;
}

HAL_StatusTypeDef MultiAS5600::read_sensors(uint16_t *buf) {
	for (uint8_t channel = 0; channel < AS5600_COUNT; ++channel) {
		CHECK_HAL_STATUS(set_channel(2 + channel));
		CHECK_HAL_STATUS(AS5600_GetAngle(as5600_, buf + channel));
	}
	return HAL_OK;
}

HAL_StatusTypeDef MultiAS5600::set_channel(uint8_t channel) {
	if (channel > 7)
		return HAL_ERROR;  // Invalid channel number

	uint8_t cmd = (1 << channel) | 0b01100000;
	auto status = HAL_I2C_Master_Transmit(i2c_, TCA9548A_ADDR, &cmd, 1,
			HAL_MAX_DELAY);
	auto error = HAL_I2C_GetError(i2c_);
	return status;
}

int32_t positive_mod(int32_t a, int32_t n) {
	return (a % n + n) % n;
}

HAL_StatusTypeDef MultiAS5600::update_cumulative_positions(void) {
	uint16_t angles[AS5600_COUNT];
	CHECK_HAL_STATUS(read_sensors(angles));

	const int32_t full_range = 4096;
	const int32_t half_range = full_range / 2;

	for (uint8_t i = 0; i < AS5600_COUNT; ++i) {
		int32_t delta = positive_mod((angles[i] - prev_angles_[i] + half_range),
				full_range) - half_range;
		cumulative_positions_[i] += delta;
		prev_angles_[i] = angles[i];
	}

	return HAL_OK;
}

void MultiAS5600::get_cumulative_positions(int32_t *buf) {
	for (uint8_t i = 0; i < AS5600_COUNT; ++i) {
		buf[i] = cumulative_positions_[i];
	}
}
