#pragma once

#include "as5600.h"

#define TCA9548A_ADDR  (0x71 << 1)  // Shifted left for HAL (7-bit address)
#define AS5600_COUNT 3

class MultiAS5600 {
public:
	MultiAS5600();

	HAL_StatusTypeDef init(I2C_HandleTypeDef*);
	HAL_StatusTypeDef update_cumulative_positions(void);
	void get_cumulative_positions(int32_t*);
private:
	I2C_HandleTypeDef *i2c_ = nullptr;
	AS5600_TypeDef *as5600_;
    uint16_t prev_angles_[AS5600_COUNT];
    int32_t cumulative_positions_[AS5600_COUNT];

	HAL_StatusTypeDef set_channel(uint8_t);
	HAL_StatusTypeDef read_sensors(uint16_t*);
};
