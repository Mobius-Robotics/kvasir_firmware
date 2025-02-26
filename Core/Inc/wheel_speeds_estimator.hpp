#pragma once

#include "peripherals/as5600.h"
#include "wheel_speed_estimator.hpp"
#include "constants.hpp"

constexpr uint8_t TCA9548A_ADDR = (0x71 << 1);  // Shifted left for HAL (7-bit address)

#pragma pack(push, 1)
struct WheelInfo {
	double wheel1_pos, wheel2_pos, wheel3_pos;
	double wheel1_speed, wheel2_speed, wheel3_speed;
};
#pragma pack(pop)

class WheelSpeedsEstimator {
public:
	HAL_StatusTypeDef init(I2C_HandleTypeDef*);
	HAL_StatusTypeDef update(void);
	WheelInfo get_wheel_info(void);
private:
	I2C_HandleTypeDef *i2c_ = nullptr;
	AS5600_TypeDef *as5600_;
    WheelSpeedEstimator wheel1_, wheel2_, wheel3_;

    uint32_t prev_time_ = 0;

	HAL_StatusTypeDef set_channel(uint8_t);
	HAL_StatusTypeDef read_sensors(uint16_t*);

	bool initialized_ = false;
};
