#pragma once

#include <wheel_speeds_estimator.hpp>
#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"
#include "peripherals/TMC2209.hpp"
#include "peripherals/pca9685.h"
#include "peripherals/lcd1602.hpp"
#include "commands.hpp"
#include "constants.hpp"
#include "main.h"

class Robot {
public:
	void init(UART_HandleTypeDef *tmc_uart, UART_HandleTypeDef *usb_uart,
			I2C_HandleTypeDef *i2c);
	void handle_command(void);
	void update(void);

	UART_HandleTypeDef *tmc_uart_ = nullptr;
	UART_HandleTypeDef *usb_uart_ = nullptr;
	I2C_HandleTypeDef *i2c_ = nullptr;

	TMC2209 stepper1_, stepper2_, stepper3_;
	WheelSpeedsEstimator wheel_speeds_estimator_;
	LCD1602_I2C lcd_;
};
