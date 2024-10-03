#pragma once

#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"
#include "peripherals/TMC2209.hpp"
#include "peripherals/pca9685.h"
#include "peripherals/multi_as5600.hpp"
#include "peripherals/lcd1602.hpp"
#include "commands.hpp"
#include "constants.hpp"

class Robot {
public:
	void init(UART_HandleTypeDef *tmc_uart, UART_HandleTypeDef *usb_uart,
			I2C_HandleTypeDef *i2c);
	void handle_command(void);
	void update(void);

	UART_HandleTypeDef *tmc_uart_ = nullptr;
	UART_HandleTypeDef *usb_uart_ = nullptr;
	I2C_HandleTypeDef *i2c_ = nullptr;

	TMC2209 stepper1, stepper2, stepper3;
	MultiAS5600 encoders;
	LCD1602_I2C lcd;
};
