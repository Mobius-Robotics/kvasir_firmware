#pragma once
#include "stm32h5xx_hal.h"  // Adjust the include based on your STM32 series

class LCD1602_I2C {
public:
	// Constructor: accepts I2C handle and LCD I2C address (default is 0x4E)
	LCD1602_I2C(uint8_t lcd_addr = (0x27 << 1));

	// Initialize the LCD
	HAL_StatusTypeDef init(I2C_HandleTypeDef *hi2c);

	// Send command to the LCD
	HAL_StatusTypeDef send_cmd(uint8_t cmd);

	// Send data to the LCD
	HAL_StatusTypeDef send_data(const char data);

	// Send a string to the LCD
	HAL_StatusTypeDef send_string(const char *str);

	// Set cursor position
	HAL_StatusTypeDef put_cursor(uint8_t row, uint8_t col);

	// Clear the LCD display
	HAL_StatusTypeDef clear();

private:
	I2C_HandleTypeDef *i2c_ = nullptr;
	uint8_t lcd_addr;
};
