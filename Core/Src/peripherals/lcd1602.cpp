#include "peripherals/lcd1602.hpp"

#define CHECK_HAL_STATUS(func_call)        \
    do {                                   \
        HAL_StatusTypeDef status = func_call; \
        if (status != HAL_OK)              \
            return status;                 \
    } while (0)

LCD1602_I2C::LCD1602_I2C(uint8_t lcd_addr) :
		lcd_addr(lcd_addr) {
}

HAL_StatusTypeDef LCD1602_I2C::init(I2C_HandleTypeDef *i2c) {
	i2c_ = i2c;

	// 4-bit initialization sequence
	HAL_Delay(50);  // Wait for >40ms
	CHECK_HAL_STATUS(send_cmd(0x30));
	HAL_Delay(5);   // Wait for >4.1ms
	CHECK_HAL_STATUS(send_cmd(0x30));
	HAL_Delay(1);   // Wait for >100us
	CHECK_HAL_STATUS(send_cmd(0x30));
	HAL_Delay(10);
	CHECK_HAL_STATUS(send_cmd(0x20));  // Set to 4-bit mode
	HAL_Delay(10);

	// Display initialization
	CHECK_HAL_STATUS(send_cmd(0x28)); // Function set: 4-bit mode, 2 lines, 5x8 dots
	HAL_Delay(1);
	CHECK_HAL_STATUS(send_cmd(0x08)); // Display off
	HAL_Delay(1);
	CHECK_HAL_STATUS(send_cmd(0x01)); // Clear display
	HAL_Delay(2);
	CHECK_HAL_STATUS(send_cmd(0x06)); // Entry mode set: increment cursor
	HAL_Delay(1);
	CHECK_HAL_STATUS(send_cmd(0x0C)); // Display on, cursor off, blink off

	return HAL_OK;
}

HAL_StatusTypeDef LCD1602_I2C::send_cmd(uint8_t cmd) {
	uint8_t data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xF0);
	data_l = ((cmd << 4) & 0xF0);
	data_t[0] = data_u | 0x0C;  // en=1, rs=0
	data_t[1] = data_u | 0x08;  // en=0, rs=0
	data_t[2] = data_l | 0x0C;  // en=1, rs=0
	data_t[3] = data_l | 0x08;  // en=0, rs=0
	return HAL_I2C_Master_Transmit(i2c_, lcd_addr, data_t, 4, 100);
}

HAL_StatusTypeDef LCD1602_I2C::send_data(const char data) {
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xF0);
	data_l = ((data << 4) & 0xF0);
	data_t[0] = data_u | 0x0D;  // en=1, rs=1
	data_t[1] = data_u | 0x09;  // en=0, rs=1
	data_t[2] = data_l | 0x0D;  // en=1, rs=1
	data_t[3] = data_l | 0x09;  // en=0, rs=1
	return HAL_I2C_Master_Transmit(i2c_, lcd_addr, data_t, 4, 100);
}

HAL_StatusTypeDef LCD1602_I2C::send_string(const char *str) {
	while (*str) {
		CHECK_HAL_STATUS(send_data(*str++));
	}
	return HAL_OK;
}

HAL_StatusTypeDef LCD1602_I2C::put_cursor(uint8_t row, uint8_t col) {
	switch (row) {
	case 0:
		col |= 0x80;
		break;
	case 1:
		col |= 0xC0;
		break;
	}
	return send_cmd(col);
}

HAL_StatusTypeDef LCD1602_I2C::clear() {
	CHECK_HAL_STATUS(send_cmd(0x01));  // Clear display command
	HAL_Delay(2);    // Wait for the command to execute
	return HAL_OK;
}
