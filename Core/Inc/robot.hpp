#pragma once

#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"
#include "peripherals/TMC2209.hpp"
#include "commands.hpp"
#include "constants.hpp"
#include "main.h"
#include "ring_buffer.hpp"

class Robot {
public:
    void init(UART_HandleTypeDef *wheel_uart, UART_HandleTypeDef *elevator_uart, UART_HandleTypeDef *usb_uart, I2C_HandleTypeDef *i2c);

    void recv_command(void);

    UART_HandleTypeDef *wheel_uart_ = nullptr;
    UART_HandleTypeDef *elevator_uart_ = nullptr;
    UART_HandleTypeDef *usb_uart_ = nullptr;
    I2C_HandleTypeDef *i2c_ = nullptr;

    TMC2209 wheel_steppers_[WHEEL_COUNT];

    RingBuffer usb_rx_buf_;
    uint8_t usb_rx_temp_;

private:
    template<typename T> void recv_payload_and_execute(void);
};

extern Robot robot;
