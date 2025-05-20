#pragma once

#include <atomic>

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

    void set_wheel_speeds(int32_t speeds[WHEEL_COUNT]);

    void set_servo_tim1_ccrs(uint16_t tim1_ccrs[TIM1_SERVOS]);
    void set_servo_tim2_ccrs(uint16_t tim2_ccrs[TIM2_SERVOS]);
    void set_servo_ccrs(uint16_t tim1_ccrs[TIM1_SERVOS], uint16_t tim2_ccrs[TIM2_SERVOS]);

    bool get_interlock();
    void set_interlock(bool);

    bool get_pullstart();
    void set_pullstart(bool);

    void rising_pin_callback(uint16_t pin);
    void falling_pin_callback(uint16_t pin);
private:
    template<typename T> void recv_payload_and_execute(void);

    std::atomic<bool> interlock_flag_{false};
    std::atomic<bool> pullstart_flag_{false};
};

extern Robot robot;
