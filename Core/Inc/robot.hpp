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
    struct InitParams {
        UART_HandleTypeDef *wheel_uart0;
        UART_HandleTypeDef *wheel_uart1;

        UART_HandleTypeDef *elevator_uart;
        UART_HandleTypeDef *usb_uart;
        I2C_HandleTypeDef *i2c;

        TIM_HandleTypeDef *us_timer;
    };

    void init(const InitParams&);

    void recv_command(void);

    UART_HandleTypeDef *elevator_uart_ = nullptr;
    UART_HandleTypeDef *usb_uart_ = nullptr;
    I2C_HandleTypeDef *i2c_ = nullptr;

    TIM_HandleTypeDef *us_timer_;

    TMC2209 wheel_steppers_[WHEEL_COUNT];
    TMC2209 elevator_steppers_[ELEVATOR_COUNT];

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

    void home_elevator();
    void step_elevator(uint8_t steps, bool dir);

    void extend_pusher(bool pushers[TIM1_SERVOS]);
    void retract_pusher();

    void home_arm();
    void extend_arm();
    void retract_arm();

    void rising_pin_callback(uint16_t pin);
    void falling_pin_callback(uint16_t pin);

    void delay_us(uint16_t);
private:
    template<typename T> void recv_payload_and_execute(void);

    std::atomic<bool> interlock_flag_ { false };
    std::atomic<bool> pullstart_flag_ { false };
};

extern Robot robot;
