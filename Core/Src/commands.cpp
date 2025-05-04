#include "commands.hpp"
#include "robot.hpp"
#include "main.h"

inline double rad_per_s_to_vactual(double u) {
    double v_rps = u / TAU; // revolutions per second
    double v_steps_per_second = v_rps * FSC * USC; // steps per second
    double vactual = v_steps_per_second / 0.715; // VACTUAL register, based on internal oscillator
    return vactual;
}

void SetServoCommand::execute() {
    static_assert(TIM1_SERVOS == 4 && TIM2_SERVOS == 2);
    TIM1->CCR1 = tim1_ccrs[0];
    TIM1->CCR2 = tim1_ccrs[1];
    TIM1->CCR3 = tim1_ccrs[2];
    TIM1->CCR4 = tim1_ccrs[3];
    TIM2->CCR1 = tim2_ccrs[0];
    TIM2->CCR2 = tim2_ccrs[1];
}

void SetWheelSpeedsCommand::execute() {
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        robot.wheel_steppers_[i].moveAtVelocity(rad_per_s_to_vactual(speeds[i]));
    }
}

void StopSteppersCommand::execute() {
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        robot.wheel_steppers_[i].moveAtVelocity(0);
    }
}

void PongCommand::execute() {
    const char pong[] = "pong";
    HAL_UART_Transmit(robot.usb_uart_, reinterpret_cast<const uint8_t*>(pong), sizeof(pong) - 1,
            100);
}
