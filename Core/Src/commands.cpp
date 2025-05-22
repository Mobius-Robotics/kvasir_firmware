#include "commands.hpp"
#include "robot.hpp"
#include "main.h"

void SetServoCommand::execute() {
    robot.set_servo_ccrs(tim1_ccrs, tim2_ccrs);
}

void SetWheelSpeedsCommand::execute() {
    robot.set_wheel_speeds(speeds);
}

void StopSteppersCommand::execute() {
    int32_t speeds[WHEEL_COUNT] = { 0 };
    robot.set_wheel_speeds(speeds);
}

void PongCommand::execute() {
    const char pong[] = "pong";
    HAL_UART_Transmit(robot.usb_uart_, reinterpret_cast<const uint8_t*>(pong), sizeof(pong) - 1,
            100);
}

void HealthCommand::execute() {
    Status status{};

    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        status.setupAndComms[i] = robot.wheel_steppers_[i].isSetupAndCommunicating();
        status.notSetupButComms[i] = robot.wheel_steppers_[i].isCommunicatingButNotSetup();
        status.driverStatuses[i] = robot.wheel_steppers_[i].getStatus();
        status.driverGlobalStatuses[i] = robot.wheel_steppers_[i].getGlobalStatus();
    }

    status.interlock = robot.get_interlock();
    status.pullstart = robot.get_pullstart();

    HAL_UART_Transmit(robot.usb_uart_, reinterpret_cast<const uint8_t*>(&status), sizeof(Status),
            100);
}

void ElevatorCommand::execute() {
    robot.step_elevator(steps, dir);
}
