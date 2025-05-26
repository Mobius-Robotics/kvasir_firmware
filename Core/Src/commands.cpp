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
    Status status { };

    status.interlock = robot.get_interlock();
    status.pullstart = robot.get_pullstart();

    status.endstops[0] = HAL_GPIO_ReadPin(Finecorsa1_GPIO_Port, Finecorsa1_Pin) == GPIO_PIN_SET;
    status.endstops[1] = HAL_GPIO_ReadPin(Finecorsa2_GPIO_Port, Finecorsa2_Pin) == GPIO_PIN_SET;

    HAL_UART_Transmit(robot.usb_uart_, reinterpret_cast<const uint8_t*>(&status), sizeof(Status),
            100);
}

void ElevatorCommand::execute() {
    robot.move_elevator(position);
}

void RetractArmCommand::execute() {
    robot.retract_arm();
}

void ExtendArmCommand::execute() {
    robot.extend_arm();
}

void RetractPusherCommand::execute() {
    robot.retract_pusher();
}

void ExtendPusherCommand::execute() {
    robot.extend_pusher(pushers);
}
