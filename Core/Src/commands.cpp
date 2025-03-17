// commands.cpp

#include "commands.hpp"

#include "robot.hpp"

#include <cstring> // for memcpy

extern Robot robot;
extern "C" void Error_Handler(void);

inline double rad_per_s_to_vactual(double u) {
	double v_rps = u / TAU; // revolutions per second
	double v_steps_per_second = v_rps * FSC * USC; // steps per second
	double vactual = v_steps_per_second / 0.715; // VACTUAL register, based on internal oscillator
	return vactual;
}

void SetServoCommand::execute() {
	TIM1->CCR1 = ccr1;
	TIM1->CCR2 = ccr2;
}

void ReadWheelInfoCommand::execute() {
	WheelInfo wheel_info = robot.wheel_speeds_estimator_.get_wheel_info();
	HAL_UART_Transmit(robot.usb_uart_, reinterpret_cast<uint8_t*>(&wheel_info),
			sizeof(wheel_info), 100);
}

void SetWheelSpeedsCommand::execute() {
	for (uint8_t i = 0; i < STEPPER_CMDS_REPETITION; ++i) {
		robot.stepper1_.moveAtVelocity(rad_per_s_to_vactual(speeds[0]));
		robot.stepper2_.moveAtVelocity(rad_per_s_to_vactual(speeds[1]));
		robot.stepper3_.moveAtVelocity(rad_per_s_to_vactual(speeds[2]));
	}
}

void StopSteppersCommand::execute() {
	for (uint8_t i = 0; i < STEPPER_CMDS_REPETITION; ++i) {
		robot.stepper1_.moveAtVelocity(0);
		robot.stepper2_.moveAtVelocity(0);
		robot.stepper3_.moveAtVelocity(0);
	}
}

void PongCommand::execute() {
	const char pong[] = "pong";
	HAL_UART_Transmit(robot.usb_uart_, reinterpret_cast<const uint8_t*>(pong),
			sizeof(pong) - 1, 100);
}

void InverseKinematicsCommand::execute() {
	double u1 = (-WHEEL_BASE * theta_dot + x_dot) / WHEEL_RADIUS;
	double u2 = (-WHEEL_BASE * theta_dot - x_dot / 2 - y_dot * SIN_PI_3)
			/ WHEEL_RADIUS;
	double u3 = (-WHEEL_BASE * theta_dot - x_dot / 2 + y_dot * SIN_PI_3)
			/ WHEEL_RADIUS;

	double vactual1 = rad_per_s_to_vactual(u1);
	double vactual2 = rad_per_s_to_vactual(u2);
	double vactual3 = rad_per_s_to_vactual(u3);

	for (uint8_t i = 0; i < STEPPER_CMDS_REPETITION; ++i) {
		robot.stepper1_.moveAtVelocity(static_cast<int32_t>(vactual1));
		robot.stepper2_.moveAtVelocity(static_cast<int32_t>(vactual2));
		robot.stepper3_.moveAtVelocity(static_cast<int32_t>(vactual3));
	}
}

void LcdPrintCommand::execute() {
	if (line >= 2) return;
	robot.lcd_.put_cursor(line, 0);
	char buf[LCD_WIDTH + 1] {}; // zero-initialized to ensure the string is NUL terminated.
	std::memcpy(buf, msg, LCD_WIDTH);
	robot.lcd_.send_string(buf);
}
