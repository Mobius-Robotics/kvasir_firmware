// commands.cpp

#include "commands.hpp"

#include "robot.hpp"

extern Robot robot;
extern "C" void Error_Handler(void);

void SetServoCommand::process() {
	if (PCA9685_SetPwm(channel, onTime, offTime) != PCA9685_OK) {
		Error_Handler();
	}
}

void ReadAnglesCommand::process() {
	int32_t positions[3];
	robot.encoders.get_cumulative_positions(positions);
	HAL_UART_Transmit(robot.usb_uart_, (uint8_t*) positions, sizeof(positions),
			100);
}

void SetWheelSpeedsCommand::process() {
	for (uint8_t i = 0; i < STEPPER_CMDS_REPETITION; ++i) {
		robot.stepper1.moveAtVelocity(speeds[0]);
		robot.stepper2.moveAtVelocity(speeds[1]);
		robot.stepper3.moveAtVelocity(speeds[2]);
	}
}

void StopSteppersCommand::process() {
	for (uint8_t i = 0; i < STEPPER_CMDS_REPETITION; ++i) {
		robot.stepper1.moveAtVelocity(0);
		robot.stepper2.moveAtVelocity(0);
		robot.stepper3.moveAtVelocity(0);
	}
}

void PongCommand::process() {
	const char pong[] = "pong";
	HAL_UART_Transmit(robot.usb_uart_, reinterpret_cast<const uint8_t*>(pong),
			sizeof(pong) - 1, 100);
}

inline double rad_per_s_to_vactual(double u) {
	double v_rps = u / TAU; // revolutions per second
	double v_steps_per_second = v_rps * FSC * USC; // steps per second
	double vactual = v_steps_per_second / 0.715; // VACTUAL register, based on internal oscillator
	return vactual;
}

void InverseKinematicsCommand::process() {
	double u1 = (-WHEEL_BASE * theta_dot + x_dot) / WHEEL_RADIUS;
	double u2 = (-WHEEL_BASE * theta_dot - x_dot / 2 - y_dot * SIN_PI_3)
			/ WHEEL_RADIUS;
	double u3 = (-WHEEL_BASE * theta_dot - x_dot / 2 + y_dot * SIN_PI_3)
			/ WHEEL_RADIUS;

	double vactual1 = rad_per_s_to_vactual(u1);
	double vactual2 = rad_per_s_to_vactual(u2);
	double vactual3 = rad_per_s_to_vactual(u3);

	for (uint8_t i = 0; i < STEPPER_CMDS_REPETITION; ++i) {
		robot.stepper1.moveAtVelocity(static_cast<int32_t>(vactual1));
		robot.stepper2.moveAtVelocity(static_cast<int32_t>(vactual2));
		robot.stepper3.moveAtVelocity(static_cast<int32_t>(vactual3));
	}
}
