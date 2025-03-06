#include "robot.hpp"

void Robot::init(UART_HandleTypeDef *tmc_uart, UART_HandleTypeDef *usb_uart,
		I2C_HandleTypeDef *i2c) {
	tmc_uart_ = tmc_uart;
	usb_uart_ = usb_uart;
	i2c_ = i2c;

	// Initialize stepper drivers.
	stepper1_.setup(tmc_uart_, 115200, TMC2209::SERIAL_ADDRESS_0);
	stepper1_.enableAutomaticCurrentScaling();
	stepper1_.setRunCurrent(100);
	stepper1_.enable();

	stepper2_.setup(tmc_uart_, 115200, TMC2209::SERIAL_ADDRESS_1);
	stepper2_.enableAutomaticCurrentScaling();
	stepper2_.setRunCurrent(100);
	stepper2_.enable();

	stepper3_.setup(tmc_uart_, 115200, TMC2209::SERIAL_ADDRESS_2);
	stepper3_.enableAutomaticCurrentScaling();
	stepper3_.setRunCurrent(100);
	stepper3_.enable();

	// Initialize LCD screen.
	lcd_.init(i2c_);
	lcd_.put_cursor(0, 0);
	lcd_.send_string("<3 from Mobius");
}

void Robot::handle_command(void) {
	uint8_t header;

	// Wait to receive the header byte 'M'
	if (HAL_UART_Receive(usb_uart_, &header, 1, 100) != HAL_OK)
		return;
	if (header != 'M') {
		// Invalid header byte; early return
		return;
	}

	uint8_t command;
	if (HAL_UART_Receive(usb_uart_, &command, 1, 100) != HAL_OK)
		return;

	switch (command) {
	case 's': {  // Set servo
		SetServoCommand cmd;
		if (HAL_UART_Receive(usb_uart_, reinterpret_cast<uint8_t*>(&cmd),
				sizeof(cmd), 100) != HAL_OK)
			return;
		cmd.process();
		break;
	}
	case 'a': {  // Read wheel info
		ReadWheelInfoCommand cmd;
		cmd.process();
		break;
	}
	case 'u': {  // Set wheel speeds
		SetWheelSpeedsCommand cmd;
		if (HAL_UART_Receive(usb_uart_, reinterpret_cast<uint8_t*>(&cmd),
				sizeof(cmd), 100) != HAL_OK)
			return;
		cmd.process();
		break;
	}
	case 'x': {  // Stop all steppers
		StopSteppersCommand cmd;
		cmd.process();
		break;
	}
	case 'p': {  // Reply pong
		PongCommand cmd;
		cmd.process();
		break;
	}
	case 'k': {  // Set wheel velocities via inverse kinematics
		InverseKinematicsCommand cmd;
		if (HAL_UART_Receive(usb_uart_, reinterpret_cast<uint8_t*>(&cmd),
				sizeof(cmd), 100) != HAL_OK)
			return;
		cmd.process();
		break;
	}
	case 'l': { // Print to LCD
		LcdPrintCommand cmd;
		if (HAL_UART_Receive(usb_uart_, reinterpret_cast<uint8_t*>(&cmd),
				sizeof(cmd), 100) != HAL_OK)
			return;
		cmd.process();
		break;
	}
	default:
		// Handle unknown command if necessary
		break;
	}
}

void Robot::update(void) {
	wheel_speeds_estimator_.update();
}
