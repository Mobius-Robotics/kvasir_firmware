#include "robot.hpp"

extern "C" void Error_Handler(void);

void Robot::init(UART_HandleTypeDef *tmc_uart, UART_HandleTypeDef *usb_uart,
		I2C_HandleTypeDef *i2c) {
	tmc_uart_ = tmc_uart;
	usb_uart_ = usb_uart;
	i2c_ = i2c;

	// Initialize stepper drivers.
	stepper1.setup(tmc_uart_, 115200, TMC2209::SERIAL_ADDRESS_0);
	stepper1.enableAutomaticCurrentScaling();
	stepper1.setRunCurrent(100);
	stepper1.enable();

	stepper2.setup(tmc_uart_, 115200, TMC2209::SERIAL_ADDRESS_1);
	stepper2.enableAutomaticCurrentScaling();
	stepper2.setRunCurrent(100);
	stepper2.enable();

	stepper3.setup(tmc_uart_, 115200, TMC2209::SERIAL_ADDRESS_2);
	stepper3.enableAutomaticCurrentScaling();
	stepper3.setRunCurrent(100);
	stepper3.enable();

	// Initialize encoder multiplexer.
	// NB: this must be the first I2C peripheral to be initialized,
	//     as it sets up the bus multiplexing for the other peripherals!
	if (position_estimator.init(i2c_) != HAL_OK)
		Error_Handler();

	// Initialize servo driver.
	if (PCA9685_Init(i2c_) != PCA9685_OK)
		Error_Handler();

	// Initialize LCD screen.
	lcd.init(i2c_);
	lcd.put_cursor(0, 0);
	lcd.send_string("IT'S ALIVE!");
	lcd.put_cursor(1, 0);
	lcd.send_string("- Dr. Frankenstein");
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
	case 'a': {  // Read angles
		ReadAnglesCommand cmd;
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
	default:
		// Handle unknown command if necessary
		break;
	}
}

void Robot::update(void) {
	position_estimator.update();
}
