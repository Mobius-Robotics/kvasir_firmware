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

	// TIM1 ARR sets the PWM frequency, empirically set to 20067 instead of the 19999 it should theoretically be for 50 Hz.
	//TIM1->ARR = 20067;

	// Start the UART RX interrupt cycle.
	HAL_UART_Receive_IT(usb_uart_, &usb_rx_temp_, 1);
}

void Robot::recv_command(void) {
	// Load header and opcode from the recv buffer, if available.
	uint8_t header, opcode;
	if (!usb_rx_buf_.peek_two(header, opcode)) {
		__WFI(); // If we haven't got two bytes available, go into sleep mode until the next interrupt.
		return;
	}

	// Check the header: if it isn't correct, we should discard it and move on, perhaps there's been some sort of noise.
	if (header != 'M') {
		usb_rx_buf_.discard();
		return;
	}

	// Based on the opcode, choose the correct command type.
	// NB: Those without a payload need to manually handle discarding their header and opcode.
	switch (opcode) {
	case 's': {  // Set servo CCRs.
		recv_payload_and_execute<SetServoCommand>();
		break;
	}
	case 'a': {  // Read wheel info.
		ReadWheelInfoCommand cmd;
		cmd.execute();
		usb_rx_buf_.discard(2);
		break;
	}
	case 'u': {  // Set wheel speeds.
		recv_payload_and_execute<SetWheelSpeedsCommand>();
		break;
	}
	case 'x': {  // Stop all steppers.
		StopSteppersCommand cmd;
		cmd.execute();
		usb_rx_buf_.discard(2);
		break;
	}
	case 'p': {  // Reply pong.
		PongCommand cmd;
		cmd.execute();
		usb_rx_buf_.discard(2);
		break;
	}
	case 'k': {  // Set wheel velocities via inverse kinematics.
		recv_payload_and_execute<InverseKinematicsCommand>();
		break;
	}
	case 'l': { // Print to LCD.
		recv_payload_and_execute<LcdPrintCommand>();
		break;
	}
	default:
		usb_rx_buf_.discard(2);
		break;
	}
}

template<typename T> void Robot::recv_payload_and_execute(void) {
	// Check if we've got enough data to load the command.
	T cmd;
	if (usb_rx_buf_.available() < (2+sizeof(T))) return;

	// If so, discard header & opcode and start copying from the buffer into the command instance.
	usb_rx_buf_.discard(2);
	auto ptr = reinterpret_cast<uint8_t*>(&cmd);
	for (size_t i = 0; i < sizeof(T); ++i) {
		usb_rx_buf_.pop(ptr[i]);
	}

	// Bombs away!
	cmd.execute();
}
