#include "robot.hpp"

#include <type_traits>

void Robot::init(UART_HandleTypeDef *wheel_uart, UART_HandleTypeDef *elevator_uart, UART_HandleTypeDef *usb_uart,
        I2C_HandleTypeDef *i2c) {
    wheel_uart_ = wheel_uart;
    elevator_uart_ = elevator_uart;
    usb_uart_ = usb_uart;
    i2c_ = i2c;

    HAL_Delay(200);

    // Initialize stepper drivers.
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        auto &stepper = wheel_steppers_[i];
        stepper.setup(wheel_uart_, 115200, static_cast<TMC2209::SerialAddress>(i));
        stepper.enableAutomaticCurrentScaling();
        stepper.setRunCurrent(60);
        stepper.enableCoolStep();
        stepper.enable();
    }

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
        case 'u': {  // Set wheel speeds.
            recv_payload_and_execute<SetWheelSpeedsCommand>();
            break;
        }
        case 'x': {  // Stop all steppers.
            recv_payload_and_execute<StopSteppersCommand>();
            break;
        }
        case 'p': {  // Reply pong.
            recv_payload_and_execute<PongCommand>();
            break;
        }
        case 'h': {  // Check board health.
            recv_payload_and_execute<HealthCommand>();
            break;
        }
        default:
            usb_rx_buf_.discard(2);
            break;
    }
}

template<typename T> void Robot::recv_payload_and_execute(void) {
    T cmd;

    if (!std::is_empty<T>()) {
        // Check if we've got enough data to load the command.
        if (usb_rx_buf_.available() < (2 + sizeof(T))) return;

        // If so, discard header & opcode and start copying from the buffer into the command instance.
        usb_rx_buf_.discard(2);
        auto ptr = reinterpret_cast<uint8_t*>(&cmd);
        for (size_t i = 0; i < sizeof(T); ++i) {
            usb_rx_buf_.pop(ptr[i]);
        }
    } else {
        usb_rx_buf_.discard(2);
    }

    // Bombs away!
    cmd.execute();
}
