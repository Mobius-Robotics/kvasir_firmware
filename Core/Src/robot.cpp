#include "robot.hpp"

#include <type_traits>

inline double rad_per_s_to_vactual(double u) {
    double v_rps = u / TAU; // revolutions per second
    double v_steps_per_second = v_rps * FSC * USC; // steps per second
    double vactual = v_steps_per_second / 0.715; // VACTUAL register, based on internal oscillator
    return vactual;
}

void Robot::init(const InitParams &params) {
    usb_uart_ = params.usb_uart;
    i2c_ = params.i2c;
    us_timer_ = params.us_timer;

    HAL_Delay(200);

    // Initialize stepper drivers.
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        auto &stepper = wheel_steppers_[i];
        stepper.setup(i < 2 ? params.wheel_uart0 : params.wheel_uart1,
                static_cast<TMC2209::SerialAddress>(i));
        stepper.enableAutomaticCurrentScaling();
        stepper.setRunCurrent(50);
        stepper.enableCoolStep();
        stepper.enable();
    }

    elevator_stepper_.setup(params.elevator_uart, static_cast<TMC2209::SerialAddress>(0));
    elevator_stepper_.enableAutomaticCurrentScaling();
    elevator_stepper_.setRunCurrent(50);
    elevator_stepper_.enableCoolStep();
    elevator_stepper_.enable();

    // Start the UART RX interrupt cycle.
    HAL_UART_Receive_IT(usb_uart_, &usb_rx_temp_, 1);

    // Check interlock preliminarily.
    set_interlock(HAL_GPIO_ReadPin(Emergency_GPIO_Port, Emergency_Pin) == GPIO_PIN_RESET);

    // Home TIM2 servos by moving outwards.
    if (HOME_ARM_SERVOS) {
        retract_arm();
    }
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
        case 'e': {  // Step elevator.
            recv_payload_and_execute<ElevatorCommand>();
            break;
        }
        case 'T': {  // Extend tin pusher.
            recv_payload_and_execute<ExtendPusherCommand>();
            break;
        }
        case 't': {  // Retract tin pusher.
            recv_payload_and_execute<RetractPusherCommand>();
            break;
        }
        case 'o': {  // Extend plank arm.
            recv_payload_and_execute<ExtendArmCommand>();
            break;
        }
        case 'i': {  // Retract plank arm.
            recv_payload_and_execute<RetractArmCommand>();
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

void Robot::set_wheel_speeds(int32_t speeds[WHEEL_COUNT]) {
    if (get_interlock()) return;
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        robot.wheel_steppers_[i].moveAtVelocity(rad_per_s_to_vactual(speeds[i]));
    }
}

void Robot::set_servo_tim1_ccrs(uint16_t tim1_ccrs[TIM1_SERVOS]) {
    if (get_interlock()) return;
    static_assert(TIM1_SERVOS == 4);
    TIM1->CCR1 = tim1_ccrs[0];
    TIM1->CCR2 = tim1_ccrs[1];
    TIM1->CCR3 = tim1_ccrs[2];
    TIM1->CCR4 = tim1_ccrs[3];
}

void Robot::set_servo_tim2_ccrs(uint16_t tim2_ccrs[TIM2_SERVOS]) {
    if (get_interlock()) return;
    static_assert(TIM2_SERVOS == 2);
    TIM2->CCR1 = tim2_ccrs[0];
    TIM2->CCR2 = tim2_ccrs[1];
}

void Robot::set_servo_ccrs(uint16_t tim1_ccrs[TIM1_SERVOS], uint16_t tim2_ccrs[TIM2_SERVOS]) {
    set_servo_tim1_ccrs(tim1_ccrs);
    set_servo_tim2_ccrs(tim2_ccrs);
}

bool Robot::get_interlock() {
    return interlock_flag_.load(std::memory_order_acquire);
}

void Robot::set_interlock(bool val) {
    if (val) {
        // engaging the interlock stops stepper motors and continuous servos
        int32_t speeds[WHEEL_COUNT] { };
        uint16_t tim2_ccrs[TIM2_SERVOS] { };
        for (size_t i = 0; i < TIM2_SERVOS; ++i)
            tim2_ccrs[i] = ARM_SERVO_STOP_CCR;
        set_wheel_speeds(speeds);
        set_servo_tim2_ccrs(tim2_ccrs);
        set_pullstart(false);
    }

    interlock_flag_.store(val, std::memory_order_release);
}

bool Robot::get_pullstart() {
    return pullstart_flag_.load(std::memory_order_acquire);
}

void Robot::set_pullstart(bool val) {
    pullstart_flag_.store(val, std::memory_order_release);
}

void Robot::delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(us_timer_, 0);            // Reset counter to 0
    while (__HAL_TIM_GET_COUNTER(us_timer_) < us)
        // Wait until CNT reaches ‘us’
        ;
}

void Robot::step_elevator(uint8_t steps, bool dir) {
    HAL_GPIO_WritePin(ElevatorDir_GPIO_Port, ElevatorDir_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for (uint8_t i = 0; i < steps; ++i) {
        if (get_interlock()) return;
        HAL_GPIO_WritePin(ElevatorStep_GPIO_Port, ElevatorStep_Pin, GPIO_PIN_SET);
        delay_us(1);
        HAL_GPIO_WritePin(ElevatorStep_GPIO_Port, ElevatorStep_Pin, GPIO_PIN_RESET);
        delay_us(1);
    }
}

void Robot::retract_arm() {
    uint16_t tim2_ccrs[TIM2_SERVOS];
    for (size_t i = 0; i < TIM2_SERVOS; ++i)
        tim2_ccrs[i] = ARM_SERVO_STOP_CCR;

    // Retract right arm until we are not hitting the endstop anymore.
    tim2_ccrs[0] = ARM_LEFT_SERVO_RETRACT_CCR;
    while (HAL_GPIO_ReadPin(Finecorsa1_GPIO_Port, Finecorsa1_Pin) == GPIO_PIN_RESET) {
        set_servo_tim2_ccrs(tim2_ccrs);
    }
    // Extend left arm until we are hitting the endstop.
    tim2_ccrs[0] = ARM_LEFT_SERVO_EXTEND_CCR;
    while (HAL_GPIO_ReadPin(Finecorsa1_GPIO_Port, Finecorsa1_Pin) == GPIO_PIN_SET) {
        set_servo_tim2_ccrs(tim2_ccrs);
    }
    tim2_ccrs[0] = ARM_SERVO_STOP_CCR;
    set_servo_tim2_ccrs(tim2_ccrs);

    // Repeat for right arm.
    tim2_ccrs[1] = ARM_RIGHT_SERVO_RETRACT_CCR;
    while (HAL_GPIO_ReadPin(Finecorsa2_GPIO_Port, Finecorsa2_Pin) == GPIO_PIN_RESET) {
        set_servo_tim2_ccrs(tim2_ccrs);

    }
    tim2_ccrs[1] = ARM_RIGHT_SERVO_EXTEND_CCR;
    while (HAL_GPIO_ReadPin(Finecorsa2_GPIO_Port, Finecorsa2_Pin) == GPIO_PIN_SET) {
        set_servo_tim2_ccrs(tim2_ccrs);
    }
    tim2_ccrs[1] = ARM_SERVO_STOP_CCR;
    set_servo_tim2_ccrs(tim2_ccrs);
}

void Robot::extend_arm() {
    retract_arm();
    uint16_t tim2_ccrs[TIM2_SERVOS];
    for (size_t i = 0; i < TIM2_SERVOS; ++i)
        tim2_ccrs[i] = ARM_SERVO_STOP_CCR;

    tim2_ccrs[0] = ARM_LEFT_SERVO_EXTEND_CCR;
    tim2_ccrs[1] = ARM_RIGHT_SERVO_EXTEND_CCR;
    set_servo_tim2_ccrs(tim2_ccrs);
    HAL_Delay(ARM_EXTENSION_TIME_MS);

    for (size_t i = 0; i < TIM2_SERVOS; ++i)
        tim2_ccrs[i] = ARM_SERVO_STOP_CCR;
    set_servo_tim2_ccrs(tim2_ccrs);
}

void Robot::extend_pusher() {
    uint16_t tim1_ccrs[TIM1_SERVOS];
    for (size_t i = 0; i < TIM1_SERVOS; ++i)
        tim1_ccrs[i] = PUSHER_EXTEND_CCRS[i];
    set_servo_tim1_ccrs(tim1_ccrs);
}

void Robot::retract_pusher() {
    uint16_t tim1_ccrs[TIM1_SERVOS];
    for (size_t i = 0; i < TIM1_SERVOS; ++i)
        tim1_ccrs[i] = PUSHER_RETRACT_CCRS[i];
    set_servo_tim1_ccrs(tim1_ccrs);
}

void Robot::rising_pin_callback(uint16_t pin) {
    switch (pin) {
        case Emergency_Pin:
            // Button is active low
            set_interlock(false);
            break;

        case Cordicella_Pin:
            // Pull-start cord goes HIGH when pulled
            set_pullstart(true);
            break;
    }
}

void Robot::falling_pin_callback(uint16_t pin) {
    switch (pin) {
        case Emergency_Pin:
            // Button is active low
            set_interlock(true);
            break;
    }
}
