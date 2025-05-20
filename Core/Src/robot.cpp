#include "robot.hpp"

#include <type_traits>

inline double rad_per_s_to_vactual(double u) {
    double v_rps = u / TAU; // revolutions per second
    double v_steps_per_second = v_rps * FSC * USC; // steps per second
    double vactual = v_steps_per_second / 0.715; // VACTUAL register, based on internal oscillator
    return vactual;
}

void Robot::init(UART_HandleTypeDef *wheel_uart, UART_HandleTypeDef *elevator_uart,
        UART_HandleTypeDef *usb_uart, I2C_HandleTypeDef *i2c, GPIO_TypeDef *elevator_step_port,
        uint16_t elevator_step_pin,

        GPIO_TypeDef *elevator_dir_port, uint16_t elevator_dir_pin, TIM_HandleTypeDef *us_timer) {
    wheel_uart_ = wheel_uart;
    elevator_uart_ = elevator_uart;
    usb_uart_ = usb_uart;
    i2c_ = i2c;

    elevator_step_port_ = elevator_step_port;
    elevator_step_pin_ = elevator_step_pin;
    elevator_dir_port_ = elevator_dir_port;
    elevator_dir_pin_ = elevator_dir_pin;
    us_timer_ = us_timer;

    HAL_Delay(200);

    // Initialize stepper drivers.
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        auto &stepper = wheel_steppers_[i];
        stepper.setup(wheel_uart_, 115200, static_cast<TMC2209::SerialAddress>(i));
        stepper.enableAutomaticCurrentScaling();
        stepper.setRunCurrent(50);
        stepper.enableCoolStep();
        stepper.enable();
    }

    elevator_stepper_.setup(elevator_uart_, 115200, static_cast<TMC2209::SerialAddress>(0));
    elevator_stepper_.enableAutomaticCurrentScaling();
    elevator_stepper_.setRunCurrent(50);
    elevator_stepper_.enableCoolStep();
    elevator_stepper_.enable();

    // Start the UART RX interrupt cycle.
    HAL_UART_Receive_IT(usb_uart_, &usb_rx_temp_, 1);

    // Check interlock preliminarily.
    set_interlock(HAL_GPIO_ReadPin(Emergency_GPIO_Port, Emergency_Pin) == GPIO_PIN_RESET);

    // Home TIM2 servos by moving outwards.
    if (false) { /* Endstops aren't wired yet */
        uint16_t tim2_ccrs[TIM2_SERVOS];
        for (size_t i = 0; i < TIM2_SERVOS; ++i)
            tim2_ccrs[i] = (TIM2->ARR + 1) / 4 * 3;
        set_servo_tim2_ccrs(tim2_ccrs);
        while (HAL_GPIO_ReadPin(Finecorsa1_GPIO_Port, Finecorsa1_Pin) == GPIO_PIN_SET
                && HAL_GPIO_ReadPin(Finecorsa2_GPIO_Port, Finecorsa2_Pin) == GPIO_PIN_SET) {
        }
        for (size_t i = 0; i < TIM2_SERVOS; ++i)
            tim2_ccrs[i] = (TIM2->ARR + 1) / 2;
        set_servo_tim2_ccrs(tim2_ccrs);
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
        case 'e': {  // Step elevator.
            recv_payload_and_execute<ElevatorCommand>();
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
            tim2_ccrs[i] = (TIM2->ARR + 1) / 2;
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
    if (get_interlock()) return;
    HAL_GPIO_WritePin(elevator_dir_port_, elevator_step_pin_, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for (uint8_t i = 0; i < steps; ++i) {
        HAL_GPIO_WritePin(elevator_step_port_, elevator_step_pin_, GPIO_PIN_SET);
        delay_us(1);
        HAL_GPIO_WritePin(elevator_step_port_, elevator_step_pin_, GPIO_PIN_RESET);
        delay_us(1);
    }
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
