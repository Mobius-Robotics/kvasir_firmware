#include "robot.hpp"
#include "peripherals/ssd1306.h"
#include "peripherals/ssd1306_fonts.h"
#include "peripherals/pico_steppers.hpp"

#include <cstring>
#include <type_traits>

#define OledFont Font_11x18

inline double rad_per_s_to_vactual(double u) {
    double v_rps = u / TAU; // revolutions per second
    double v_steps_per_second = v_rps * FSC * USC; // steps per second
    double vactual = v_steps_per_second / 0.715; // VACTUAL register, based on internal oscillator
    return vactual;
}

void Robot::init(const InitParams &params) {
    pico_uart_ = params.pico_uart;
    usb_uart_ = params.usb_uart;
    i2c_ = params.i2c;

    HAL_Delay(200);

    ssd1306_Init();
    ssd1306_UpdateScreen();

    // Start the UART RX interrupt cycle.
    HAL_UART_Receive_IT(usb_uart_, &usb_rx_temp_, 1);

    // Check interlock preliminarily.
    set_interlock(HAL_GPIO_ReadPin(Emergency_GPIO_Port, Emergency_Pin) == GPIO_PIN_RESET);

    // Home TIM2 servos by moving outwards.
    if (HOME_ARM_SERVOS) {
        home_arm();
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
    //if (get_interlock()) return;
    pico::UpdateSpeedsCommand
    cmd { .header = 'M', .opcode = 'u', };
    std::memcpy(&cmd.speeds, speeds, sizeof(int32_t) * WHEEL_COUNT);

    HAL_UART_Transmit(robot.pico_uart_, reinterpret_cast<const uint8_t*>(&cmd),
            sizeof(pico::UpdateSpeedsCommand), 100);
}

void Robot::set_servo_tim1_ccrs(uint16_t tim1_ccrs[TIM1_SERVOS]) {
//    if (get_interlock()) return;
    static_assert(TIM1_SERVOS == 4);
    TIM1->CCR1 = tim1_ccrs[0];
    TIM1->CCR2 = tim1_ccrs[1];
    TIM1->CCR3 = tim1_ccrs[2];
    TIM1->CCR4 = tim1_ccrs[3];
}

void Robot::set_servo_tim2_ccrs(uint16_t tim2_ccrs[TIM2_SERVOS]) {
//    if (get_interlock()) return;
    static_assert(TIM2_SERVOS == 2);
    TIM2->CCR1 = tim2_ccrs[0];
    TIM2->CCR2 = tim2_ccrs[1];
}

void Robot::set_servo_ccrs(uint16_t tim1_ccrs[TIM1_SERVOS], uint16_t tim2_ccrs[TIM2_SERVOS]) {
    set_servo_tim1_ccrs(tim1_ccrs);
    set_servo_tim2_ccrs(tim2_ccrs);
}

bool Robot::get_interlock() {
    return interlock_flag_.load(std::memory_order_seq_cst);
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

    pico::SetInterlockCommand cmd { .header = 'M', .opcode = 'l', .interlock = val };
    HAL_UART_Transmit(robot.pico_uart_, reinterpret_cast<const uint8_t*>(&cmd),
            sizeof(pico::SetInterlockCommand), 100);

    interlock_flag_.store(val, std::memory_order_seq_cst);

    update_screen();
}

void Robot::update_screen() {
    ssd1306_Fill(Black);

    ssd1306_SetCursor(5, 5);
    char header[] = "Pts: 20";
    ssd1306_WriteString(header, OledFont, White);

    ssd1306_SetCursor(5, 5 + 2 * OledFont.height);
    char footer[] = "Mobius <3";
    ssd1306_WriteString(footer, OledFont, White);

    {
        ssd1306_SetCursor(5, 5 + OledFont.height);
        char msg1[] = "IL:S ";
        char msg0[] = "IL:R ";
        ssd1306_WriteString(get_interlock() ? msg1 : msg0, OledFont, White);
    }

    {
        char msg1[] = "PS:S ";
        char msg0[] = "PS:R ";
        ssd1306_WriteString(get_pullstart() ? msg1 : msg0, OledFont, White);
    }

    ssd1306_UpdateScreen();
}

bool Robot::get_pullstart() {
    return pullstart_flag_.load(std::memory_order_seq_cst);
}

void Robot::set_pullstart(bool val) {
    pullstart_flag_.store(val, std::memory_order_seq_cst);
    update_screen();
}

void Robot::move_elevator(int16_t position) {
    //if (get_interlock()) return;
    pico::MoveElevatorCommand
    cmd { .header = 'M', .opcode = 'u', .position = position };

    HAL_UART_Transmit(robot.pico_uart_, reinterpret_cast<const uint8_t*>(&cmd),
            sizeof(pico::MoveElevatorCommand), 100);
}

void Robot::home_arm() {
    uint16_t tim2_ccrs[TIM2_SERVOS];
    for (size_t i = 0; i < TIM2_SERVOS; ++i)
        tim2_ccrs[i] = ARM_SERVO_STOP_CCR;

    // Retract right arm until we are not hitting the endstop anymore.
    tim2_ccrs[0] = ARM_LEFT_SERVO_RETRACT_CCR;
    while (HAL_GPIO_ReadPin(Finecorsa1_GPIO_Port, Finecorsa1_Pin) == GPIO_PIN_SET) {
        set_servo_tim2_ccrs(tim2_ccrs);
    }
    // Extend left arm until we are hitting the endstop.
    tim2_ccrs[0] = ARM_LEFT_SERVO_EXTEND_CCR;
    while (HAL_GPIO_ReadPin(Finecorsa1_GPIO_Port, Finecorsa1_Pin) == GPIO_PIN_RESET) {
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

void Robot::retract_arm() {
    home_arm();
    uint16_t tim2_ccrs[TIM2_SERVOS];
    for (size_t i = 0; i < TIM2_SERVOS; ++i)
        tim2_ccrs[i] = ARM_SERVO_STOP_CCR;

    tim2_ccrs[0] = ARM_LEFT_SERVO_RETRACT_CCR;
    tim2_ccrs[1] = ARM_RIGHT_SERVO_RETRACT_CCR;
    set_servo_tim2_ccrs(tim2_ccrs);
    HAL_Delay(ARM_RETRACTION_TIME_MS);

    for (size_t i = 0; i < TIM2_SERVOS; ++i)
        tim2_ccrs[i] = ARM_SERVO_STOP_CCR;
    set_servo_tim2_ccrs(tim2_ccrs);
}

void Robot::extend_arm() {
    home_arm();
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

void Robot::extend_pusher(bool pushers[TIM1_SERVOS]) {
    uint16_t tim1_ccrs[TIM1_SERVOS];
    for (size_t i = 0; i < TIM1_SERVOS; ++i)
        tim1_ccrs[i] = pushers[i] ? PUSHER_EXTEND_CCRS[i] : PUSHER_RETRACT_CCRS[i];
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
