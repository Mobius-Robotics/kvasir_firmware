#pragma once

#include <cstddef>

constexpr double FSC = 200; // motor fullsteps per rotation
constexpr double USC = 1; // microsteps
constexpr double TAU = 6.283185307179586; // 2pi

constexpr uint8_t WHEEL_COUNT = 4;

constexpr size_t TIM1_SERVOS = 4;
constexpr size_t TIM2_SERVOS = 2;

/* Continuous arm servos are on TIM2
 * Left servo has 1500 stop, 1600 forward and 1400 backwards
 * Right servo has 1500 stop, 1400 forward and 1600 backwards
 */
constexpr uint16_t ARM_SERVO_STOP_CCR = 1500;
constexpr uint16_t ARM_LEFT_SERVO_EXTEND_CCR = 1600;
constexpr uint16_t ARM_LEFT_SERVO_RETRACT_CCR = 1400;
constexpr uint16_t ARM_RIGHT_SERVO_EXTEND_CCR = 1400;
constexpr uint16_t ARM_RIGHT_SERVO_RETRACT_CCR = 1600;

constexpr bool HOME_ARM_SERVOS = false;

constexpr uint32_t ARM_EXTENSION_TIME_MS = 250;
constexpr uint32_t ARM_RETRACTION_TIME_MS = 250;


constexpr uint16_t PUSHER_EXTEND_CCRS[TIM1_SERVOS] = {1750, 1600, 1600, 1600};
constexpr uint16_t PUSHER_RETRACT_CCRS[TIM1_SERVOS] = {1210, 1175, 1260, 1230};
