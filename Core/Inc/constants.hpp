#pragma once

#include <cstddef>

constexpr double FSC = 200; // motor fullsteps per rotation
constexpr double USC = 1; // microsteps
constexpr double TAU = 6.283185307179586; // 2pi

constexpr uint8_t WHEEL_COUNT = 4;

constexpr size_t TIM1_SERVOS = 4;
constexpr size_t TIM2_SERVOS = 2;
