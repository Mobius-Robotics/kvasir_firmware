#pragma once

constexpr double WHEEL_RADIUS = 65e-3;  // m
constexpr double WHEEL_BASE = 200e-3; // m
constexpr double SIN_PI_3 = 0.8660254037844386;
constexpr double SQRT_3 = 1.7320508075688772;

constexpr double FSC = 200; // motor fullsteps per rotation
constexpr double USC = 1; // microsteps
constexpr double TAU = 6.283185307179586; // 2pi

constexpr uint8_t STEPPER_CMDS_REPETITION = 1;

constexpr int32_t ENCODER_FULL_RANGE = 4096;

constexpr uint8_t WHEEL_COUNT = 3;
