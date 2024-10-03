#pragma once

constexpr double WHEEL_RADIUS = 58e-3;  // m
constexpr double WHEEL_BASE = 16.4e-3;
constexpr double SIN_PI_3 = 0.8660254037844386;

constexpr double FSC = 200; // motor fullsteps per rotation
constexpr double USC = 1; // microsteps
constexpr double TAU = 6.283185307179586; // 2pi

constexpr uint8_t STEPPER_CMDS_REPETITION = 1;
