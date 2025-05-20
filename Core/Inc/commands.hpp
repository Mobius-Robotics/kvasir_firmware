#pragma once

#include <cstdint>

#include "constants.hpp"

// Ensure structs are packed to avoid padding
#pragma pack(push, 1)

// Struct for 's' command - Set servo
struct SetServoCommand {
    uint16_t tim1_ccrs[TIM1_SERVOS];
    uint16_t tim2_ccrs[TIM2_SERVOS];

    void execute();
};

// Struct for 'u' command - Set wheel speeds
struct SetWheelSpeedsCommand {
    int32_t speeds[WHEEL_COUNT];

    void execute();
};

// Struct for 'x' command - Stop all steppers
struct StopSteppersCommand {
    void execute();
};

// Struct for 'p' command - Reply pong
struct PongCommand {
    void execute();
};

// Struct for 'h' command - Reply with board health
struct HealthCommand  {
    void execute();
};

struct Status {
    bool setupAndComms[WHEEL_COUNT];
    bool notSetupButComms[WHEEL_COUNT];

    bool pullstart;
    bool interlock;
};

// Struct for 'e' command - Move elevator
struct ElevatorCommand {
    uint8_t steps;
    bool dir;

    void execute();
};

#pragma pack(pop)
