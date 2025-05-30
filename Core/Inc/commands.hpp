#pragma once

#include <cstdint>

#include "constants.hpp"
#include "peripherals/TMC2209.hpp"

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
    bool pullstart;
    bool interlock;

    bool endstops[2];
};

// Struct for 'e' command - Move elevator
struct ElevatorCommand {
    uint16_t position;

    void execute();
};

// Struct for 'T' command - Extend tin pusher
struct ExtendPusherCommand {
    bool pushers[TIM1_SERVOS];

    void execute();
};

// Struct for 't' command - Retract tin pusher
struct RetractPusherCommand {
    void execute();
};

// Struct for 'i' command - Retract plank arm
struct RetractArmCommand {
    void execute();
};

// Struct for 'o' command - Extend plank arm
struct ExtendArmCommand {
    void execute();
};

#pragma pack(pop)
