#pragma once

#include <cstdint>

#include "constants.hpp"

#pragma pack(push, 1)

namespace pico {

struct UpdateSpeedsCommand {
    uint8_t header;
    uint8_t opcode;

    int32_t speeds[WHEEL_COUNT];
};

struct MoveElevatorCommand {
    uint8_t header;
    uint8_t opcode;

    int16_t position;
};

struct SetInterlockCommand {
    uint8_t header;
    uint8_t opcode;

    bool interlock;
};

}

#pragma pack(pop)
