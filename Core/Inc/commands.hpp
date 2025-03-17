#pragma once

#include <cstdint>

#include "constants.hpp"

// Ensure structs are packed to avoid padding
#pragma pack(push, 1)

// Struct for 's' command - Set servo
struct SetServoCommand {
	uint16_t ccr1;
	uint16_t ccr2;

	void execute();
};

// Struct for 'a' command - Read angles
struct ReadWheelInfoCommand {
	void execute();
};

// Struct for 'u' command - Set wheel speeds
struct SetWheelSpeedsCommand {
	int32_t speeds[3];

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

// Struct for 'k' command - Set wheel velocities via inverse kinematics
struct InverseKinematicsCommand {
	double x_dot;
	double y_dot;
	double theta_dot;

	void execute();
};

// Struct for 'l' command - Print message to LCD
struct LcdPrintCommand {
	uint8_t line;
	char msg[LCD_WIDTH];

	void execute();
};

#pragma pack(pop)
