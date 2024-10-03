#pragma once

#include <cstdint>

// Ensure structs are packed to avoid padding
#pragma pack(push, 1)

// Struct for 's' command - Set servo
struct SetServoCommand {
	uint8_t channel;
	uint16_t onTime;
	uint16_t offTime;

	void process();
};

// Struct for 'a' command - Read angles
struct ReadAnglesCommand {
	void process();
};

// Struct for 'u' command - Set wheel speeds
struct SetWheelSpeedsCommand {
	int32_t speeds[3];

	void process();
};

// Struct for 'x' command - Stop all steppers
struct StopSteppersCommand {
	void process();
};

// Struct for 'p' command - Reply pong
struct PongCommand {
	void process();
};

// Struct for 'k' command - Set wheel velocities via inverse kinematics
struct InverseKinematicsCommand {
	double x_dot;
	double y_dot;
	double theta_dot;

	void process();
};

#pragma pack(pop)
