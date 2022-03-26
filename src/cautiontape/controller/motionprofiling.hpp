#pragma once

#include "odometry.hpp"

namespace lamaLib {
typedef struct {
	/*
	* Position  X, Y, Heading
	* Velocity
	* Acceleration
	* Jerk
	*/
	Pose position;
	double distance;
	double velocity;
	double acceleration;
	double jerk;
} MotionData;

typedef struct {
	std::vector<MotionData> profile;
} MotionProfile;

typedef struct {
	double startVelocity;
	double maxVelocity;
	double maxAcceleration;
} MotionLimit;

MotionProfile generateTrapezoid (MotionLimit imotionLimit, double idistance);
} // namespace lamaLib