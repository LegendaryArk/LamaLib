#pragma once

#include "odometry.hpp"
#include "../utilities/mathHelper.hpp"

namespace lamaLib {
struct MotionData {
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
};

struct MotionProfile {
	std::vector<MotionData> profile;
};

struct MotionLimit {
	double startVelocity;
	double maxVelocity;
	double maxAcceleration;
};

MotionProfile generateTrapezoid(MotionLimit imotionLimit, double idistance);
} // namespace lamaLib