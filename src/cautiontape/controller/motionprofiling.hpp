#pragma once

#include "odometry.hpp"
#include "../utilities/mathHelper.hpp"

namespace lamaLib {
struct MotionData {
	/**
	* Distance
	* Velocity
	* Acceleration
	* Jerk
	* Position  X, Y, Heading
	*/
	double distance {0};
	double velocity {0};
	double acceleration {0};
	double jerk {0};
	Pose position {0};
};

struct MotionProfile {
	std::vector<MotionData> profile;
};

struct MotionLimit {
	double maxVelocity;
	double maxAcceleration;
};

MotionProfile generateTrapezoid(MotionLimit imotionLimit, MotionData istart, MotionData iend);
} // namespace lamaLib