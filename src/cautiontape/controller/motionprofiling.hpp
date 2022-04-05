#pragma once

#include "api.h"
#include "../utilities/mathHelper.hpp"

namespace lamaLib {
/**
 * @brief A point of data in the profile
 */
struct MotionData {
	/**
	* Distance
	* Velocity
	* Acceleration
	* Jerk
	* Time
	*/
	double distance {0};
	double velocity {0};
	double acceleration {0};
	double jerk {0};
	double time {0};
	double x {0};
	double y {0};
	double heading {0};
};
/**
 * @brief The motion profile
 */
struct MotionProfile {
	std::vector<MotionData> profile;
};

/**
 * @brief Controls the max speed and acceleration
 */
struct MotionLimit {
	double maxVelocity;
	double maxAcceleration;
};

/**
 * @brief When the cutoffs begins and ends
 */
struct CutoffPoint {
	double distance;
	double velocity {0};
	double time {0};
};

/**
 * @brief Generates a trapezoidal profile
 * 
 * @param imotionLimit The max velocity and acceleration
 * @param istart The starting position, speed, and time
 * @param iend The ending position, speed, and time
 * @return MotionProfile 
 */
MotionProfile generateTrapezoid(MotionLimit imotionLimit, CutoffPoint istart, CutoffPoint iend);
} // namespace lamaLib