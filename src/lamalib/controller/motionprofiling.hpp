#pragma once

#include "api.h"
#include "../utilities/pose.hpp"
#include "../utilities/mathhelper.hpp"
#include <vector>

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
	Pose position {0, 0, 0};
};
/**
 * @brief The motion profile
 */
struct MotionProfile {
	std::vector<MotionData> profile;

	/**
	 * @brief Combines 2 motion profiles
	 * 
	 * @param rhs The profile that is added on to the end of this profile
	 */
	void operator+=(MotionProfile rhs);
};
/**
 * @brief The profile on the acceleration and deceleration portion of a motion profile
 */
struct AccelProfile {
	std::vector<MotionData> accelerationProfile;
	std::vector<MotionData> decelerationProfile;
	double accelerationDist;
	double accelerationTime;
	double decelerationDist;
	double decelerationTime;
};

/**
 * @brief Controls the max speed and acceleration
 */
struct MotionLimit {
	double maxVelocity;
	double maxAcceleration;
	double maxJerk {0};
	
	/**
	 * @brief Multiplies the max velocities and max accelerations by a factor
	 * 
	 * @param rhs The factor
	 * @return The new MotionLimit with the products
	 */
	MotionLimit operator*(double rhs);
	/**
	 * @brief Divides the max velocities and max acceleration by a divisor
	 * 
	 * @param rhs The divisor
	 * @return The new MotionLimit with the quotients
	 */
	MotionLimit operator/(double rhs);
};

/**
 * @brief Different points that a profile needs to reach
 */
struct MotionPoint {
	MotionLimit motionLimit;
	double distance;

	/**
	 * @brief Multiplies the max velocitie, max acceleration, and distance by a factor
	 * 
	 * @param rhs The factor
	 * @return The new MotionPoint with the products
	 */
	MotionPoint operator*(double rhs);
	/**
	 * @brief Divides the max velocitie, max acceleration, and distance by a divisor
	 * 
	 * @param rhs The divisor
	 * @return The new MotionPoint with the quotients
	 */
	MotionPoint operator/(double rhs);
};

/**
 * @brief When the cutoffs begins and ends
 */
struct CutoffPoint {
	double distance;
	double velocity {0};
	double time {0};
};

class MotionProfiling {
	public:
	/**
	* @brief Generates a trapezoidal profile
	* 
	* @param motionLimit The max velocity and acceleration
	* @param start The starting position, speed, and time
	* @param end The ending position, speed, and time
	* @return A vector of velocities that the robot will have to move at to meet the requirements
	*/
	static MotionProfile generateFullTrapezoid(MotionLimit motionLimit, CutoffPoint start, CutoffPoint end);

	/**
	* @brief Generates a profile of the acceleration portion of a movement.
	*
	* @param motionLimit The max velocity and acceleration
	* @param start The starting poistion, speed, and time
	* @param end The ending position, speed, and time
	* @return A vector of the acceleration, with information such as the accel time and dist
	*/
	static AccelProfile generateAccelerationTrapezoid(MotionLimit motionLimit, CutoffPoint start, CutoffPoint end);
};
} // namespace lamaLib