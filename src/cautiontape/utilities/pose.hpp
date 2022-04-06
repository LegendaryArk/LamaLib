#pragma once

#include "api.h"

namespace lamaLib {
/**
* @brief The coordinate position of the robot in inches and degrees
*
* Includes the time of the position
*/
struct Pose {
	public:
	double x;
	double y;
	double theta;
	uint time;

	/**
	* @brief Finds the distance between 2 points
	* 
	* @param ipoint The second point from the current point
	* @return double 
	*/
	double distTo(Pose ipoint);
	/**
	* @brief Finds the angle between the 2 points
	* 
	* @param ipoint The second point that the first point will face towards
	* @return double 
	*/
	double angleTo(Pose ipoint);

	Pose operator+(Pose rhs);
	Pose operator-(Pose rhs);
};
} // namespace lamaLib