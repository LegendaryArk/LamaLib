#include "api.h"
#include "../../utilities/Pose.hpp"

namespace lamaLib {
class GPS : public pros::GPS{
	public:
	/**
	* @brief Input the GPS sensor port number for the constructor.
	* 
	* @param port
	*/
	GPS(int port);
	GPS(pros::GPS gps);

	/**
	* @brief Initialize all the GPS sensor parameters.
	* 
	* @param startingX
	* @param startingY
	* @param startingHeading
	* @param offsetX
	* @param offsetY
	*/
	void gpsInitialize(Pose pose, int offsetX, int offsetY);

	/**
	* @brief Returns robot rotation.
	* 
	* @return The robot's orientation
	*/
	int getRotation();

	/**
	* @brief Returns GPS status (the coordinates of the GPS).
	* 
	* @return The coordinates of the robot
	*/
	Pose getPose();
};
} // namespace lamalib