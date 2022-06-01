#include "api.h"
#include "../../utilities/Pose.hpp"

namespace lamaLib {
class GPS : public pros::Gps{
	public:
	/**
	* @brief The GPS sensor is an absolute positioning system using the GPS tape across the field walls
	* 
	* @param port The port the GPS sensor is connected to
	* @param initialPose The starting position of the robot
	* @param offsetX The offset of the gps sensor from the center of rotation of the robot along the x axis
	* @param offsetY The offset of the gps sensor from the center of rotation of the robot along the y axis
	*/
	GPS(int port, Pose initialPose = {0, 0, 0}, double offsetX = 0, double offsetY = 0);
	/**
	 * @brief The GPS sensor is an absolute positioning system using the GPS tape across the field walls
	 * 
	 * @param gps The pros gps sensor
	 */
	GPS(pros::Gps gps);

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