#pragma once

#include "okapi/api.hpp"

namespace lamaLib {
/**
 * @brief The orientation of the inertial sensor along the x, y, and z axis, in degrees
 * roll, pitch, and yaw properties are the same as x, y, and z.
 */
struct Axes {
	double x;
	double y;
	double z;
	double& roll = x;
	double& pitch = y;
	double& yaw = z;

	Axes operator+(Axes rhs);
	Axes operator-(Axes rhs);
};

class Inertial {
	okapi::IMU roll;
	okapi::IMU pitch;
	okapi::IMU yaw;

	// bool calibrating = false;

	pros::task_t driftCompensationTask {};

	public:
	/**
	 * @brief The inertial sensor to keep track of the robot's orientation
	 * 
	 * @param port The port that the inertial sensor is connected to
	 */
	Inertial(int port);

	/**
	 * @brief Reset all the readings back to 0
	 */
	void reset();

	/**
	 * @brief Gets the angles or the orientations about each axes in degrees
	 * 
	 * When the upper bound and lower bound are not the same, then they will be used to bound the angles
	 *
	 * @param upperBound The maximum angle reading
	 * @param lowerBound The minimum angle reading
	 * @return The angles on each axes
	 */
	Axes get(double upperBound = 0, double lowerBound = 0);

	/**
	 * @brief Calibrates the inertial sensor.
	 * 
	 * Make sure to run this method while the robot is still and is not moving around. Once this method has started running, do not touch the robot until calibration is complete.
	 */
	void calibrate();
	/**
	 * @brief Returns whether the sensor is calibrating or not
	 * 
	 * @return true 
	 * @return false 
	 */
	bool isCalibrating();

	/**
	 * @brief Starts the drift correction task
	 */
	void startCorrection();
	/**
	 * @brief Ends the drift correction task
	 */
	void endCorrection();
};

/**
 * @brief The function that corrects the readings from the inertial sensor in the case of gyro drift
 * 
 * @param inertial Passing the inertial sensor into the function through pros tasks
 */
void driftCompensation(void* inertial);
} // namespace lamaLib