#pragma once

#include "okapi/api.hpp"
#include "../subsystems/Sensors/Encoders/Encoder.hpp"
#include "../utilities/MathHelper.hpp"
#include "../utilities/Pose.hpp"
#include "pros/rtos.h"

using namespace std;

namespace lamaLib {
/**
 * @brief Some values used for odom for organization purposes.
 */
struct OdomValues {
    double left;
    double right;
    double rear;
    double theta;
};

/**
 * @brief Tracking wheels that are used in odom and their TPR
 */
struct Encoders {
    shared_ptr<Encoder> left;
    shared_ptr<Encoder> right;
    shared_ptr<Encoder> rear;
};

struct EncoderScales {
	double leftRadius;
	double rightRadius;
	double rearRadius;
	double leftWheelDiameter;
	double rightWheelDiameter;
	double rearWheelDiameter;

	double wheelTrack = leftRadius + rightRadius;

	const int tpr = 360;

	void operator=(EncoderScales rhs);
};

/**
 * @brief The encoder values of all 3 tracking wheels in ticks
 */
struct EncoderTicks {
    double left;
    double right;
    double rear;

    EncoderTicks operator+(EncoderTicks rhs);
    EncoderTicks operator-(EncoderTicks rhs);
};

class Odometry {
    public:
    /**
     * @brief Odometry to keep track of the robot's position
     */
    Odometry(Encoders encoders, EncoderScales scales);

    /**
     * @brief Calculates and updates the new position coordinates with the change in the encoders
     * 
     * @param ireadingsDiff The difference in the readings since the last update
     */
    void updatePose(EncoderTicks ireadingsDiff);

	/**
	 * @brief Gets the pose
	 * 
	 * @return The current pose of the robot
	 */
	Pose getPose();
	/**
	 * @brief Sets the pose
	 * 
	 * @param newPose The new pose of the robot
	 */
	void setPose(Pose newPose);

	/**
	 * @brief Gets the encoders
	 * 
	 * @return The tracking wheels
	 */
	Encoders getEncoders();
	/**
	 * @brief Sets the encoders
	 *
	 * @param newEncoders The new tracking wheels
	 */
	void setEncoders(Encoders newEncoders);

	/**
	 * @brief Gets the encoder scales
	 * 
	 * @return The measurements of the tracking wheels
	 */
	EncoderScales getEncoderScales();
	/**
	 * @brief Sets the encoder scales
	 * 
	 * @param newScales The new measurements of the tracking wheels
	 */
	void setEncoderScales(EncoderScales newScales);

	/**
	 * @brief Gets the encoder ticks
	 * 
	 * @return The encoder rotations in ticks
	 */
	EncoderTicks getEncoderTicks();

	/**
	 * @brief Starts the odom task
	 */
	void startOdom();
	/**
	 * @brief Ends the odom task
	 */
	void endOdom();

	private:
	Pose pose {0, 0, 0, 0};

	Encoders encoders;
	EncoderScales scales;

	pros::task_t odomTask;
};

/**
 * @brief The main odometry task
 */
void odometryMain(void* odometry);
} // namespace lamaLib
