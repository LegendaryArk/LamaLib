#pragma once

#include "okapi/api.hpp"
#include "../devices/sensors/encoders/encoder.hpp"
#include "../devices/sensors/inertial.hpp"
#include "../utilities/mathhelper.hpp"
#include "../utilities/pose.hpp"

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
    std::shared_ptr<Encoder> left;
    std::shared_ptr<Encoder> right;
    std::shared_ptr<Encoder> rear;

	int tpr = left->getTicksPerRotation();
};

/**
 * @brief The measurements of the tracking wheels in inches
 * 
 * Left and right are separate in the case that they are different
 */
struct EncoderScales {
	double leftRadius;
	double rightRadius;
	double rearRadius;
	double leftWheelDiameter;
	double rightWheelDiameter;
	double rearWheelDiameter;

	double wheelTrack = leftRadius + rightRadius;

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
	EncoderTicks operator*(EncoderTicks rhs);
	EncoderTicks operator/(EncoderTicks rhs);
	EncoderTicks operator*(double factor);
	EncoderTicks operator/(double dividend);

	EncoderTicks toDistance(EncoderScales scales, int ticksPerRotation);
};

class Odometry {
	Pose pose {0, 0, 0, 0};

	Encoders encoders {nullptr, nullptr, nullptr};
	EncoderScales scales {0, 0, 0, 0, 0, 0};

	bool usingInertial {false};
	std::shared_ptr<Inertial> inertial;

	bool calibrating {false};
	EncoderTicks calibrationStart {0, 0, 0};

	pros::task_t odometryTask;
	
    public:
    /**
     * @brief Odometry to keep track of the robot's position
     */
	Odometry();
    Odometry(Encoders encoders, EncoderScales scales);
	Odometry(Encoders encoders, EncoderScales scales, std::shared_ptr<Inertial> inertial);

    /**
     * @brief Calculates and updates the new position coordinates with the change in the encoders
     * 
     * @param deltaReadings The difference in the readings since the last update
     */
    void updatePose(EncoderTicks deltaReadings);

	/**
	 * @brief Gets the pose
	 * 
	 * @return The current pose of the robot
	 */
	Pose getPose();
	/**
	 * @brief Sets the pose
	 * 
	 * @param pose The new pose of the robot
	 */
	void setPose(Pose pose);

	/**
	 * @brief Gets the encoders
	 * 
	 * @return The tracking wheels
	 */
	Encoders getEncoders();
	/**
	 * @brief Sets the encoders
	 *
	 * @param encoders The new tracking wheels
	 */
	void setEncoders(Encoders encoders);
	void resetEncoders();

	/**
	 * @brief Gets the encoder scales
	 * 
	 * @return The measurements of the tracking wheels
	 */
	EncoderScales getEncoderScales();
	/**
	 * @brief Sets the encoder scales
	 * 
	 * @param scales The new measurements of the tracking wheels
	 */
	void setEncoderScales(EncoderScales scales);

	/**
	 * @brief Gets the encoder ticks
	 * 
	 * @return The encoder rotations in ticks
	 */
	EncoderTicks getEncoderTicks();

	/**
	 * @brief Saves the current pose as the start of calibration.
	 * If not run before calibrating, it will assume (0, 0), 0 is the starting point for calibration
	 */
	void startCalibration();
	/**
	 * @brief Prints the number of ticks to the screen
	 * Generally used for calibration
	 */
	void printTicks();
	/**
	 * @brief Calibrates the new wheel diameters of the tracking wheels
	 * After the calculations, it will print the new wheel diameter to the terminal and to the lcd
	 * 
	 * @param actualDist The distance that the robot travelled (measured, not calculated)
	 * @param ticksTravelled The number of ticks each encoder has spun
	 */
	void calibrateWheelDiameter(double actualDist, EncoderTicks ticksTravelled);
	/**
	 * @brief Calibrates the new chassis diameter of the tracking wheels
	 * After the calculations, it will print the new chassis diameter to the terminal and to the lcd
	 * 
	 * @param actualAng The number of degrees that the robot turned (measured, not calculated)
	 * @param ticksTravelled The number of ticks each encoder has spun
	 */
	void calibrateChassisDiameter(double actualAng, EncoderTicks ticksTravelled);

	/**
	 * @brief Whether startCalib() has been called
	 * 
	 * @return true 
	 * @return false 
	 */
	bool isCalibrating();

	/**
	 * @brief Whether the current odometry object is using an inertial sensor
	 * 
	 * @return true 
	 * @return false 
	 */
	bool isUsingInertial();
	/**
	 * @brief Gets the inertial sensor
	 * 
	 * @return A shared pointer to the inertial sensor
	 */
	std::shared_ptr<Inertial> getInertial();

	/**
	 * @brief Starts the odom task
	 */
	void startOdometry();
	/**
	 * @brief Ends the odom task
	 */
	void endOdometry();
};

/**
 * @brief The main odometry task
 */
void odometryMain(void* odometry);
} // namespace lamaLib