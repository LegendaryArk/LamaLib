#pragma once

#include "api.h"
#include "../subsystems/chassis.hpp"
#include "../utilities/mathHelper.hpp"

namespace lamaLib {
/**
 * @brief The coordinate position of the robot in inches and degrees
 *
 * Includes the time of the position
 */
typedef struct {
    double x;
    double y;
    double theta;
    uint time;
} Pose;

/**
 * @brief The encoder values of all 3 tracking wheels in ticks
 */
typedef struct {
    double left;
    double right;
    double rear;
    double theta;
} EncoderValues;

/**
 * @brief The measurements of the tracking wheels in inches
 * 
 * Left and right are separate in the case that they are different
 */
typedef struct {
    double wheelDiameter;
    double leftRadius;
    double rightRadius;
    double rearRadius;
} OdomScales;

class Odometry {
    public:
    /**
     * @brief Odometry to keep track of the robot's position
     * 
     * @param leftEncoder The left encoder sensor
     * @param rightEncoder The right encoder sensor
     * @param rearEncoder The rear encoder sensor
     * @param scales The measurements of the tracking wheels in inches
     */
    Odometry(pros::ADIEncoder leftEncoder, pros::ADIEncoder rightEncoder, pros::ADIEncoder rearEncoder, OdomScales scales, int tpr);

    /**
     * @brief Get the left encoder tick counts
     * 
     * @return int 
     */
    EncoderValues getEncoders();

    // Move get and set pose to private and then create a setStartPose for setting the start position
    /**
     * @brief Get the current coordinates
     * 
     * @return Pose 
     */
    Pose getPose();
    /**
     * @brief Set the pose to a new coordinate position
     * 
     * @param ipose The new coordinate position
     */
    void setPose(Pose ipose);

    /**
     * @brief Get the current tracking wheel measurements
     * 
     * @return OdomScales 
     */
    OdomScales getScales();
    /**
     * @brief Set the tracking wheel measurements with new measurements
     * 
     * @param iscales the new measurements of the tracking wheel
     */
    void setScales(OdomScales iscales);

    /**
     * @brief Calibrate the distance from the tracking wheel to the center of the robot
     *
     * The robot will first turn 10 circles using the inertial sensor. Afterwards, you are to turn the robot yourself to face
     * 0 degrees (turn the shortest direction). Once that is done, press 'A' on the controller to find the calibrated values
     * outputted onto the lcd, as well as the terminal
     *
     * This process should only be done once unless the tracking wheel positions have changed
     * 
     * @param ichassis Used to turn the robot
     * @param controller Used to determine when the calculations should be done; when the robot is in the correct orientation
     * @param iinertial Used to do the initial turn
     * @return OdomScales 
     */
    OdomScales calibrate(Chassis ichassis, pros::Controller controller, pros::IMU iinertial);

    /**
     * @brief Starts the odometry task
     */
    void startOdom();
    /**
     * @brief Ends the odometry task
     */
    void endOdom();

    int tpr;

    private:
    pros::ADIEncoder leftEncoder;
    pros::ADIEncoder rightEncoder;
    pros::ADIEncoder rearEncoder;

    Pose pose;
    OdomScales scales;

    pros::task_t odomTask {};
};

extern Odometry odom;

/**
 * @brief Main function of odometry
 */
void odometryMain(void* iparam);
} // namespace lamaLib