#pragma once

#include "pros/adi.hpp"
#include "motorgroup.hpp"
#include "../controller/motionprofiling.hpp"
#include "../controller/odometry.hpp"
#include "../controller/pid.hpp"
#include "../subsystems/inertial.hpp"
#include "../utilities/chassisdata.hpp"
#include "../utilities/mathhelper.hpp"
#include "../utilities/pose.hpp"

namespace lamaLib {
/**
 * @brief The encoder values of all 3 tracking wheels in ticks
 */
struct EncoderValues {
    double left;
    double right;
    double rear;
};

/**
 * @brief Used for controller control. Allows for more control, such as more or less sensitivity
 *
 * 0-100 in rpm
 */
static int joyMap[128] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    31, 31, 31, 31, 31, 31, 31, 31, 31, 31,
    31, 31, 31, 31, 31, 31, 31, 31, 31, 31,
    39, 40, 41, 42, 43, 44, 44, 45, 46, 47,
    48, 49, 50, 50, 50, 51, 51, 51, 52, 66,
    78, 78, 78, 78, 78, 78, 78, 78, 78, 78,
    86, 86, 86, 86, 86, 86, 86, 86, 86, 86,
    94, 94, 95, 96, 97, 98, 99, 100,100,100,
    100,100,100,100,100,100,100,100
};

class Chassis {
    public:
    /**
     * @brief Controls the chassis motors
     * 
     * @param ileftMotors Left motors
     * @param irightMotors Right motors
     * @param igearRatio The external gear ratio of the chasssis
     */
    Chassis(MotorGroup ileftMotors, MotorGroup irightMotors, double wheelCircumference, double igearRatio = 1);

    /**
     * @brief Moves the left and right motors when using the controller. The power that is given to the motors are according to the joyMap
     * 
     * @param left Left joycon reading (-127 to 127)
     * @param right Right joycon reading (-127 to 127)
     */
    void move(int ileft, int iright);

    /**
     * @brief Moves the robot forward a certain distance in meters.
     *
     * Has cutoff implemented
     * 
     * @param idistances The distances where cutoff is; the last one should be the total distance; accumulative
     * @param imaxes The different max velocities and max accelerations for each cutoff segment
     * @param iends The different end velocities for each cutoff segment
     */
    void moveDistance(vector<double> idistances, vector<MotionLimit> imaxes, vector<double> iends);

    /**
     * @brief Turns the robot relative to the starting heading at the beginning of the program
     *
     * Uses odometry heading
     * 
     * @param itarget The target angle
     * @param imaxVel The max velocity
     * @param kp The Kp used for the PID; default is 0
     * @param ki The Ki used for the PID; default is 0
     * @param kd The Kd used for the PID; default is 0
     * @param kf The Kf used for the PID; default is 0
     */
    void turnAbsolute(double itarget, double imaxVel, double kp = 0, double ki = 0, double kd = 0, double kf = 0);

    /**
     * @brief Turns the robot relative to the current heading
     * 
     * @param itarget The target angle
     * @param imaxVel The max velocity
     * @param kp The Kp used for the PID; default is 0
     * @param ki The Ki used for the PID; default is 0
     * @param kd The Kd used for the PID; default is 0
     * @param kf The Kf used for the PID; default is 0
     */
    void turnRelative(double itarget, double imaxVel, double kp = 0, double ki = 0, double kd = 0, double kf = 0);

    /**
     * @brief Turns the robot to face a given coordinate and moves to that point
     * 
     * @param itarget The target point/coordinate
     * @param turnVel The turn velocity
     * @param cutoffDists The distances where a cutoff happens, excluding the final point
     * @param imaxes The max velocities for each profiling segment
     * @param iends The end velocities for each profiling segment
     * @param reverse Whether the robot should move forward or backwards to the point. True = backwards, false = forwards
     */
    void moveToPose(Pose itarget, double turnVel, vector<double> cutoffDists, vector<MotionLimit> imaxes, vector<double> iends, bool reverse = false);

    /**
     * @brief Gets the left motors
     * 
     * @return The left motors
     */
    MotorGroup getLeftMotors();
    /**
     * @brief Gets the right motors
     * 
     * @return The right motors 
     */
    MotorGroup getRightMotors();

    /**
     * @brief Gets the tracking wheels
     * 
     * @return The tracking wheels that are used in odom
     */
    Encoders getTrackingWheels();
    /**
     * @brief Get the encoder tick counts of the tracking wheels
     * 
     * @return The encoder readings
     */
    EncoderValues getEncoders();


    // Move get and set pose to private and then create a setStartPose for setting the start position
    /**
     * @brief Get the current coordinates
     * 
     * @return The current position coordinates
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
     * @return The tracking wheel measurements
     */
    RobotScales getScales();
    /**
     * @brief Set the tracking wheel measurements with new measurements
     * 
     * @param iscales The new measurements of the tracking wheel
     */
    void setScales(RobotScales iscales);

    /**
     * @brief Calibrate the distance from the tracking wheel to the center of the robot
     *
     * The robot will first turn 10 circles using the inertial sensor. Afterwards, you are to turn the robot yourself to face
     * 0 degrees (turn the shortest direction). Once that is done, press 'A' on the controller to find the calibrated values
     * outputted onto the lcd, as well as the terminal
     *
     * This process should only be done once unless the tracking wheel positions have changed
     * 
     * @param controller Used to determine when the calculations should be done; when the robot is in the correct orientation
     * @param iinertial Used to do the initial turn
     * @return The new measurements; the only updated should be the radii 
     */
    RobotScales calibrateOdom(pros::Controller controller, Inertial iinertial);

    /**
     * @brief Starts the odometry task
     */
    void startOdom();
    /**
     * @brief Ends the odometry task
     */
    void endOdom();
    
    private:
    lamaLib::MotorGroup leftMotors;
    lamaLib::MotorGroup rightMotors;

    double wheelCircumference;

    okapi::AbstractMotor::GearsetRatioPair gearset {okapi::AbstractMotor::gearset::green, 1};

    /**
     * 
     * Info used for odometry
     * 
     */
    Pose pose {0, 0, 0, 0};

    Encoders encoders;
    RobotScales scales;

    pros::task_t odomTask {};
};

extern Chassis chassis;

/**
 * @brief The main odometry loop
 * 
 * @param iparam 
 */
void odometryMain(void *iparam);
} // namespace lamaLib