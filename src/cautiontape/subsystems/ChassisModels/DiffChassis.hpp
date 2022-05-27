#pragma once

#include "pros/adi.hpp"
#include "../../controller/MotionProfiling.hpp"
#include "../../controller/Odometry.hpp"
#include "../../controller/PID.hpp"
#include "../FourBar.hpp"
#include "../Sensors/Inertial.hpp"
#include "../MotorGroup.hpp"
#include "../../utilities/MathHelper.hpp"
#include "../../utilities/Pose.hpp"
#include <cmath>
#include <map>
#include <string>

namespace lamaLib {
/**
 * @brief The measurements of the tracking wheels in inches
 * 
 * Left and right are separate in the case that they are different
 */
struct ChassisScales {
    double wheelDiameter;
	okapi::AbstractMotor::GearsetRatioPair gearset {okapi::AbstractMotor::gearset::green, 1};
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
     * @param wheelDiameter The wheel diameter of the chassis wheels
     * @param iencoders The encoders used for odom, also includes their TPR
     * @param igearRatio The external gear ratio of the chasssis
     * @param iinterval The interval between slew rate calculations
     */
    Chassis(MotorGroup leftMotors, MotorGroup rightMotors, ChassisScales chassisScales, Encoders encoders, EncoderScales encoderScales, int iinterval);

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
     * @param idistances The distances where cutoff is in m; the last one should be the total distance; accumulative
     * @param imaxes The different max velocities and max accelerations for each cutoff segment in m/s and m/s2 respectively
     * @param iends The different end velocities for each cutoff segment in m/s
     */
    void moveDistance(vector<double> idistances, vector<MotionLimit> imaxes, vector<double> iends);

    /**
     * @brief Turns the robot relative to the starting heading at the beginning of the program
     *
     * Uses odometry heading
     * 
     * @param itarget The target angle in deg
     * @param imaxVel The max velocity in m/s
     * @param pidVals The Kpk Ki, Kd, and Kf used for the PID; default is all 0
     */
    void turnAbsolute(double itarget, double imaxVel, PIDGains pidVals = {0, 0, 0, 0});

    /**
     * @brief Turns the robot relative to the current heading
     * 
     * @param itarget The target angle in deg
     * @param imaxVel The max velocity in m/s
     * @param pidVals The Kp, Ki, Kd and Kf used for the PID, default is all 0
     */
    void turnRelative(double itarget, double imaxVel, PIDGains pidVals = {0, 0, 0, 0});

    /**
     * @brief Turns the robot to face a given coordinate and moves to that point
     * 
     * @param itarget The target point/coordinate (x, y, theta)
     * @param turnVel The turn velocity in m/s
     * @param cutoffDists The distances where a cutoff happens, excluding the final point in m
     * @param imaxes The max velocities for each profiling segment in m/s and m/s2 respecitvely
     * @param iends The end velocities for each profiling segment in m/s
     * @param turnPID The Kp, Ki, Kd, and Kf used in the turn, default is all 0
     * @param reverse Whether the robot should move forward or backwards to the point. True = backwards, false = forwards
     */
    void moveToPose(Pose itarget, double turnVel, vector<Pose> cutoffPoses, vector<MotionLimit> imaxes, vector<double> iends, PIDGains turnPID = {0, 0, 0, 0}, bool reverse = false, bool angleWrap = false);

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

    void setBrakeMode(okapi::AbstractMotor::brakeMode ibrakeMode);
    okapi::AbstractMotor::brakeMode getBrakeMode();

    /**
     * @brief Get the chassis scales of the robot
     * 
     * @return The wheel diameter and gear ratio of the robot
     */
    ChassisScales getChassisScales();
    /**
     * @brief Set the chassis scales of the robot
     * 
     * @param iscales The new wheel diameter and gear ratio of the robot
     */
    void setChassisScales(ChassisScales iscales);

    EncoderScales calibrateWheelDiameter(pros::Controller controller, double actualDist);

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
     * @param inertial Used to do the initial turn
     * @return The new measurements; the only updated should be the radii; the measurements should already be updated
     */
    EncoderScales calibrateChassisDiameter(pros::Controller controller, Inertial inertial);

    /**
     * @brief Pass in PID values for vision sensor tracking and movement.
     * 
     * @param track 
     * @param move 
     * @param width
     */
    void setVisionPID(PIDGains track, PIDGains move, PIDGains width);

    /**
     * @brief Turn to an object, move to a certain width. Returns true when the object is within desired range.
     * 
     * @param directionalTarget 
     * @param widthTarget 
     * @param moveSpeed 
     * @param turnSpeed 
     * @param widthMin 
     * @param width
     * @return 1 Turning to target
     * @return 2 Moving to target
     * @return 3 Target reached
     * @return 4 Target too small
     */
    int moveToVision(int directionalTarget, int widthTarget, int moveSpeed, int turnSpeed, int widthMin, int width);
        //maybe add a sort of a failsafe for if the goal gets snatched

    /**
     * @brief Calculates slew rate for left motor inputs
     * WARNING, ALWAYS RUN rcalcSlew AND lcalcSlew IN THE SAME LOOP OR ELSE THE 
     * INTERVAL WON't UPDATE PROPERLY
     * 
     * @param itarget The controller input that is fed into the slew rate
     * @param istep The step in between motor inputs
     *
     * @return Motor input with slew
     */
    int lCalcSlew(int itarget, int istep);

    /**
     * @brief Calculates slew rate for right motor inputs
     * WARNING, ALWAYS RUN rcalcSlew AND lcalcSlew IN THE SAME LOOP OR ELSE THE 
     * INTERVAL WON't UPDATE PROPERLY
     * 
     * @param itarget The controller input that is fed into the slew rate
     * @param istep The step in between motor inputs
     *
     * @return Motor input with slew
     */
    int rCalcSlew(int itarget, int istep);

    /**
     * @brief Starts the odometry task
     */
    void startOdom();
    /**
     * @brief Ends the odometry task
     */
    void endOdom();
    vector<FourBar> fourBarList;
    void addFourBar(MotorGroup motors, double gearRatio, PIDGains pidValues);
    
    private:
    int counter;
    int interval;
    int previousOutputL = 0;
    int previousOutputR = 0;

    PIDController visionTrackPID {{0, 0, 0, 0}};
    PIDController visionMovePID {{0, 0, 0, 0}};
    PIDController visionWidthPID {{0, 0, 0, 0}};
    
	MotorGroup leftMotors;
    MotorGroup rightMotors;

    okapi::AbstractMotor::GearsetRatioPair gearset {okapi::AbstractMotor::gearset::green, 1};

    Odometry odom;

	ChassisScales chassisScales;

    pros::task_t fbTsk {};
};

extern Chassis chassis;

/**
 * @brief The main odometry loop
 * 
 * @param iparam 
 */
void odometryMain(void *iparam);
void fourBarTask(void *iparam);
} // namespace lamaLib