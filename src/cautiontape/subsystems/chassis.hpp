#pragma once

#include "okapi/api.hpp"
#include "motorgroup.hpp"
#include "../controller/motionprofiling.hpp"
#include "../controller/pid.hpp"
#include "../utilities/pose.hpp"
#include "../utilities/mathHelper.hpp"

namespace lamaLib {

/**
 * @brief Used for controller control. Allows for more control, such as more or less sensitivity
 */
static int joyMap[128] = { //joymap goes from 0 to 100
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
    Chassis(MotorGroup ileftMotors, MotorGroup irightMotors, double igearRatio = 1);

    /**
     * @brief Moves the left and right motors when using the controller. The power that is given to the motors are according to the joyMap
     * 
     * @param left Left joycon reading (-127 to 127)
     * @param right Right joycon reading (-127 to 127)
     */
    void move(int ileft, int iright);

    void moveDistance(vector<Pose> itargets, vector<MotionLimit> imotionLimit);

    /**
     * @brief Gets the left motors
     * 
     * @return MotorGroup 
     */
    MotorGroup getLeftMotors();
    /**
     * @brief Gets the right motors
     * 
     * @return MotorGroup 
     */
    MotorGroup getRightMotors();
    
    private:
    lamaLib::MotorGroup leftMotors;
    lamaLib::MotorGroup rightMotors;
    okapi::AbstractMotor::GearsetRatioPair gearset {okapi::AbstractMotor::gearset::green, 1};
};
} // namespace lamaLib