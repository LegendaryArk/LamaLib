#pragma once

#include "okapi/api.hpp"
#include "../controller/PID.hpp"
#include "../utilities/MathHelper.hpp"

#define YINTERCEPT -19.191
#define SLOPE 0.0189

namespace lamaLib {

class Motor: public okapi::Motor {
    public:
    Motor(int8_t port);
    Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset, okapi::AbstractMotor::encoderUnits encoderUnits = AbstractMotor::encoderUnits::degrees);

    /**
     * @brief Moves the robot at a certain velocity with a pid
     * 
     * @param ivelocity The desired velocity
     */
    void moveMotor(int ivelocity, double slope = 1, double yIntercept = 0, PIDGains pid = {0, 0, 0, 1});
    /**
     * @brief Sets the PID values for moveVelocity
     * 
     * @param velPID The kp, ki, kd, and kf; default is 0
     */
    void setVelocityPID(PIDGains ivelPID);
};
} // namespace lamaLib
