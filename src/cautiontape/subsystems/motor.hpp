#pragma once

#include "okapi/api.hpp"
#include "../controller/pid.hpp"

#define YINTERCEPT -19.191
#define SLOPE 0.0189

namespace lamaLib {

class Motor: public okapi::Motor {
    public:
    Motor(int8_t port);
    Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset, okapi::AbstractMotor::encoderUnits encoderUnits = AbstractMotor::encoderUnits::degrees);
    Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset, PIDValues pidVals, okapi::AbstractMotor::encoderUnits encoderUnits = AbstractMotor::encoderUnits::degrees);

    /**
     * @brief Moves the robot at a certain velocity with a pid
     * 
     * @param ivelocity The desired velocity
     */
    void moveVelocity(int ivelocity, double slope, double yIntercept);
    /**
     * @brief Sets the PID values for moveVelocity
     * 
     * @param velPID The kp, ki, kd, and kf; default is 0
     */
    void setVelocityPID(PIDValues ivelPID);

    private:
    PIDController pidControl {{0.001, 0, 0, 0}, 1};
};
} // namespace lamaLib