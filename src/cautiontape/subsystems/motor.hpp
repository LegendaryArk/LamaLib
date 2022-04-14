#pragma once

#include "okapi/api.hpp"
#include "../controller/pid.hpp"

#define YINTERCEPT -8.0757575
#define SLOPE 0.0197937062

namespace lamaLib {

class Motor: public okapi::Motor {
    public:
    Motor(int8_t port);
    Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset, okapi::AbstractMotor::encoderUnits encoderUnits = AbstractMotor::encoderUnits::degrees);

    /**
     * @brief Moves the robot at a certain velocity with a pid
     * 
     * @param ivelocity The desired velocity
     * @return Should be 0, just used for override
     */
    int32_t moveVelocity(int16_t ivelocity) override;
    /**
     * @brief Sets the PID values for moveVelocity
     * 
     * @param velPID The kp, ki, kd, and kf; default is 0
     */
    void setVelocityPID(PIDValues velPID);

    private:
    PIDValues velPID {0, 0, 0, 0};
};
} // namespace lamaLib