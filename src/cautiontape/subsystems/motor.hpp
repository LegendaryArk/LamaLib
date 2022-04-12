#pragma once

#include "okapi/api.hpp"
#include "../controller/pid.hpp"

#define YINTERCEPT -8.0757575
#define SLOPE 0.0197937062

using namespace std;

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
     * @param kp The Kp that is used for proportional
     * @param ki The Ki that is used for integral
     * @param kd The Kd that is used for derivative
     * @param kf The Kf that is used for feed forward
     */
    void setVelocityPID(double kp, double ki, double kd, double kf);

    private:
    double velKp {0};
    double velKi {0};
    double velKd {0};
    double velKf {0};
};
} // namespace lamaLib