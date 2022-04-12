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

    int32_t moveVelocity(int16_t ivelocity) override;
    void setVelocityPID(double kp, double ki, double kd, double kf);

    private:
    double velKp;
    double velKi;
    double velKd;
    double velKf;
};
}