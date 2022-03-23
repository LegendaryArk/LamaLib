#pragma once

#include "motor.hpp"

typedef struct {
    double motorVels[];
} MotorVels;

namespace lamaLib {
class MotorGroup {
    public:
    MotorGroup(std::vector<Motor> motors);

    void moveVelocity(int ivel);
    void moveVoltage(int ivolt);

    MotorVels getActualVelocity();

    void setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode);
    okapi::AbstractMotor::brakeMode getBrakeMode();

    private:
    std::vector<Motor> motors;
};
} // namespace lamaLib