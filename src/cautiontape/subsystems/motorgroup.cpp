#include "motorgroup.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"

using namespace lamaLib;

MotorGroup::MotorGroup(std::vector<Motor> motors) : motors(motors) {}

void MotorGroup::moveVelocity(int ivel) {
    for (Motor motor : motors)
        motor.moveVelocity(ivel);
}
void MotorGroup::moveVoltage(int ivolt) {
    for (Motor motor : motors)
        motor.moveVoltage(ivolt);
}

MotorVels MotorGroup::getActualVelocity() {
    MotorVels motorVels;
    for (int i = 0; i < motors.size(); i++)
        motorVels.motorVels[i] = motors.at(i).getActualVelocity();
    return motorVels;
}

void MotorGroup::setBrakeMode(okapi::AbstractMotor::brakeMode ibrakeMode) {
    for (Motor motor : motors)
        motor.setBrakeMode(ibrakeMode);
}
okapi::AbstractMotor::brakeMode MotorGroup::getBrakeMode() {
    return motors.at(0).getBrakeMode();
}