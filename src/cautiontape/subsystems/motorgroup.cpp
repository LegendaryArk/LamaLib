#include "motorgroup.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"

using namespace std;
using namespace lamaLib;

MotorGroup::MotorGroup(vector<Motor> imotors) : motors(imotors) {}

void MotorGroup::moveVelocity(int ivel) {
    for (Motor motor : motors)
        motor.moveVelocity(ivel);
}
void MotorGroup::moveVoltage(int ivolt) {
    for (Motor motor : motors)
        motor.moveVoltage(ivolt);
}

double MotorGroup::getActualVelocity() {
    return motors.at(0).getActualVelocity();
}

void MotorGroup::setBrakeMode(okapi::AbstractMotor::brakeMode ibrakeMode) {
    for (Motor motor : motors)
        motor.setBrakeMode(ibrakeMode);
}
okapi::AbstractMotor::brakeMode MotorGroup::getBrakeMode() {
    return motors.at(0).getBrakeMode();
}

void MotorGroup::setGearing(okapi::AbstractMotor::gearset igearset) {
    for (Motor motor : motors)
        motor.setGearing(igearset);
}
okapi::AbstractMotor::gearset MotorGroup::getGearing() {
    return motors.at(0).getGearing();
}