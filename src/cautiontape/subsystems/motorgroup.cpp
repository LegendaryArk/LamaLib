#include "motorgroup.hpp"

using namespace std;
using namespace lamaLib;

MotorGroup::MotorGroup(vector<Motor> imotors) : motors(imotors) {}

void MotorGroup::moveVelocity(int ivel) {
    for (Motor motor : motors)
        motor.moveVelocity(ivel);
}
void MotorGroup::moveMotor(int ivel, double slope, double yIntercept, PIDValues pid) {
    for (Motor motor : motors)
        motor.moveMotor(ivel, slope, yIntercept, pid);
}
void MotorGroup::moveVoltage(int ivolt) {
    for (Motor motor : motors)
        motor.moveVoltage(ivolt);
}

vector<Motor> MotorGroup::getMotors() {
    return motors;
}

void MotorGroup::setVelocityPID(PIDValues velPID) {
    for (Motor motor : motors)
        motor.setVelocityPID(velPID);
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

bool MotorGroup::isOverTemp() {
    for (Motor motor : motors) {
        if (motor.isOverTemp())
            return true;
    }
    return false;
}

void MotorGroup::setGearing(okapi::AbstractMotor::gearset igearset) {
    for (Motor motor : motors)
        motor.setGearing(igearset);
}
okapi::AbstractMotor::gearset MotorGroup::getGearing() {
    return motors.at(0).getGearing();
}