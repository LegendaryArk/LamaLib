#include "motor.hpp"

using namespace std;
using namespace lamaLib;

Motor::Motor(int8_t port) : okapi::Motor(port) {}
Motor::Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset, okapi::AbstractMotor::encoderUnits encoderUnits) :
okapi::Motor (port, reverse, igearset, encoderUnits) {
    Motor::setBrakeMode(AbstractMotor::brakeMode::brake);
}
Motor::Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset, PIDValues pidVals, okapi::AbstractMotor::encoderUnits encoderUnits) :
okapi::Motor (port, reverse, igearset, encoderUnits), pidControl(pidVals) {
    Motor::setBrakeMode(AbstractMotor::brakeMode::brake);
}

void Motor::moveMotor(int ivelocity, double slope, double yIntercept) {
    double volt = (ivelocity - yIntercept) / slope;
    double signal = pidControl.calculatePID(getActualVelocity(), ivelocity, 2);

    if (ivelocity < 10) {
        volt = 0;
        signal = 0;
    }

    moveVoltage(volt + signal);
    // cout << volt << "\t" << signal << "\t" << getActualVelocity() << "\n";
}
void Motor::setVelocityPID(PIDValues ivelPID) {
    pidControl.setPID(ivelPID);
}