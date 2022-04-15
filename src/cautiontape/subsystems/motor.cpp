#include "motor.hpp"

using namespace std;
using namespace lamaLib;

Motor::Motor(int8_t port) : okapi::Motor(port) {}

Motor::Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset, okapi::AbstractMotor::encoderUnits encoderUnits) :
okapi::Motor (port, reverse, igearset, encoderUnits) {
    Motor::setBrakeMode(AbstractMotor::brakeMode::brake);
}

int32_t Motor::moveVelocity(int16_t ivelocity) {
    double volt = (ivelocity - YINTERCEPT) / SLOPE;
    
    double signal = pidControl.calculatePID(getActualVelocity(), ivelocity, 2);
    moveVoltage(volt * signal);
    cout << volt << "\t" << signal << "\t" << getActualVelocity() << "\n";

    return 0;
}
void Motor::setVelocityPID(PIDValues ivelPID) {
    pidControl.updatePID(ivelPID);
}