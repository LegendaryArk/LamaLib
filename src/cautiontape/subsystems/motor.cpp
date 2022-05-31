#include "Motor.hpp"

using namespace std;
using namespace lamaLib;

Motor::Motor(int8_t port) : okapi::Motor(port) {}
Motor::Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset, okapi::AbstractMotor::encoderUnits encoderUnits) :
okapi::Motor (port, reverse, igearset, encoderUnits) {
    Motor::setBrakeMode(AbstractMotor::brakeMode::brake);
}

void Motor::moveMotor(int ivelocity, double slope, double yIntercept, PIDGains pid) {
    double volt = (abs(ivelocity) - yIntercept) / slope;

    PIDController pidControl(pid, 2, 0);
    double signal = pidControl.calculatePID(getActualVelocity(), ivelocity, 2);

    if (abs(ivelocity) < 5) {
        volt = 0;
        signal = 0;
    }

    moveVoltage(volt * signal * sign(ivelocity));
    // cout << signal << "," << getActualVelocity() << "\n";
    // cout << volt << "\t" << signal << "\t" << getActualVelocity() << "\n";
}

double Motor::getTicks() {
	return getEncoder()->get();
}