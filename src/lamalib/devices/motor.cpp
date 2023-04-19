#include "motor.hpp"

using namespace std;
using namespace lamaLib;

Motor::Motor(int8_t port) : okapi::Motor(port) {
	setBrakeMode(AbstractMotor::brakeMode::brake);

	ticksPerRotation = getGearing() == AbstractMotor::gearset::blue ? 300 : // Blue gearset
						getGearing() == AbstractMotor::gearset::green ? 900 : // Green gearset
																		1800; // Red gearset
}
Motor::Motor(int8_t port, bool isReversed, okapi::AbstractMotor::gearset gearset, okapi::AbstractMotor::encoderUnits encoderUnits)
			: okapi::Motor (port, isReversed, gearset, encoderUnits) {
	setBrakeMode(AbstractMotor::brakeMode::brake);

	ticksPerRotation = getGearing() == AbstractMotor::gearset::blue ? 300 : // Blue gearset
						getGearing() == AbstractMotor::gearset::green ? 900 : // Green gearset
																		1800; // Red gearset
}

void Motor::moveMotor(int velocity, double m, double b, PIDGains pid) {
	double voltage = m * velocity + b;

	PIDController pidControl(pid, 2, 0, 10, 2);
	double signal = pidControl.calculatePID(getActualVelocity(), velocity);

	if (abs(velocity) < 5) {
		voltage = 0;
		signal = 0;
	}

	moveVoltage(voltage * signal * sign(velocity));
	// cout << signal << "," << getActualVelocity() << "\n";
	// cout << volt << "\t" << signal << "\t" << getActualVelocity() << "\n";
}

double Motor::getTicks() {
	return getEncoder()->get();
}
void Motor::resetZero() {
	getEncoder()->reset();
}