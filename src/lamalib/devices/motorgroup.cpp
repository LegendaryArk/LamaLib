#include "motorgroup.hpp"

using namespace std;
using namespace lamaLib;

MotorGroup::MotorGroup(vector<shared_ptr<Motor>> motors) : motors(motors) {}

void MotorGroup::moveVelocity(int velocity) {
    for (shared_ptr<Motor> motor : motors)
        motor->moveVelocity(velocity);
}
void MotorGroup::moveMotor(int velocity, double m, double b, PIDGains pid) {
    for (shared_ptr<Motor> motor : motors)
        motor->moveMotor(velocity, m, b, pid);
}
void MotorGroup::moveVoltage(int voltage) {
    for (shared_ptr<Motor> motor : motors)
        motor->moveVoltage(voltage);
}

void MotorGroup::moveAbsolute(double position, double velocity) {
	for (shared_ptr<Motor> motor : motors)
		motor->moveAbsolute(position, velocity);
}
void MotorGroup::moveRelative(double position, double velocity) {
	for (shared_ptr<Motor> motor : motors)
		motor->moveRelative(position, velocity);
}

vector<shared_ptr<Motor>> MotorGroup::getMotors() {
    return motors;
}

double MotorGroup::getActualVelocity() {
    return motors.at(0)->getActualVelocity();
}

void MotorGroup::setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode) {
    for (shared_ptr<Motor> motor : motors)
        motor->setBrakeMode(brakeMode);
}
okapi::AbstractMotor::brakeMode MotorGroup::getBrakeMode() {
    return motors.at(0)->getBrakeMode();
}

bool MotorGroup::isOverTemp() {
    for (shared_ptr<Motor> motor : motors)
        if (motor->isOverTemp()) return true;
    return false;
}

void MotorGroup::setGearing(okapi::AbstractMotor::gearset gearset) {
    for (shared_ptr<Motor> motor : motors)
        motor->setGearing(gearset);
}
okapi::AbstractMotor::gearset MotorGroup::getGearing() {
    return motors.at(0)->getGearing();
}

double MotorGroup::getTicks() {
	double sum;
	for (shared_ptr<Motor> motor : motors)
		sum += motor->getTicks();
	return sum / motors.size();
}
void MotorGroup::resetZero() {
	for (shared_ptr<Motor> motor : motors)
		motor->resetZero();
}