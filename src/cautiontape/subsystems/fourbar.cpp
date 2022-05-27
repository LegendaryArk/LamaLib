#include "FourBar.hpp"

using namespace lamaLib;

FourBar::FourBar(MotorGroup motors, double ratio, PIDGains pidValues) : motors(motors), pidController(pidValues, 12000, -12000) {
  	gearRatio = ratio;
}

void FourBar::initialize(double current) {
	currentAngle = current;
	offsetAngle = current;
}

void FourBar::startMove(double targetAngle) {
	double offsetEncoder = offsetAngle * 5 * gearRatio;
	target = targetAngle * 5 * gearRatio - offsetEncoder;
	moving = true;
}

void FourBar::endMove() {
	moving = false;
}

void FourBar::moveVelocity(int velocity) {
	motors.moveVelocity(velocity);
}

void FourBar::setBrakeMode(okapi::AbstractMotor::brakeMode brakeInput) {
  motors.setBrakeMode(brakeInput);
}

MotorGroup FourBar::getMotors() {
	return motors;
}
double FourBar::getEncoder() {
	return motors.getMotors()[0].getPosition();
}

double FourBar::getTarget() {
	return target;
}

PIDController FourBar::getPIDController() {
	return pidController;
}

bool FourBar::isMoving() {
	return moving;
}