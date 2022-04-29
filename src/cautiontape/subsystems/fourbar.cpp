#include "fourbar.hpp"
#include "..\controller\pid.hpp"

using namespace lamaLib;

FourBar::FourBar(MotorGroup motors, double ratio, PIDValues pidValues) : 
motors({motors}), pidController(pidValues, 12000, -12000) { 
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
  std::cout << "AAAAAAAAAAAAAAAAAAA\n";
}

void FourBar::endMove() {
  moving = false;
}

void FourBar::moveVelocity(int velocity) {
  motors.moveVelocity(velocity);
}

double FourBar::getEncoder() {
  return motors.getMotors()[0].getPosition();
}

void FourBar::setBrakeMode(okapi::AbstractMotor::brakeMode brakeInput) {
  motors.setBrakeMode(brakeInput);
}