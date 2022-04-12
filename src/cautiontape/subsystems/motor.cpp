#include "motor.hpp"

using namespace lamaLib;

Motor::Motor(int8_t port) : okapi::Motor(port) {}

Motor::Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset, okapi::AbstractMotor::encoderUnits encoderUnits) :
okapi::Motor (port, reverse, igearset, encoderUnits) {
    Motor::setBrakeMode(AbstractMotor::brakeMode::brake);
}

int32_t Motor::moveVelocity(int16_t ivelocity) {
    PIDController pidControl(velKp, velKi, velKd, velKf, 1);

    while (fabs(getActualVelocity() - ivelocity) < 2) {
        double signal = pidControl.calculatePID(getActualVelocity(), ivelocity, 2);
        double volt = (ivelocity - YINTERCEPT) / SLOPE;

        moveVoltage(volt * signal);

        pros::delay(20);
    }

    moveVoltage(0);
    return 0;
}
void Motor::setVelocityPID(double kp, double ki, double kd, double kf) {
    velKp = kp;
    velKi = ki;
    velKd = kd;
    velKf = kf;
}