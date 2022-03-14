#include "motor.hpp"

using namespace lamaLib;

Motor::Motor(int8_t port) : okapi::Motor(port) {
    
}

Motor::Motor(int8_t port, bool reverse, AbstractMotor::gearset igearset,
AbstractMotor::encoderUnits encoderUnits) :
okapi::Motor (
    port, reverse, igearset, encoderUnits
)
{
    Motor::setBrakeMode(AbstractMotor::brakeMode::brake);
}