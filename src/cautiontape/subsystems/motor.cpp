#include "motor.hpp"

using namespace lamaLib;

Motor::Motor(int8_t port) : okapi::Motor(port) {
    
}

Motor::Motor(int8_t port, bool reverse, okapi::AbstractMotor::gearset igearset,
okapi::AbstractMotor::encoderUnits encoderUnits) :
okapi::Motor (
    port, reverse, igearset, encoderUnits
)
{
    Motor::setBrakeMode(AbstractMotor::brakeMode::brake);
}