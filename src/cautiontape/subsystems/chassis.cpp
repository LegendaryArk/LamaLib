#include "chassis.hpp"

using namespace lamaLib;

Chassis::Chassis(int8_t motorPorts[4], bool reverseConfig[4], okapi::AbstractMotor::gearset igearset) :
    leftMotors({{motorPorts[0], reverseConfig[0], igearset, okapi::AbstractMotor::encoderUnits::counts},
                {motorPorts[1], reverseConfig[1], igearset, okapi::AbstractMotor::encoderUnits::counts}}), 
    rightMotors({{motorPorts[2], reverseConfig[2], igearset, okapi::AbstractMotor::encoderUnits::counts},
                {motorPorts[3], reverseConfig[3], igearset, okapi::AbstractMotor::encoderUnits::counts}}) {
    
    gearBox = igearset;

    leftMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    rightMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void Chassis::move(int left, int right) { //uses the pros controller which goes from -127 to 127
    int aleft = abs(left);
    int aright = abs(right);

    if (aleft > 127) {
        aleft = 127;
    }
    if (aright > 127) {
        aright = 127;
    }

    //getting an absolute value to use for the joymap
    int leftV = joyMap[aleft] * sign(left);
    int rightV = joyMap[aright] * sign(right);

    //applying gearbox values
    switch (gearBox) {
        case okapi::AbstractMotor::gearset::red:
            leftV *= 1;
            rightV *= 1;
            break;
        case okapi::AbstractMotor::gearset::green:
            leftV *= 2;
            rightV *= 2;
            break;
        case okapi::AbstractMotor::gearset::blue:
            leftV *= 6;
            rightV *= 6;
            break;
        default:
            break;
    }

    //send these values to the motors to move
    leftMotors.moveVelocity(leftV);
    rightMotors.moveVelocity(rightV);
}

MotorGroup Chassis::getLeftMotors() {
    return leftMotors;
}
MotorGroup Chassis::getRightMotors() {
    return rightMotors;
}