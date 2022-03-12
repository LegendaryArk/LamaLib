#include "chassis.hpp"
using namespace lamaLib;

Chassis::Chassis(int8_t motorPorts[4], bool reverseConfig[4], AbstractMotor::gearset igearset) :
topLeft(motorPorts[0], reverseConfig[0], igearset, AbstractMotor::encoderUnits::degrees),
bottomLeft(motorPorts[1], reverseConfig[1], igearset, AbstractMotor::encoderUnits::degrees),
topRight(motorPorts[2], reverseConfig[2], igearset, AbstractMotor::encoderUnits::degrees),
bottomRight(motorPorts[3], reverseConfig[3], igearset, AbstractMotor::encoderUnits::degrees)
{
    gearBox = igearset;
}

void Chassis::move(int left, int right) { //uses the pros controller which goes from -127 to 127
    int joyMap[128] = { //joymap goes from 0 to 100
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        31, 31, 31, 31, 31, 31, 31, 31, 31, 31,
        31, 31, 31, 31, 31, 31, 31, 31, 31, 31,
        39, 40, 41, 42, 43, 44, 44, 45, 46, 47,
        48, 49, 50, 50, 50, 51, 51, 51, 52, 66,
        78, 78, 78, 78, 78, 78, 78, 78, 78, 78,
        86, 86, 86, 86, 86, 86, 86, 86, 86, 86,
        94, 94, 95, 96, 97, 98, 99,100,100,100,
        100,100,100,100,100,100,100,100
    };
    //getting the sign bit in order to apply the correct direction to the motor
    int signL = (left > 0) - (left < 0);
    int signR = (right > 0) - (right < 0);

    //getting an absolute value to use for the joymap
    int leftV = joyMap[abs(left)] * signL;
    int rightV = joyMap[abs(right)] * signR;

    //applying gearbox values
    switch (gearBox) {
        case AbstractMotor::gearset::green:
            leftV *= 2;
            rightV *= 2;
            break;
        case AbstractMotor::gearset::blue:
            leftV *= 6;
            rightV *= 6;
            break;

    }

    //send these values to the motors to move
    topLeft.moveVelocity(leftV);
    bottomLeft.moveVelocity(leftV);
    topRight.moveVelocity(rightV);
    bottomRight.moveVelocity(rightV);
}