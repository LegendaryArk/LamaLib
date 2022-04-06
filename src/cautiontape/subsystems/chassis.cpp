#include "chassis.hpp"

using namespace std;
using namespace lamaLib;

Chassis::Chassis(MotorGroup ileftMotors, MotorGroup irightmotors, double igearRatio) : leftMotors(ileftMotors), rightMotors(irightmotors), gearset(leftMotors.getGearing(), igearRatio) {
    leftMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    rightMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void Chassis::move(int left, int right) { //uses the pros controller which goes from -127 to 127
    int aleft = abs(left);
    int aright = abs(right);

    if (aleft > 127)
        aleft = 127;
    if (aright > 127)
        aright = 127;

    //getting an absolute value to use for the joymap
    int leftV = joyMap[aleft] * sign(left);
    int rightV = joyMap[aright] * sign(right);

    //applying gearbox values
    switch (gearset.internalGearset) {
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
            cout << "Inavlid gearbox\n";
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