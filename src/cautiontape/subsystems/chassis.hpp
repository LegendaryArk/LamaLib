#pragma once

#include "okapi/api.hpp"
#include "motorgroup.hpp"

using namespace std;
namespace lamaLib {

static int joyMap[128] = { //joymap goes from 0 to 100
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
    94, 94, 95, 96, 97, 98, 99, 100,100,100,
    100,100,100,100,100,100,100,100
};

class Chassis {
    public:
    Chassis(int8_t motorPorts[4], bool reverseConfig[4], okapi::AbstractMotor::gearset igearset);
    void move(int left, int right);
    
    private:
    lamaLib::MotorGroup leftMotors;
    lamaLib::MotorGroup rightMotors;
    okapi::AbstractMotor::gearset gearBox;
};
} // namespace lamaLib