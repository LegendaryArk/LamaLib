#pragma once

#include <cmath>

namespace lamaLib {
struct PIDValues {
    double kp;
    double ki;
    double kd;
    double kf;
};

class PIDController {
    public:
    PIDController(PIDValues ipidValues, double max = 1, double min = -1, double iComp = 10);

    double calculatePID(double current, double target, double leeway);

    void setPID(PIDValues ipidValues);
    PIDValues getPID();
    void resetPID();
    
    private:
    double integral;
    double integralComp;
    double prevError;
    PIDValues pidValues;
    double max;
    double min;
    double count;
};
}