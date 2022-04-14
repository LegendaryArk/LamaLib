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
    PIDController(PIDValues pidValues, double max, double iComp = 10);

    double calculatePID(double current, double target, double leeway);
    void asyncPID(double current, double target, double leeway);

    void updatePID(PIDValues pidValues);
    void resetPID();
    
    private:
    double integral;
    double integralComp;
    double prevError;
    PIDValues pidValues;
    double max;
    double count;
};
}