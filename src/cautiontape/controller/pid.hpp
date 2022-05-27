#pragma once

#include <cmath>

namespace lamaLib {
struct PIDGains {
    double kp;
    double ki;
    double kd;
    double kf;
};

class PIDController {
    public:
    PIDController(PIDGains ipidValues, double max = 1, double min = -1, double iComp = 10);

    double calculatePID(double current, double target, double leeway);

    void setPID(PIDGains ipidValues);
    PIDGains getPID();
    void resetPID();
    
    private:
    double integral;
    double integralComp;
    double prevError;
    PIDGains pidValues;
    double max;
    double min;
    double count;
};
}