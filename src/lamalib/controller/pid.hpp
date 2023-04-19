#pragma once

#include "api.h"
#include <cmath>

namespace lamaLib {
struct PIDGains {
    double kp;
    double ki;
    double kd;
    double kf = 0;
	double a = 1;
};

class PIDController {
    PIDGains pidValues;

    bool flag_c;
	
    double integral;
    double integralCompensation;
    double previousError;
    double maximum;
    double minimum;
    double count;
	double leeway;

    public:
    PIDController(PIDGains pidValues, double maximum = 1, double minimum = -1, double integralCompensation = 10, double leeway = 1);

    double calculatePID(double current, double target);

    bool isComplete();

    void setGains(PIDGains pidValues);
    PIDGains getGains();
    double getIntegral();
    void setIntegralCompensation(double integralCompensation);
    double getIntegralComp();
    double getprevError();
    void setMaximum(double maximum);
    double getMaximum();
    void setMinimum(double minimum);
    double getMinimum();
    double getCount();
	double getLeeway();
    void resetPID();

    void PIDTuner(bool x, PIDGains pidValues);
};
} // namespace lamaLib