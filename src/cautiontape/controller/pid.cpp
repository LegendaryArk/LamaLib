#include "pid.hpp"
#include <iostream>

using namespace std;
using namespace lamaLib;

PIDController::PIDController(PIDValues ipidValues, double max, double min, double iComp) :
                            pidValues(ipidValues), max(max), min(min), integralComp(iComp) {}

double PIDController::calculatePID(double current, double target, double leeway) {
    double error = target - current;
    // cout << error << endl;

    if (fabs(error) <= leeway) {
        count++;
        if (count > 4) {
            return 0;
        }
    } else {
        count = 0;
    }

    double derivative = error - prevError;
    double newIntegral = fabs(error) < integralComp ? integral + error : 0;
    prevError = error;

    double signal = error * pidValues.kp + integral * pidValues.ki + derivative * pidValues.kd + pidValues.kf;
    if (signal > max)
        signal = max;
    else if (signal < min)
        signal = min;
    else
        integral = newIntegral;

    return signal;
}

void PIDController::setPID(PIDValues ipidValues) {
    pidValues = ipidValues;
}
PIDValues PIDController::getPID() {
    return pidValues;
}
void PIDController::resetPID() {
    integral = 0;
    prevError = 0;
    count = 0;
}