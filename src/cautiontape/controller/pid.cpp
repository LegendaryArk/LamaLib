#include "pid.hpp"

using namespace lamaLib;

PIDController::PIDController(PIDValues pidValues, double max, double iComp) :
                            pidValues(pidValues), max(max), integralComp(iComp) {}

double PIDController::calculatePID(double current, double target, double leeway) {
    double error = target - current;

    if (fabs(error) <= leeway) {
        count++;
        if (count > 5) {
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
    else if (signal < -max)
        signal = -max;
    else
        integral = newIntegral;

    return signal;
}

void PIDController::updatePID(PIDValues pidValues) {
    this->pidValues = pidValues;
}
void PIDController::resetPID() {
    integral = 0;
    prevError = 0;
    count = 0;
}