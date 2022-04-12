#include "pid.hpp"

using namespace lamaLib;

PIDController::PIDController(double kp, double ki, double kd, double kf, double max, double iComp) :
                            kp(kp), ki(ki), kd(kd), kf(kf), max(max), integralComp(iComp) {}

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

    double signal = error * kp + integral * ki + derivative * kd + kf;
    if (signal > max) {
        signal = max;
    } else if (signal < -max) {
        signal = -max;
    } else {
        integral = newIntegral;
    }

    return signal;
}

void PIDController::updatePID(double kp, double ki, double kd, double kf) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kf = kf;
}
void PIDController::resetPID() {
    integral = 0;
    prevError = 0;
    count = 0;
}