#include "pid.hpp"
using namespace lamaLib;

PIDController::PIDController(double p, double i, double d, double min, double max) {
    kp = p;
    ki = i;
    kd = d;
    minimum = min;
    maximum = max;
    difference = max - min;
}

void PIDController::resetPID() {
    integral = 0;
    derivative = 0;
    prevError = 0;
    count = 0;
}

int fabs(double val) {
    return ((val > 0) - (val < 0)) * val;
}

double PIDController::calculatePID(double current, double target, double leeway) {
    double error = target - current;
    if (error > maximum) {
        error -= difference;
    }
    if (error < minimum) {
        error += difference;
    }

    if (fabs(error) <= leeway) {
        count++;
        if (count > 5) {
            return 0;
        }
    } else {
        count = 0;
    }

    derivative = error - prevError;
    integral = fabs(error) < 10 ? integral + error : 0;
    prevError = error;
    return error * kp + integral * ki + derivative * kd;
}