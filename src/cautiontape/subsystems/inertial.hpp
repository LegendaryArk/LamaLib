#pragma once

#include "api.h"

namespace lamaLib {
class Inertial {
    public:
    Inertial(pros::IMU inertial);

    void reset();

    double getHeading();
    void setHeading(double ingle);

    void calibrate();
    bool isCalibrating();

    private:
    pros::IMU inertial;

    double correction;
};
} // namespace lamaLib