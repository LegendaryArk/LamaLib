#pragma once

#include "api.h"
#include "pros/imu.h"
#include "pros/rtos.h"

namespace lamaLib {

typedef struct {
    double x;
    double y;
    double z;
} Angles;

class Inertial : pros::IMU {
    public:
    Inertial(int port);

    void resetAll();

    pros::c::imu_gyro_s_t getDeltaAngles();

    double getRoll();
    void setRoll(double iangle);

    double getPitch();
    void setPitch(double iangle);

    double getHeading();
    void setHeading(double iangle);

    Angles calibrate();
    bool isCalibrating();

    void startTask();
    void endTask();

    private:
    pros::task_t inertialTask;
};

extern Inertial inertial;

void driftCompensation(void* iparam);
} // namespace lamaLib