#include "Inertial.hpp"

using namespace lamaLib;

Inertial::Inertial(int iport) : pros::IMU(iport) {
    calibrating = false;

    startCorrection();
}

void Inertial::resetAll() {
    this->tare();
}

pros::c::imu_gyro_s_t Inertial::getDeltaAngles() {
    return this->get_gyro_rate();
}

double Inertial::getRoll() {
    return this->get_roll();
}
void Inertial::setRoll(double iangle) {
    this->set_roll(iangle);
}

double Inertial::getPitch() {
    return this->get_pitch();
}
void Inertial::setPitch(double iangle) {
    this->set_pitch(iangle);
}

double Inertial::getYaw() {
    return this->get_yaw();
}
void Inertial::setYaw(double iangle) {
    this->set_yaw(iangle);
}
double Inertial::getHeading() {
    return this->get_heading();
}
void Inertial::setHeading(double iangle) {
    this->set_heading(iangle);
}
double Inertial::getRotation() {
    return this->get_rotation();
}
void Inertial::setRotation(double iangle) {
    this->set_rotation(iangle);
}

Angles Inertial::calibrate() {
    if (calibrating) return {0, 0, 0};

    calibrating = true;
    pros::lcd::print(1, "Calibrating...");

    this->reset();
    pros::delay(2500);

    int count = 0;
    Angles maxDrift {0, 0, 0};
    while (count < 100) {
        double deltaRoll = fabs(getDeltaAngles().x);
        double deltaPitch = fabs(getDeltaAngles().y);
        double deltaYaw = fabs(getDeltaAngles().z);
        
        if (deltaRoll > maxDrift.x)
            maxDrift.x = deltaRoll;
        if (deltaPitch > maxDrift.y)
            maxDrift.y = deltaPitch;
        if (deltaYaw > maxDrift.z) 
            maxDrift.z = deltaYaw;

        pros::delay(100);
        count++;
    }
    if (maxDrift.x > 0.5)
        maxDrift.x = 0.5;
    if (maxDrift.y > 0.5)
        maxDrift.y = 0.5;
    if (maxDrift.z > 0.5)
        maxDrift.z = 0.5;

    calibrating = false;
    pros::lcd::print(1, "Calibration Complete");
    return maxDrift;
}
bool Inertial::isCalibrating() {
    return calibrating;
}

void Inertial::startCorrection() {
    inertialTask = pros::c::task_create(driftCompensation, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "inertial");
}
void Inertial::endCorrection() {
    pros::c::task_delete(inertialTask);
}

void lamaLib::driftCompensation(void* iparam) {
    Angles maxDrift = inertial.calibrate();

    while (true) {
        Angles angles = {inertial.getRoll(), inertial.getPitch(), inertial.getYaw()};
        Angles prev = angles;
        Angles delta = {inertial.getDeltaAngles().x, inertial.getDeltaAngles().y, inertial.getDeltaAngles().z};
        Angles errors;

        if (delta.x > maxDrift.x) {
            if (angles.x != prev.x)
                errors.x = prev.x - angles.x;
        }
        if (delta.y > maxDrift.y) {
            if (angles.y != prev.y)
                errors.y = prev.y - angles.y;
        }
        if (delta.z > maxDrift.z) {
            if (angles.z != prev.z)
                errors.z = prev.z - angles.z;
        }

        prev = angles;

        inertial.setRoll(angles.x + errors.x);
        inertial.setPitch(angles.y + errors.y);
        inertial.setYaw(angles.z + errors.z);

        // std::cout << "x: " << inertial.getRoll() << "\ty: " << inertial.getPitch() << "\tz: " << inertial.getHeading() << "\n";

        pros::delay(10);
    }
}