#include "odometry.hpp"
#include "pros/misc.h"
#include <math.h>

using namespace lamaLib;

Odometry::Odometry(pros::ADIEncoder leftEncoder, pros::ADIEncoder rightEncoder, pros::ADIEncoder rearEncoder, OdomScales scales, int tpr):
                    leftEncoder(leftEncoder), rightEncoder(rightEncoder), rearEncoder(rearEncoder), scales(scales), tpr(tpr) {
    // Default coordinate position
    pose = {0, 0, 0, pros::millis()};

    // Start odometry task
    startOdom();
}

EncoderValues Odometry::getEncoders() {
    return {(double) leftEncoder.get_value(), (double) rightEncoder.get_value(), (double) rearEncoder.get_value(), 0};
}

Pose Odometry::getPose() {
    return pose;
}
void Odometry::setPose(Pose ipose) {
    pose = ipose;
}

OdomScales Odometry::calibrate(Chassis ichassis, pros::Controller controller, pros::IMU iinertial) {
    uint32_t time = pros::millis();
    while (iinertial.get_rotation() < 3600) {
        ichassis.move(200, -200);

        pros::Task::delay_until(&time, 10);
    }

    ichassis.move(0, 0);

    while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        pros::Task::delay_until(&time, 10);
    }

    double leftDiameter = ((getEncoders().left / tpr) * scales.wheelDiameter) / 10;
    double rightDiameter = ((getEncoders().right / tpr) * scales.wheelDiameter) / 10;
    double rearDiameter = ((getEncoders().rear / tpr) * scales.wheelDiameter) / 10;

    OdomScales calibratedScales = {scales.wheelDiameter, leftDiameter / 2, rightDiameter / 2, rearDiameter / 2};
    pros::lcd::print(5, "Calibrated radii:");
    pros::lcd::print(6, "left: %f in   right: %f in", calibratedScales.leftRadius, calibratedScales.rightRadius);
    pros::lcd::print(7, "rear: %f in", calibratedScales.rearRadius);
    return calibratedScales;
}

OdomScales Odometry::getScales() {
    return scales;
}
void Odometry::setScales(OdomScales iscales) {
    scales = iscales;
}

void Odometry::startOdom() {
    odomTask = pros::c::task_create(odometryMain, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
}
void Odometry::endOdom() {
    pros::c::task_delete(odomTask);
}

void lamaLib::odometryMain(void* iparam) {
    OdomScales scales = odom.getScales();
    double chassisDiameter = scales.leftRadius + scales.rightRadius;

    EncoderValues readings = odom.getEncoders();
    EncoderValues readingsPrev = {0, 0, 0, 0};

    const int tpr = odom.tpr;
    const double wheelCircumference = scales.wheelDiameter * M_PI;

    uint32_t time = pros::millis();
    while (true) {
        Pose currPose = odom.getPose();

        readingsPrev = readings;
        readings = odom.getEncoders();

        EncoderValues readingsDiff;
        readingsDiff.left = readings.left - readingsPrev.left;
        readingsDiff.right = readings.right - readingsPrev.right;
        readingsDiff.rear = readings.rear - readingsPrev.rear;
        
        // Delta distance
        EncoderValues delta;
        delta.left = (readingsDiff.left / tpr) * wheelCircumference;
        delta.right = (readingsDiff.right / tpr) * wheelCircumference;

        // Delta theta
        delta.theta = (delta.left - delta.right) / chassisDiameter;

        delta.rear = ((readingsDiff.rear / tpr) * wheelCircumference) - (delta.theta * scales.rearRadius);

        // Local coordinates
        double localOffsetX = delta.rear;
        double localOffsetY = (delta.left + delta.right) / 2;

        if (delta.left != delta.right) {
            localOffsetX = 2 * sin(delta.theta / 2) * (delta.rear / delta.theta + scales.rearRadius);
            localOffsetY = 2 * sin(delta.theta / 2) * (delta.right / delta.theta + (delta.left > delta.right ? scales.rightRadius : scales.leftRadius));
        }

        // Polar coordinates
        double polarRadius = sqrt(pow(localOffsetX, 2) + pow(localOffsetY, 2));
        double polarAngle =  atan2(localOffsetY, localOffsetX) - (degToRad(currPose.theta) + delta.theta / 2);

        // Global coordinates
        double deltaGlobalX = cos(polarAngle) * polarRadius;
        double deltaGlobalY = sin(polarAngle) * polarRadius;

        if (isnan(deltaGlobalX)) deltaGlobalX = 0;
        if (isnan(deltaGlobalY)) deltaGlobalY = 0;
        if (isnan(delta.theta)) delta.theta = 0;

        double globalX = currPose.x + deltaGlobalX;
        double globalY = currPose.y + deltaGlobalY;
        double globalTheta = currPose.theta + radToDeg(delta.theta);
        odom.setPose({globalX, globalY, globalTheta, pros::millis()});

        pros::lcd::print(0, "x: %.2f in   y: %.2f in", odom.getPose().x, odom.getPose().y);
        pros::lcd::print(1, "theta: %.2f deg", odom.getPose().theta);

        pros::lcd::print(2, "left: %.2f    right: %.2f", readings.left, readings.right);
        pros::lcd::print(3, "rear: %.2f", readings.rear);
        
        pros::Task::delay_until(&time, 10);
    }
}