#include "odometry.hpp"
#include <math.h>

using namespace lamaLib;

Odometry::Odometry(ADIEncoder leftEncoder, ADIEncoder rightEncoder, ADIEncoder rearEncoder, OdomScales scales, int tpr):
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

OdomScales Odometry::calibrate(IMU inertial) {
    EncoderValues vals = {0, 0, 0, 0};
    while (inertial.get_rotation() < 3600) {
        vals.left = getEncoders().left;
        vals.right = getEncoders().right;
        vals.rear = getEncoders().rear;
    }

    return {0, 0, 0, 0};
}

OdomScales Odometry::getScales() {
    return scales;
}
void Odometry::setScales(OdomScales iscales) {
    scales = iscales;
}

void Odometry::startOdom() {
    odomTask = c::task_create(odometryMain, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
}
void Odometry::endOdom() {
    c::task_delete(odomTask);
}

void lamaLib::odometryMain(void* param) {
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

        lcd::print(0, "x: %.2f in   y: %.2f in", odom.getPose().x, odom.getPose().y);
        lcd::print(1, "theta: %.2f deg", odom.getPose().theta);

        lcd::print(3, "left: %.2f    right: %.2f", readings.left, readings.right);
        lcd::print(4, "rear: %.2f", readings.rear);
        
        Task::delay_until(&time, 10);
    }
}