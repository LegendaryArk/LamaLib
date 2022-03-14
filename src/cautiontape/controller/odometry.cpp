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

OdomScales Odometry::calibrate() {
    
}

OdomScales Odometry::getScales() {
    return scales;
}
void Odometry::setScales(OdomScales iscales) {
    scales = iscales;
}

void Odometry::startOdom() {
    odom = c::task_create(Odom, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
}
void Odometry::endOdom() {
    c::task_delete(odom);
}

void Odom(void* param) {
    Odometry* odom = (Odometry*) param;

    OdomScales scales = odom->getScales();

    double chassisDiameter = scales.leftRadius + scales.rightRadius;

    EncoderValues readings = odom->getEncoders();
    EncoderValues readingsPrev = odom->prev;
    EncoderValues readingsDiff;

    readingsDiff.left = readings.left - readingsPrev.left;
    readingsDiff.right = readings.right - readingsPrev.right;
    readingsDiff.rear = readings.rear - readingsPrev.rear;
    
    // Delta distance
    EncoderValues delta;
    delta.left = (readingsDiff.left / odom->tpr) * (chassisDiameter * M_PI);
    delta.right = (readingsDiff.right / odom->tpr) * (chassisDiameter * M_PI);
    delta.rear = (readingsDiff.rear / odom->tpr) * (chassisDiameter * M_PI);

    // Delta theta
    delta.theta = (readingsDiff.left - readingsDiff.right) / chassisDiameter;

    // Local coordinates
    double localOffsetX = delta.rear;
    double localOffsetY = delta.left;
    if (delta.left != delta.right) {
        localOffsetX = 2 * sin(delta.theta / 2) * (delta.rear / delta.theta + scales.rearRadius);
        localOffsetY = 2 * sin(delta.theta / 2) * (delta.left / delta.theta + (delta.left > delta.right ? scales.rightRadius : scales.leftRadius));
    }

    // Polar coordinates
    double polarRadius = sqrt(localOffsetX * localOffsetX + localOffsetY * localOffsetY);
    double polarAngle =  atan2(localOffsetY, localOffsetX) - (odom->getPose().theta + delta.theta / 2);

    // Global coordinates
    double deltaGlobalX = cos(polarAngle) * polarRadius;
    double deltaGlobalY = sin(polarAngle) * polarRadius;

    if (isnan(deltaGlobalX)) deltaGlobalX = 0;
    if (isnan(deltaGlobalY)) deltaGlobalY = 0;
    if (isnan(delta.theta)) delta.theta = 0;

    double globalX = odom->getPose().x + deltaGlobalX;
    double globalY = odom->getPose().y + deltaGlobalY;
    double globalTheta = odom->getPose().theta + delta.theta;
    odom->setPose({globalX, globalY, globalTheta, pros::millis()});

    odom->prev = readings;
}