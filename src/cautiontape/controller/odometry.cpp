#include "odometry.hpp"

using namespace std;
using namespace lamaLib;

Odometry::Odometry() {}

Pose Odometry::updatePose(Pose icurrPose, RobotScales iscales, Encoders iencoders, OdomValues ireadingsDiff) {
    cout << iscales.wheelDiameter << "\n";
    double wheelCircumference = iscales.wheelDiameter * M_PI;
    double chassisDiameter = iscales.leftRadius + iscales.rightRadius;
    
    // Delta distance
    OdomValues delta;
    delta.left = ((double) ireadingsDiff.left / iencoders.leftTPR) * wheelCircumference * iscales.gearRatio;
    delta.right = ((double) ireadingsDiff.right / iencoders.rightTPR) * wheelCircumference * iscales.gearRatio;

    // Delta theta
    delta.theta = (delta.left - delta.right) / chassisDiameter;

    delta.rear = ((ireadingsDiff.rear / iencoders.rearTPR) * wheelCircumference) - (delta.theta * iscales.rearRadius);

    // Local coordinates
    double localOffsetX = delta.rear;
    double localOffsetY = (delta.left + delta.right) / 2;

    if (delta.left != delta.right) {
        localOffsetX = 2 * sin(delta.theta / 2) * (delta.rear / delta.theta + iscales.rearRadius);
        localOffsetY = 2 * sin(delta.theta / 2) * (delta.right / delta.theta + (delta.left > delta.right ? iscales.rightRadius : iscales.leftRadius));
    }

    // Polar coordinates
    double polarRadius = sqrt(pow(localOffsetX, 2) + pow(localOffsetY, 2));
    double polarAngle =  atan2(localOffsetY, localOffsetX) - (degToRad(icurrPose.theta) + delta.theta / 2);

    // Global coordinates
    double deltaGlobalX = cos(polarAngle) * polarRadius;
    double deltaGlobalY = sin(polarAngle) * polarRadius;

    if (isnan(deltaGlobalX)) deltaGlobalX = 0;
    if (isnan(deltaGlobalY)) deltaGlobalY = 0;
    if (isnan(delta.theta)) delta.theta = 0;

    double globalX = icurrPose.x + deltaGlobalX;
    double globalY = icurrPose.y + deltaGlobalY;
    double globalTheta = icurrPose.theta + radToDeg(delta.theta);
    
    return {globalX, globalY, globalTheta, pros::millis()};
}