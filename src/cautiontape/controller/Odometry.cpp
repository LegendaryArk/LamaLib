#include "Odometry.hpp"

using namespace std;
using namespace lamaLib;

void EncoderScales::operator=(EncoderScales rhs) {
	leftRadius = rhs.leftRadius;
	rightRadius = rhs.rightRadius;
	rearRadius = rhs.rearRadius;
	leftWheelDiameter = rhs.leftWheelDiameter;
	rightWheelDiameter = rhs.rightWheelDiameter;
	rearWheelDiameter = rhs.rearWheelDiameter;
	
	wheelTrack = leftRadius + rearRadius;
}

EncoderTicks EncoderTicks::operator+(EncoderTicks rhs) {
    return {left + rhs.left, right + rhs.right, rear + rhs.rear};
}
EncoderTicks EncoderTicks::operator-(EncoderTicks rhs) {
    return {left - rhs.left, right - rhs.right, rear - rhs.rear};
}

Odometry::Odometry(Encoders encoders, EncoderScales scales) : encoders(encoders), scales(scales) {}

void Odometry::updatePose(EncoderTicks ireadingsDiff) {
    if (fabs(ireadingsDiff.left) > 100 || fabs(ireadingsDiff.right) > 100 || fabs(ireadingsDiff.rear) > 100) {
        cerr << ireadingsDiff.left << "\t" << ireadingsDiff.right << "\t" << ireadingsDiff.rear << "\n";
        return;
    }

    double leftWheelCircumference = scales.leftWheelDiameter * M_PI;
    double rightWheelCircumference = scales.rightWheelDiameter * M_PI;
    double rearWheelCircumference = scales.rearWheelDiameter * M_PI;
    double chassisDiameter = scales.leftRadius + scales.rightRadius;
    
    // Delta distance
    OdomValues delta;
    delta.left = ((double) ireadingsDiff.left / scales.tpr) * leftWheelCircumference;
    delta.right = ((double) ireadingsDiff.right / scales.tpr) * rightWheelCircumference;

    // Delta theta
    delta.theta = (delta.left - delta.right) / chassisDiameter;

    delta.rear = 0; // ((ireadingsDiff.rear / iencoders.rearTPR) * rearWheelCircumference) - (delta.theta * scales.rearRadius);

    // Local coordinates
    double localOffsetX = delta.rear;
    double localOffsetY = (delta.left + delta.right) / 2;

    if (delta.left != delta.right) {
        localOffsetX = 2 * sin(delta.theta / 2) * (delta.rear / delta.theta + scales.rearRadius);
        localOffsetY = 2 * sin(delta.theta / 2) * (delta.right / delta.theta + (delta.left > delta.right ? scales.rightRadius : scales.leftRadius));
    }

    // Polar coordinates
    double polarRadius = sqrt(pow(localOffsetX, 2) + pow(localOffsetY, 2));
    double polarAngle = atan2(localOffsetY, localOffsetX) - (degToRad(pose.theta) + delta.theta / 2);

    // Global coordinates
    double deltaGlobalX = cos(polarAngle) * polarRadius;
    double deltaGlobalY = sin(polarAngle) * polarRadius;

    if (isnan(deltaGlobalX)) deltaGlobalX = 0;
    if (isnan(deltaGlobalY)) deltaGlobalY = 0;
    if (isnan(delta.theta)) delta.theta = 0;

    double globalX = pose.x + deltaGlobalX;
    double globalY = pose.y + deltaGlobalY;
    double globalTheta = pose.theta + radToDeg(delta.theta);
	setPose({globalX, globalY, globalTheta, pros::millis()});
}

Pose Odometry::getPose() {
	return pose;
}
void Odometry::setPose(Pose newPose) {
	pose = newPose;
}

Encoders Odometry::getEncoders() {
	return encoders;
}
void Odometry::setEncoders(Encoders newEncoders) {
	encoders = newEncoders;
}

EncoderScales Odometry::getEncoderScales() {
	return scales;
}
void Odometry::setEncoderScales(EncoderScales newScales) {
	scales = newScales;
}

EncoderTicks Odometry::getEncoderTicks() {
	return {encoders.left.get(), encoders.right.get(), encoders.rear.get()};
}

void Odometry::startOdom() {
	odomTask = pros::c::task_create(odometryMain, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odoemtry");
}
void Odometry::endOdom() {
	pros::c::task_delete(odomTask);
}

void odometryMain(void* odometry) {
	Odometry* odom = (Odometry*) odometry;

	Encoders encoders = odom->getEncoders();
	EncoderScales scales = odom->getEncoderScales();

	EncoderTicks prev = {0, 0, 0};

	uint32_t time = pros::millis();
	while (true) {
		EncoderTicks current = odom->getEncoderTicks();
		EncoderTicks diff = current - prev;

		odom->updatePose(diff);

		pros::lcd::print(0, "x: %.2f in   y: %.2f in", odom->getPose().x, odom->getPose().y);
        pros::lcd::print(1, "theta: %.2f deg", odom->getPose().theta);

        pros::lcd::print(2, "left: %.2f    right: %.2f", odom->getEncoderTicks().left, odom->getEncoderTicks().right);
        pros::lcd::print(3, "rear: %.2f", odom->getEncoderTicks().rear);

		prev = current;
		pros::Task::delay_until(&time, 10);
	}
}
