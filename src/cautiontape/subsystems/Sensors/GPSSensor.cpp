#include "GPSSensor.hpp"
#include "pros/gps.h"

using namespace lamaLib;

GPS::GPS(int gpsPort)
	: pros::Gps(gpsPort) {}
GPS::GPS(pros::Gps gps)
	: pros::Gps(gps) {}

void GPS::gpsInitialize(Pose pose, int offsetX, int offsetY) {
	initialize_full(pose.x, pose.y, pose.theta, offsetX, offsetY);
}

int GPS::getRotation() {
	return get_rotation();
}

Pose GPS::getPose() {
	return {get_status().x, get_status().y, get_status().yaw};
}