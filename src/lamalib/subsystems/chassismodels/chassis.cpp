#include "chassis.hpp"

using namespace std;
using namespace lamaLib;

ChassisScales Chassis::getChassisScales() {
	return scales;
}

shared_ptr<Odometry> Chassis::getOdom() {
	return odom;
}