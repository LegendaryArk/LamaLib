#include "inertial.hpp"

using namespace std;
using namespace lamaLib;

Axes Axes::operator+(Axes rhs) {
	return {x + rhs.x, y + rhs.y, z + rhs.z};
}
Axes Axes::operator-(Axes rhs) {
	return {x - rhs.x, y - rhs.y, z - rhs.z};
}

Inertial::Inertial(int port) : roll(port, okapi::IMUAxes::x), pitch(port, okapi::IMUAxes::y), yaw(port, okapi::IMUAxes::z) {}

void Inertial::reset() {
	roll.reset();
	pitch.reset();
	yaw.reset();
}

Axes Inertial::get(double upperBound, double lowerBound) {
	if (upperBound == lowerBound)
		return {roll.get(), pitch.get(), yaw.get()};
	return {roll.getRemapped(upperBound, lowerBound), pitch.getRemapped(upperBound, lowerBound), yaw.getRemapped(upperBound, lowerBound)};
}

void Inertial::calibrate() {
	// if (calibrating) {
	//     cerr << "Already calibrating\n";
	//     return {0, 0, 0};
	// }

	// calibrating = true;
	// pros::lcd::print(1, "Calibrating...");

	roll.calibrate();
	pitch.calibrate();
	yaw.calibrate();
/*
	int count = 0;
	Axes maxDrift {0, 0, 0};
	while (count < 100) {
		double deltaRoll = fabs(roll.get());
		double deltaPitch = fabs(pitch.get());
		double deltaYaw = fabs(yaw.get());
		pros::delay(10);
		deltaRoll -= roll.get();
		deltaPitch -= pitch.get();
		deltaYaw -= yaw.get();
		
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
	return maxDrift;*/
}
bool Inertial::isCalibrating() {
	return roll.isCalibrating() || pitch.isCalibrating() || yaw.isCalibrating();
}

void Inertial::startCorrection() {
	driftCompensationTask = pros::c::task_create(driftCompensation, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "inertial");
}
void Inertial::endCorrection() {
	pros::c::task_delete(driftCompensationTask);
}

void lamaLib::driftCompensation(void* inertial) {
	/*
	Inertial* inertial = (Inertial*) inertial;
	Axes maxDrift = inertial->calibrate();

	while (true) {
		Axes angles = inertial->get();
		Axes prev = angles;
		Axes delta = angles - prev;
		Axes errors;

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

		prev.x = angles.x;
		prev.y = angles.y;
		prev.z = angles.z;

		// inertial->setRoll(angles.x + errors.x);
		// inertial->setPitch(angles.y + errors.y);
		// inertial->setYaw(angles.z + errors.z);

		// cout << "x: " << inertial->getRoll() << "\ty: " << inertial->getPitch() << "\tz: " << inertial->getHeading() << "\n";

		pros::delay(10);
	}
	*/
}