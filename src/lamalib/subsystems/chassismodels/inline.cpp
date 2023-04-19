#include "inline.hpp"

using namespace std;
using namespace lamaLib;

Inline* Inline::chassis = nullptr;

Inline* Inline::getChassis(shared_ptr<MotorGroup> left, shared_ptr<MotorGroup> right, ChassisScales chassisScales, shared_ptr<Odometry> odom) {
	if (!chassis) // Checks if a chassis object was created before
		chassis = new Inline(left, right, chassisScales, odom);
	return chassis;
}
Inline* Inline::getChassis(shared_ptr<MotorGroup> left, shared_ptr<MotorGroup> right, ChassisScales chassisScales, Encoders encoders, EncoderScales encoderScales) {
	if (!chassis) { // Checks if a chassis object was created before
		Odometry odom(encoders, encoderScales);
		chassis = new Inline(left, right, chassisScales, make_shared<Odometry>(odom));
	}
	return chassis;
}

Inline::Inline(shared_ptr<MotorGroup> left, shared_ptr<MotorGroup> right, ChassisScales chassisScales, shared_ptr<Odometry> odom)
			: leftMotors(left), rightMotors(right) {
	scales = chassisScales;
	this->odom = odom;

	leftMotors->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	rightMotors->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

	this->odom->startOdometry();
}

void Inline::move(double forward, double turn) {
	leftMotors->moveVelocity((forward + turn) * scales.MOTOR_RPM);
	rightMotors->moveVelocity((forward - turn) * scales.MOTOR_RPM);
}

void Inline::tank(double left, double right, double deadzone) {
	if (fabs(left) < deadzone)
		left = 0;
	if (fabs(right) < deadzone)
		right = 0;
	leftMotors->moveVelocity(left);
	rightMotors->moveVelocity(right);
}
void Inline::arcade(double forward, double turn, double deadzone) {
	if (fabs(forward) < deadzone)
		forward = 0;
	if (fabs(turn) < deadzone)
		turn = 0;
	move(forward, turn);
}

void Inline::moveDistance(double dist, vector<MotionPoint> points) {
	const double wheelDiameter = scales.wheelDiameter;
	const double gearRatio = scales.gearset.ratio;

	MotionPoint first = points.at(0) / gearRatio;
	MotionProfile motionProfile = MotionProfiling::generateFullTrapezoid(first.motionLimit, {0}, {first.distance, first.motionLimit.maxVelocity});
	// Used in the case of cutoff
	for (int i = 1; i < points.size(); i++) {
		MotionPoint curr = points.at(i) / gearRatio, prev = points.at(i - 1) / gearRatio;
		motionProfile += MotionProfiling::generateFullTrapezoid(curr.motionLimit,
													{prev.distance, prev.motionLimit.maxVelocity, motionProfile.profile.at(i - 1).time},
													{curr.distance, curr.motionLimit.maxVelocity});
	}

	for (MotionData vel : motionProfile.profile) {
		double rpm = vel.velocity * 60 / (M_PI * wheelDiameter);
		// cout << rpm << "," << vel.distance << "," << leftMotors.getActualVelocity() << "," << rightMotors.getActualVelocity() << "\n";
		move(rpm, 0);

		pros::delay(20);
	}
	leftMotors->moveVelocity(0);
	rightMotors->moveVelocity(0);
}

void Inline::turnAbsolute(double target, double maxVel, PIDGains pid) {
	Pose pose = odom->getPose();
	const double wheelDiameter = scales.wheelDiameter;

	PIDController pidControl(pid);
	while (fabs(target - pose.theta) > 1) {
		// Calculates the signal depending on the current heading and the target heading
		double pid = pidControl.calculatePID(pose.theta, target);

		// Converts from velocity to rpm
		double rpm = maxVel * 60 / (M_PI * wheelDiameter);
		move(0, rpm / scales.MOTOR_RPM);

		// Updates the pose
		pose = odom->getPose();

		pros::delay(10);
	}
	leftMotors->moveVelocity(0);
	rightMotors->moveVelocity(0);
}

void Inline::turnRelative(double target, double maxVel, PIDGains pid) {
	turnAbsolute(odom->getPose().theta + target, maxVel, pid);
}

void Inline::moveToPose(Pose target, vector<MotionPoint> points, double turnVel, PIDGains turnPID) {
	Pose pose = odom->getPose();

	// Turns to face the point
	double angle = pose.angleTo(target);
	turnAbsolute(angle, turnVel, turnPID);

	// Moves to the point
	double dist = pose.distTo(target);
	moveDistance(dist, points);

	// Faces the specified heading
	turnAbsolute(target.theta, turnVel, turnPID);
}

shared_ptr<MotorGroup> Inline::getLeftMotors() {
	return leftMotors;
}
shared_ptr<MotorGroup> Inline::getRightMotors() {
	return rightMotors;
}

void Inline::setBrakeMode(okapi::AbstractMotor::brakeMode brakMode) {
	leftMotors->setBrakeMode(brakMode);
	rightMotors->setBrakeMode(brakMode);
}
okapi::AbstractMotor::brakeMode Inline::getBrakeMode() {
	return leftMotors->getBrakeMode();
}

void Inline::updateGearRatio(double gearRatio) {
	scales.gearset.ratio = gearRatio;
}