#include "mecanum.hpp"

using namespace std;
using namespace lamaLib;

Mecanum* Mecanum::chassis = nullptr;

Mecanum* Mecanum::getChassis(Motor ifrontLeft, Motor ifrontRight, Motor irearLeft, Motor irearRight, ChassisScales ichassisScales, shared_ptr<Odometry> iodom) {
	if (!chassis)
		chassis = new Mecanum(ifrontLeft, ifrontRight, irearLeft, irearRight, ichassisScales, iodom);
	return chassis;
}
Mecanum* Mecanum::getChassis(Motor ifrontLeft, Motor ifrontRight, Motor irearLeft, Motor irearRight, ChassisScales ichassisScales, Encoders iencoders, EncoderScales iencoderScales) {
	if (!chassis) {
		shared_ptr<Odometry> odom = make_shared<Odometry>(iencoders, iencoderScales);
		chassis = new Mecanum(ifrontLeft, ifrontRight, irearLeft, irearRight, ichassisScales, odom);
	}
	return chassis;
}

Mecanum::Mecanum(Motor ifrontLeft, Motor ifrontRight, Motor irearLeft, Motor irearRight, ChassisScales ichassisScales, shared_ptr<Odometry> iodom)
			: frontLeft(ifrontLeft), frontRight(ifrontRight), rearLeft(irearLeft), rearRight(irearRight) {
	scales = ichassisScales;
	odom = iodom;

	frontLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	frontRight.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	rearLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	rearRight.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

	odom->startOdometry();
}

void Mecanum::move(double ipower, double itheta, double iturn) {
	// Math from: https://youtu.be/gnSW2QpkGXQ

    double sine = sin(itheta - M_PI / 4);
    double cosine = cos(itheta - M_PI / 4);
    double maximum = max(fabs(sine), fabs(cosine));

    double fl = ipower * (cosine / maximum) + iturn;
    double fr = ipower * (sine / maximum) - iturn;
    double rl = ipower * (sine / maximum) + iturn;
    double rr = ipower * (cosine / maximum) - iturn;

	frontLeft.moveVelocity(fl);
	frontRight.moveVelocity(fr);
	rearLeft.moveVelocity(rl);
	rearRight.moveVelocity(rr);
}

void Mecanum::tank(double ileft, double iright, double ideadzone) {
	if (fabs(ileft) < ideadzone)
		ileft = 0;
	if (fabs(iright) < ideadzone)
		iright = 0;
	frontLeft.moveVelocity(ileft * scales.MOTOR_RPM);
	frontRight.moveVelocity(iright * scales.MOTOR_RPM);
	rearLeft.moveVelocity(ileft * scales.MOTOR_RPM);
	rearRight.moveVelocity(iright * scales.MOTOR_RPM);
}

void Mecanum::arcade(double iforward, double iturn, double ideadzone) {
	if (fabs(iforward) < ideadzone)
		iforward = 0;
	if (fabs(iturn) < ideadzone)
		iturn = 0;
	move(iforward * scales.MOTOR_RPM, degToRad(90), iturn * scales.MOTOR_RPM);
}

void Mecanum::xArcade(double iforward, double istrafe, double iturn, double ideadzone) {
	if (fabs(iforward) < ideadzone)
		iforward = 0;
	if (fabs(istrafe) < ideadzone)
		istrafe = 0;
	if (fabs(iturn) < ideadzone)
		iturn = 0;

	double power = hypot(iforward, istrafe);
	double theta = atan2(iforward, istrafe);
	move(power * scales.MOTOR_RPM, theta, iturn * scales.MOTOR_RPM);
}

void Mecanum::moveDistance(double idist, vector<MotionPoint> ipoints, double degree) {
	double gearRatio = scales.gearset.ratio;
    // Generates trapezoidal profiles for each cutoff section
    MotionProfile profile = MotionProfiling::generateFullTrapezoid(ipoints.at(0).motionLimit / gearRatio, {0}, {ipoints.at(0).distance / gearRatio});

    for (int i = 1; i < ipoints.size(); i++)
        profile += MotionProfiling::generateFullTrapezoid(ipoints.at(i).motionLimit / gearRatio, {ipoints.at(i - 1).distance / gearRatio, ipoints.at(i - 1).motionLimit.maxVelocity / gearRatio, profile.profile.at(i - 1).time}, {ipoints.at(i).distance / gearRatio, ipoints.at(i).motionLimit.maxVelocity / gearRatio});

    for (MotionData vel : profile.profile) {
        // Calculates velocity to rpm
        double rpm = vel.velocity * 60 / (M_PI * scales.wheelDiameter);
        move(rpm, degToRad(degree), 0); // 90 to move forward
        pros::delay(20);
    }
    move(0, 0, 0);
}
void Mecanum::moveDistance(double idist, vector<MotionPoint> ipoints) {
	moveDistance(idist, ipoints, 90);
}

void Mecanum::turnAbsolute(double iangle, double ivel, PIDGains ipid) {
	PIDController pidControl(ipid);

	double currHeading = odom->getPose().theta;
	while (fabs(currHeading - iangle) > 1) {
		double signal = pidControl.calculatePID(currHeading, iangle);

		double rpm = ivel * 60 / (M_PI * scales.wheelDiameter);

		move(0, 0, ivel * signal);

		currHeading = odom->getPose().theta;
		pros::delay(10);
	}

	move(0, 0, 0);
}
void Mecanum::turnRelative(double iangle, double ivel, PIDGains ipid) {
	turnAbsolute(odom->getPose().theta + iangle, ivel, ipid);
}

/*
void Mecanum::moveToPose(Pose ipose, vector<MotionPoint> ipoints, double iturnVel, PIDGains iturnPid, double idecelRangeOffset) {
	// Logic Flowchart: https://lucid.app/lucidchart/c1fa4ac9-e08a-4f6e-883b-f7a631f65cb0/edit?viewport_loc=12%2C252%2C1792%2C876%2C0_0&invitationId=inv_720f95dd-2602-42aa-b9a9-e8100adc7785#

	// Acceleration profile
	AccelProfile accels[ipoints.size()];
	for (int i = 0; i < ipoints.size(); i++)
		accels[i] = generateAccelTrapezoid(ipoints.at(i).motionLimit, {i == 0 ? 0 : ipoints.at(i - 1).distance}, {odom->getPose().distTo(ipose)});

	// Calculates initial distance and theta
	double distLeft = odom->getPose().distTo(ipose);
	double angLeft = ipose.theta - odom->getPose().theta;
	double theta = degToRad(odom->getPose().angleTo(ipose) + odom->getPose().theta);
	double startTime = pros::millis();
	double time = startTime - pros::millis();
	int i = 0, j = 0; // needed to keep track of where in the acceleration profile it is at

	int isComplete = 0;
	bool isAccelerating = false;

	PIDController pidControl(iturnPid, 1, -1, 10);
	while (isComplete < 5) {
		double rpm = ipoints.at(i).motionLimit.maxVelocity * 60 / (M_PI * scales.wheelDiameter); // Max Velocity
		double turnSignal = pidControl.calculatePID(odom->getPose().theta, ipose.theta);
		if (distLeft <= (accels[i].decelDist + idecelRangeOffset)) { // Deceleration
			// cout << "decelerating\n";
			rpm = accels[i].profile.at(j--).velocity * 60 / (M_PI * scales.wheelDiameter);
			if (j < 1 && !isAccelerating)
				j = 1;
		} else if (time <= accels[i].accelTime) { // Acceleration
			// cout << "accelerating\n";
			isAccelerating = true;
			rpm = accels[i].profile.at(j++).velocity * 60 / (M_PI * scales.wheelDiameter);
		}
		// Predicts and compensates for the turn velocity
		// theta += (((iturnVel * turnSignal) / 60) * (M_PI * scales.wheelDiameter) / (M_PI * odom->getEncoderScales().wheelTrack)) * 0.36;
		move(rpm, theta, iturnVel * turnSignal);

		if (time > accels[i].accelTime && isAccelerating) {
			j = accels[i].profile.size() - 1;
			isAccelerating = false;
		} else if (distLeft > (accels[i].decelDist + idecelRangeOffset) && i < ipoints.size() - 1) { // DecelDist does not take into account of the total distance
			i++;
			j = 0;
			isAccelerating = true;
		}

		if (fabs(distLeft) < 0.2 && fabs(angLeft) < 1) {
			j = 0;
			isAccelerating = true;
			isComplete++;
		} else if (fabs(distLeft) < 0.2) {
			j = 0;
			isAccelerating = true;
			isComplete = 0;
		} else {
			isComplete = 0;
		}

		cout << odom->getPose().x << "\t" << odom->getPose().y << "\t" << odom->getPose().theta << "\t\t" << distLeft << "\t" << angLeft << "\t" << rpm << "\t" << radToDeg(theta) << "\t" << turnSignal << "\t" << time << "\n";

		pros::delay(20);

		pros::lcd::print(6, "dist left: %f", distLeft);
		pros::lcd::print(7, "ang left: %f", angLeft);

		// Updates the distance, theta, and time
		distLeft = odom->getPose().distTo(ipose);
		angLeft = ipose.theta - odom->getPose().theta;
		theta = degToRad(odom->getPose().angleTo(ipose) + odom->getPose().theta);
		time = pros::millis() - startTime;
	}

	move(0, 0, 0);
	cout << "complete\n";
}
*/

void Mecanum::moveToPose(Pose itarget, vector<MotionPoint> ipoints, double iturnVel, PIDGains iturnPid) {}

okapi::AbstractMotor::brakeMode Mecanum::getBrakeMode() {
	return frontLeft.getBrakeMode();
}
void Mecanum::setBrakeMode(okapi::AbstractMotor::brakeMode ibrakeMode) {
	frontLeft.setBrakeMode(ibrakeMode);
	frontRight.setBrakeMode(ibrakeMode);
	rearLeft.setBrakeMode(ibrakeMode);
	rearRight.setBrakeMode(ibrakeMode);
}