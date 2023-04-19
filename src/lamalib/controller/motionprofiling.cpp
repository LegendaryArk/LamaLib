#include "motionprofiling.hpp"

using namespace std;
using namespace lamaLib;

void MotionProfile::operator+=(MotionProfile rhs) {
	for (MotionData data : rhs.profile)
		profile.emplace_back(data);
}

MotionLimit MotionLimit::operator*(double rhs) {
	return {maxVelocity * rhs, maxAcceleration * rhs, maxJerk * rhs};
}
MotionLimit MotionLimit::operator/(double rhs) {
	return {maxVelocity / rhs, maxAcceleration / rhs, maxJerk / rhs};
}

MotionPoint MotionPoint::operator*(double rhs) {
	return {motionLimit * rhs, distance * rhs};
}
MotionPoint MotionPoint::operator/(double rhs) {
	return {motionLimit / rhs, distance / rhs};
}

MotionProfile MotionProfiling::generateFullTrapezoid(MotionLimit motionLimit, CutoffPoint start, CutoffPoint end) {
	MotionProfile trapezoid;
	trapezoid.profile = vector<MotionData>();

	double cutoffStartTime = fabs(start.velocity / motionLimit.maxAcceleration);
	double cutoffStartDistance = 0.5 * motionLimit.maxAcceleration * cutoffStartTime * cutoffStartTime;

	double cutoffEndTime = fabs(end.velocity / motionLimit.maxAcceleration);
	double cutoffEndDistance = 0.5 * motionLimit.maxAcceleration * cutoffEndTime * cutoffEndTime;

	// Acceleration t = v/d
	double accelerationTime = motionLimit.maxVelocity / motionLimit.maxAcceleration;
	// Acceleration distance = 1/2at^2
	double accelerationDist = 0.5 * motionLimit.maxAcceleration * accelerationTime * accelerationTime;
	double trapezoidDistance = cutoffStartDistance + fabs(end.distance - start.distance) + cutoffEndDistance;
	double maxDistance = trapezoidDistance - 2 * accelerationDist;

	// When max speed is not able to be reached
	if (maxDistance < 0) {
		accelerationTime = sqrt(fabs(end.distance) / motionLimit.maxAcceleration);
		maxDistance = 0;
	}

	double endAcceleration = accelerationTime - cutoffStartTime;
	// maxDistance time = (distance / maxVel) - endAccel
    double endMax = endAcceleration + fabs(maxDistance / motionLimit.maxVelocity);
	// endDecel = maxDistance + endAccel
	double endDeceleration = endMax + accelerationTime - cutoffEndTime;

	cout << endAcceleration << "\t" << endMax << "\t" << endDeceleration << "\t" << accelerationTime << "\t" << cutoffStartTime << "\t" << cutoffEndTime << "\n";

	int direction = sign(end.distance);
	motionLimit.maxVelocity *= direction;
	motionLimit.maxAcceleration *= direction;

	double time = 0;

	// std::cout << "velocity,distance,time\n";
	while (time <= endDeceleration) {
		MotionData movement;

		if (time < endAcceleration) { // Acceleration
			movement.velocity = start.velocity + motionLimit.maxAcceleration * time;
			movement.distance = start.distance + start.velocity * time + 0.5 * motionLimit.maxAcceleration * time * time;
			movement.acceleration = motionLimit.maxAcceleration;
			movement.jerk = motionLimit.maxAcceleration / time;
		} else if (time < endMax){ // Max Velocity
            movement.velocity = motionLimit.maxVelocity;
            movement.distance = start.distance + (start.velocity * time + 0.5 * motionLimit.maxAcceleration * accelerationTime * accelerationTime) + (motionLimit.maxVelocity * (time - endAcceleration));
			movement.acceleration = 0;
			movement.jerk = 0;
        } else if (time <= endDeceleration) { // Deceleration
            double timeLeft = endDeceleration - time;
            movement.velocity = end.velocity + motionLimit.maxAcceleration * timeLeft;
            movement.distance = end.distance - (0.5 * motionLimit.maxAcceleration * timeLeft * timeLeft);
			movement.acceleration = -motionLimit.maxAcceleration;
			movement.jerk = -motionLimit.maxAcceleration / time;
        } else {
			movement.velocity = 0;
		}
		movement.time = start.time + time;

		trapezoid.profile.emplace_back(movement);

		std::cout << movement.velocity << "," << movement.distance << "," << movement.time << "\n";

		time += 0.02;
	}

	return trapezoid;
}

AccelProfile MotionProfiling::generateAccelerationTrapezoid(MotionLimit motionLimit, CutoffPoint start, CutoffPoint end) {
	AccelProfile profiles;
	// profiles.accel = vector<MotionData>();
	// profiles.decel = vector<MotionData>();

	double cutoffStartTime = fabs(start.velocity / motionLimit.maxAcceleration);
	double cutoffStartDistance = 0.5 * motionLimit.maxAcceleration * cutoffStartTime * cutoffStartTime;

	double cutoffEndTime = fabs(end.velocity / motionLimit.maxAcceleration);
	double cutoffEndDistance = 0.5 * motionLimit.maxAcceleration * cutoffEndTime * cutoffEndTime;

	// Acceleration t = v/d
	double accelerationTime = motionLimit.maxVelocity / motionLimit.maxAcceleration;
	// Acceleration distance = 1/2at^2
	double accelerationDist = 0.5 * motionLimit.maxAcceleration * accelerationTime * accelerationTime;

	double fullDistance = cutoffStartDistance + fabs(end.distance - start.distance) + cutoffEndDistance;
	double maxDistance = fullDistance - 2 * accelerationDist;

	// When max speed is not able to be reached
	if (maxDistance < 0) {
		accelerationTime = sqrt(fabs(end.distance) / motionLimit.maxAcceleration);
		accelerationDist = 0.5 * motionLimit.maxAcceleration * accelerationTime * accelerationTime;
		maxDistance = 0;
	}

	profiles.accelerationDist = accelerationDist - cutoffStartDistance;
	profiles.accelerationTime = accelerationTime - cutoffStartTime;
	profiles.decelerationDist = accelerationDist - cutoffEndDistance;
	profiles.decelerationTime = accelerationTime - cutoffEndTime;

	cout << profiles.accelerationDist << "\t" << profiles.accelerationTime << "\t" << profiles.decelerationDist << "\t" << profiles.decelerationTime << "\n";

	double time = 0;
	while (time <= accelerationTime) {
		MotionData movement;
		movement.velocity = start.velocity + motionLimit.maxAcceleration * time;
		movement.distance = start.distance + start.velocity * time + 0.5 * motionLimit.maxAcceleration * time * time;
		movement.acceleration = motionLimit.maxAcceleration;
		movement.jerk = motionLimit.maxAcceleration / time;
		movement.time = start.time + time;

		if (movement.distance <= profiles.accelerationDist && movement.time <= profiles.accelerationTime)
			profiles.accelerationProfile.emplace_back(movement);
		if (movement.distance <= profiles.decelerationDist && movement.time <= profiles.decelerationTime)
			profiles.decelerationProfile.emplace_back(movement);

		cout << movement.velocity << "\t" << movement.distance << "\t" << time << "\n";

		time += 0.02;
	}

	return profiles;
}