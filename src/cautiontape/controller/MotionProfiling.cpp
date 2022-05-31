#include "MotionProfiling.hpp"

using namespace std;
using namespace lamaLib;

void MotionProfile::operator+=(MotionProfile rhs) {
	for (MotionData data : rhs.profile)
		profile.emplace_back(data);
}

MotionLimit MotionLimit::operator*(double rhs) {
	return {maxVelocity * rhs, maxAcceleration * rhs};
}
MotionLimit MotionLimit::operator/(double rhs) {
	return {maxVelocity / rhs, maxAcceleration / rhs};
}

MotionProfile lamaLib::generateTrapezoid(MotionLimit imotionLimit, CutoffPoint istart, CutoffPoint iend) {
	MotionProfile trapezoid;
	trapezoid.profile = vector<MotionData>();

	double cutoffStartTime = fabs(istart.velocity / imotionLimit.maxAcceleration);
	double cutoffStartDist = 0.5 * imotionLimit.maxAcceleration * cutoffStartTime * cutoffStartTime;

	double cutoffEndTime = fabs(iend.velocity / imotionLimit.maxAcceleration);
	double cutoffEndDist = 0.5 * imotionLimit.maxAcceleration * cutoffEndTime * cutoffEndTime;

	// Acceleration t = v/d
	double accelTime = imotionLimit.maxVelocity / imotionLimit.maxAcceleration;
	// Acceleration distance = 1/2at^2
	double accelDist = 0.5 * imotionLimit.maxAcceleration * accelTime * accelTime;
	double trapezoidDist = cutoffStartDist + fabs(iend.distance - istart.distance) + cutoffEndDist;
	double maxDist = trapezoidDist - 2 * accelDist;

	// When max speed is not able to be reached
	if (maxDist < 0) {
		accelTime = sqrt(fabs(iend.distance) / imotionLimit.maxAcceleration);
		maxDist = 0;
	}

	double endAccel = accelTime - cutoffStartTime;
	// maxDistance time = (distance / maxVel) - endAccel
    double endMax = endAccel + fabs(maxDist / imotionLimit.maxVelocity);
	// endDecel = maxDistance + endAccel
	double endDecel = endMax + accelTime - cutoffEndTime;

	cout << endAccel << "\t" << endMax << "\t" << endDecel << "\t" << accelTime << "\t" << cutoffStartTime << "\t" << cutoffEndTime << "\n";

	int direction = sign(iend.distance);
	imotionLimit.maxVelocity *= direction;
	imotionLimit.maxAcceleration *= direction;

	double time = 0;
	double prevTime = 0;

	// std::cout << "velocity,distance,time\n";
	while (time <= endDecel) {
		MotionData movement;

		double deltaTime = time - prevTime;

		if (time < endAccel) {
			movement.velocity = istart.velocity + imotionLimit.maxAcceleration * time;
			movement.distance = istart.distance + istart.velocity * time + 0.5 * imotionLimit.maxAcceleration * time * time;
			movement.acceleration = imotionLimit.maxAcceleration;
			movement.jerk = imotionLimit.maxAcceleration / time;
		} else if (time < endMax){
            movement.velocity = imotionLimit.maxVelocity;
            movement.distance = istart.distance + (istart.velocity * time + 0.5 * imotionLimit.maxAcceleration * accelTime * accelTime) + (imotionLimit.maxVelocity * (time - endAccel));
			movement.acceleration = 0;
			movement.jerk = 0;
        } else if (time <= endDecel) {
            double timeLeft = endDecel - time;
            movement.velocity = iend.velocity + imotionLimit.maxAcceleration * timeLeft;
            movement.distance = iend.distance - (0.5 * imotionLimit.maxAcceleration * timeLeft * timeLeft);
			movement.acceleration = -imotionLimit.maxAcceleration;
			movement.jerk = -imotionLimit.maxAcceleration / time;
        } else {
			movement.velocity = 0;
		}

		movement.time = istart.time + time;

		trapezoid.profile.emplace_back(movement);

		// std::cout << movement.velocity << "," << movement.distance << "," << movement.time << "\n";

		prevTime = time;
		time += 0.02;
	}

	return trapezoid;
}
