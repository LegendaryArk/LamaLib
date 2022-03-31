#include "motionprofiling.hpp"

using namespace lamaLib;
using namespace std;

MotionProfile lamaLib::generateTrapezoid(MotionLimit imotionLimit, MotionData istart, MotionData iend) {
	MotionProfile trapezoid = {};
	trapezoid.profile = std::vector<MotionData>();

	double cutoffStartTime = fabs(istart.velocity / imotionLimit.maxAcceleration);
	double cutoffStartDist = 0.5 * imotionLimit.maxAcceleration * cutoffStartTime * cutoffStartTime;

	double cutoffEndTime = fabs(iend.velocity / imotionLimit.maxAcceleration);
	double cutoffEndDist = 0.5 * imotionLimit.maxAcceleration * cutoffEndTime * cutoffEndTime;

	// Acceleration t = v/d
	double accelTime = imotionLimit.maxVelocity / imotionLimit.maxAcceleration;
	// Acceleration distance = 1/2at^2
	double accelDist = 0.5 * imotionLimit.maxAcceleration * accelTime * accelTime;
	double trapezoidDist = cutoffStartDist + iend.distance - istart.distance + cutoffEndDist;
	double maxDist = trapezoidDist - 2 * accelDist;

	// When max speed is not able to be reached
	if (maxDist < 0) {
		accelTime = sqrt(iend.distance / imotionLimit.maxAcceleration);
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

	std::cout << "velocity,distance,time\n";
	while (time <= endDecel) {
		MotionData movement;

		double deltaTime = time - prevTime;

		if (time < endAccel) {
			movement.velocity = istart.velocity + imotionLimit.maxAcceleration * time;
			movement.distance = istart.distance + istart.velocity * time + 0.5 * imotionLimit.maxAcceleration * time * time;
		} else if (time < endMax){
            movement.velocity = imotionLimit.maxVelocity;
            movement.distance = istart.distance + (istart.velocity * time + 0.5 * imotionLimit.maxAcceleration * accelTime * accelTime) + (imotionLimit.maxVelocity * (time - endAccel));
        } else if (time <= endDecel) {
            double timeLeft = endDecel - time;
            movement.velocity = iend.velocity + imotionLimit.maxAcceleration * timeLeft;
            movement.distance = iend.distance - (0.5 * imotionLimit.maxAcceleration * timeLeft * timeLeft);
        } else {
			movement.velocity = 0;
		}

		movement.position.time = istart.position.time + time;

		trapezoid.profile.emplace_back(movement);

		std::cout << movement.velocity << "," << movement.distance << "," << movement.position.time << "\n";

		prevTime = time;
		time += 0.02;
	}

	return trapezoid;
}