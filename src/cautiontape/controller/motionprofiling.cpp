#include "motionprofiling.hpp"

using namespace lamaLib;

MotionProfile lamaLib::generateTrapezoid(MotionLimit imotionLimit, double idistance) {
	MotionProfile trapezoid = {};
	trapezoid.profile = std::vector<MotionData>();

	// Acceleration time = initialVel + maxAccel / maxVel
	double accelTime = imotionLimit.maxVelocity / imotionLimit.maxAcceleration;
	// Acceleration distance = startDistance + initialVel * time + 0.5 * acceleration * t^2
	double accelDist = 0.5 * imotionLimit.maxAcceleration * accelTime * accelTime;

	double endAccel = accelTime;
	// maxDistance time = (distance / maxVel) - endAccel
    double endMax = accelTime + (idistance - 2 * accelDist) / imotionLimit.maxVelocity;
	// endDecel = maxDistance + endAccel
	double endDecel = endMax + accelTime;

	std::cout << endAccel << "\t" << endMax << "\t" << endDecel << "\n";

	double time = 0;
	double prevTime = 0;

	std::cout << "velocity,distance,time\n";
	while (time < endDecel) {
		MotionData data = {};

		double deltaTime = time - prevTime;

		if (time < endAccel) {
			data.velocity = imotionLimit.startVelocity + (imotionLimit.maxAcceleration * time);
			data.distance = 0.5 * imotionLimit.maxAcceleration * time * time;
		} else if (time < endMax){
            data.velocity = imotionLimit.maxVelocity;
            data.distance = (0.5 * imotionLimit.maxAcceleration * accelTime * accelTime) + (imotionLimit.maxVelocity * (time - endAccel));
        } else if (time <= endDecel) {
            double timeLeft = endDecel - time;
            data.velocity = imotionLimit.startVelocity + (imotionLimit.maxAcceleration * timeLeft);
            data.distance = idistance - (0.5 * imotionLimit.maxAcceleration * timeLeft * timeLeft);
        } else {
			data.velocity = 0;
		}

		trapezoid.profile.emplace_back(data);

		std::cout << data.velocity << "," << data.distance << "," << time << "\n";

		prevTime = time;
		time += 0.02;
	}

	return trapezoid;
}