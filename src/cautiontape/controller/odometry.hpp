#pragma once

#include "api.h"
#include "../utilities/chassisdata.hpp"
#include "../utilities/mathhelper.hpp"
#include "../utilities/pose.hpp"

namespace lamaLib {

/**
 * @brief 
 */
struct OdomValues {
    double left;
    double right;
    double rear;
    double theta;
};

class Odometry {
    public:
    /**
     * @brief Odometry to keep track of the robot's position
     */
    Odometry();

    Pose updatePose(Pose icurrentPose, RobotScales iscales, Encoders iencoders, OdomValues ireadingsDiff);

    private:
    pros::task_t odomTask {};
};
} // namespace lamaLib