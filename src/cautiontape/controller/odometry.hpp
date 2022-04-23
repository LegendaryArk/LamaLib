#pragma once

#include "api.h"
#include "../utilities/chassisdata.hpp"
#include "../utilities/mathhelper.hpp"
#include "../utilities/pose.hpp"

namespace lamaLib {

/**
 * @brief Some values used for odom for organization purposes.
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

    /**
     * @brief Updates the position coordinates in chassis
     * 
     * @param icurrentPose The current coordinate
     * @param iscales The robot measurements (wheel diameter, robot radii to the center of the robot, etc.)
     * @param iencoders The encoder wheels, only used to access their TPR
     * @param ireadingsDiff The difference in the readings since the last update
     * @return The updated position coordinate
     */
    Pose updatePose(Pose icurrentPose, RobotScales iscales, Encoders iencoders, OdomValues ireadingsDiff);

    private:
    pros::task_t odomTask {};
};
} // namespace lamaLib