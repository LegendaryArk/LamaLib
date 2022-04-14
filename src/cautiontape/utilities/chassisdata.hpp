#pragma once

#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"

using namespace std;

namespace lamaLib {
/**
 * @brief Tracking wheels that are used in odom and their TPR
 */
struct Encoders {
    shared_ptr<okapi::ContinuousRotarySensor> left {nullptr};
    shared_ptr<okapi::ContinuousRotarySensor> right {nullptr};
    pros::ADIEncoder rear {1, 1};
    double leftTPR {0};
    double rightTPR {0};
    double rearTPR {0};
};

/**
 * @brief The measurements of the tracking wheels in inches
 * 
 * Left and right are separate in the case that they are different
 */
struct RobotScales {
    double wheelDiameter;
    double leftRadius;
    double rightRadius;
    double rearRadius {0};
};
} // namespace lamaLib