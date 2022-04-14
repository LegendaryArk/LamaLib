#pragma once

#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"

using namespace std;

namespace lamaLib {
/**
 * @brief Tracking wheels that are used in odom and their TPR
 */
struct Encoders {
    okapi::ContinuousRotarySensor *left;
    okapi::ContinuousRotarySensor *right;
    pros::ADIEncoder rear;
    double leftTPR;
    double rightTPR;
    double rearTPR;
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