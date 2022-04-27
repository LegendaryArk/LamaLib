#include "pose.hpp"

using namespace lamaLib;

double Pose::distTo(Pose ipoint) {
    Pose diff = ipoint - *this;
    return sqrt(pow(diff.x, 2) + pow(diff.y, 2));
}

double Pose::angleTo(Pose ipoint) {
    Pose diff = ipoint - *this;
    std::cout << diff.x << "\t" << diff.y << "\t" << radToDeg(std::atan2(diff.x, diff.y)) << "\n";
    return radToDeg(std::atan2(diff.x, diff.y));
}

Pose Pose::operator+(Pose rhs) {
    return {x + rhs.x, y + rhs.y, theta + rhs.theta, time + rhs.time};
}
Pose Pose::operator-(Pose rhs) {
    return {x - rhs.x, y - rhs.y, theta - rhs.theta, time - rhs.time};
}