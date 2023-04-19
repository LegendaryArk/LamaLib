#include "pose.hpp"

using namespace std;
using namespace lamaLib;

double Pose::distTo(Pose ipoint) {
    Pose diff = ipoint - *this;
    return hypot(diff.x, diff.y);
}

double Pose::angleTo(Pose ipoint) {
    Pose diff = ipoint - *this;
    return radToDeg(atan2(diff.y, diff.x));
}

Pose Pose::operator+(Pose rhs) {
    return {x + rhs.x, y + rhs.y, theta + rhs.theta, time + rhs.time};
}
Pose Pose::operator-(Pose rhs) {
    return {x - rhs.x, y - rhs.y, theta - rhs.theta, time - rhs.time};
}
