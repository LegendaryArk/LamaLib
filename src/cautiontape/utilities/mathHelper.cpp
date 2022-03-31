#include "mathHelper.hpp"

double degToRad(double deg) {
    return deg * (M_PI / 180);
}
double radToDeg(double rad) {
    return rad * (180 / M_PI);
}

int sign(double n) {
    return (n > 0) - (n < 0);
}