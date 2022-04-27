#include "mathhelper.hpp"

double degToRad(double deg) {
    return deg * (M_PI / 180);
}
double radToDeg(double rad) {
    return rad * (180 / M_PI);
}

double angleWrap180(double ang) {
    if (fmod(ang, 360) < 0)
        ang += 360;
    return ang;
}
double angleWrap360(double ang) {
    if (fmod(ang + 180, 360) < 0)
        ang += 360;
    return ang - 180;
}

int sign(double n) {
    return (n > 0) - (n < 0);
}
