#include "mathhelper.hpp"

using namespace std;
using namespace lamaLib;

double lamaLib::degToRad(double deg) {
    return deg * (M_PI / 180);
}
double lamaLib::radToDeg(double rad) {
    return rad * (180 / M_PI);
}

double lamaLib::inToCm(double in) {
    return in * 2.54;
}
double lamaLib::cmToIn(double cm) {
    return cm / 2.54;
}

double lamaLib::ftToCm(double ft) {
    return ft * 30.48;
}
double lamaLib::cmToFt(double cm) {
    return cm / 30.48;
}

double lamaLib::inToM(double in) {
    return in / 39.37;
}
double lamaLib::mToIn(double m) {
    return m * 39.37;
}

double lamaLib::ftToM(double ft) {
    return ft / 3.281;
}
double lamaLib::mToFt(double m) {
    return m * 3.281;
}

double lamaLib::angleWrap180(double ang) {
	ang = fmod(ang + 180, 360);
    if (ang < 0)
        ang += 360;
    return ang - 180;
}
double lamaLib::angleWrap360(double ang) {
	ang = fmod(ang, 360);
    if (ang < 0)
        ang += 360;
    return ang;
}

int lamaLib::sign(double n) {
    return (n > 0) - (n < 0);
}
