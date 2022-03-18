#include "inertial.hpp"

using namespace lamaLib;

void Inertial::reset() {
    inertial.tare();
}

double Inertial::getHeading() {
    int time = pros::millis();
    return inertial.get_heading() - (correction * time);
}
void Inertial::setHeading(double iangle) {
    inertial.set_heading(iangle);
}

void Inertial::calibrate() {
    inertial.reset();

    int count = 0;
    while (isCalibrating() && count < 50) {
        pros::delay(100);
        count++;

        if (count > 50) {
            std::cout << "Inertial calibration failed; please restart program\n";
            pros::lcd::print(3, "Inertial calibration failed; please restart program");
        }
    }
    pros::delay(2000);

    double val1 = getHeading();
    pros::delay(10000);
    double val2 = getHeading();
    correction = ((val1 - val2) / 10) / 1000;
}
bool Inertial::isCalibrating() {
    return inertial.is_calibrating();
}