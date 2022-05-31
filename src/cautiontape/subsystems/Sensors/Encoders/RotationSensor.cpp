#include "RotationSensor.hpp"

using namespace std;
using namespace lamaLib;

Rotation::Rotation(uint8_t port, bool reversed)
				: pros::Rotation(port) {
	setReversed(reversed);
}
Rotation::Rotation(pros::Rotation rotation)
				: pros::Rotation(rotation) {}

double Rotation::getTicks() {
	return get_position();
}

double Rotation::getPos() {
	return get_position();
}
void Rotation::setPos(int pos) {
	set_position(pos);
}

double Rotation::getAng() {
	return get_angle();
}

double Rotation::getVel() {
	// 600 because / 100 first to convert to degrees, and then / 6 to convert to rpm.
	return get_velocity() / 600.0;
}

bool Rotation::getReversed() {
	return get_reversed();
}
void Rotation::setReversed(bool state) {
	set_reversed(state);
}

void Rotation::setDataRate(int rate) {
	set_data_rate(rate);
}