#include "rotation.hpp"

using namespace std;
using namespace lamaLib;

Rotation::Rotation(uint8_t port, bool isReversed) : pros::Rotation(port) {
	setReversed(isReversed);

	ticksPerRotation = 36000;
}
Rotation::Rotation(pros::Rotation rotation) : pros::Rotation(rotation) {}

double Rotation::getTicks() {
	return get_position();
}
void Rotation::resetZero() {
	reset_position();
}

double Rotation::getPosition() {
	return get_position();
}
void Rotation::setPosition(int position) {
	set_position(position);
}

double Rotation::getAngle() {
	return get_angle();
}

double Rotation::getVelocity() {
	// 600 because / 100 first to convert to degrees, and then / 6 to convert to rpm.
	return get_velocity() / 600.0;
}

bool Rotation::getReversed() {
	return get_reversed();
}
void Rotation::setReversed(bool isReversed) {
	set_reversed(isReversed);
}

void Rotation::setDataRate(int rate) {
	set_data_rate(rate);
}