#include "opticalencoder.hpp"

using namespace lamaLib;

OpticalEncoder::OpticalEncoder(uint8_t topPort, uint8_t bottomPort, bool isReversed)
							: pros::ADIEncoder(topPort, bottomPort, isReversed) {
	ticksPerRotation = 360;
}
OpticalEncoder::OpticalEncoder(uint8_t smartPort, uint8_t topPort, uint8_t bottomPort, bool isReversed)
							: pros::ADIEncoder({smartPort, topPort, bottomPort}, isReversed) {
	ticksPerRotation = 360;
}
OpticalEncoder::OpticalEncoder(pros::ADIEncoder encoder)
							: pros::ADIEncoder(encoder) {
	ticksPerRotation = 360;
}

double OpticalEncoder::getTicks() {
	return get_value();
}
void OpticalEncoder::resetZero() {
	reset();
}