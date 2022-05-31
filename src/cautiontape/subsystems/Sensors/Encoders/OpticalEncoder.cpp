#include "OpticalEncoder.hpp"

using namespace lamaLib;

OpticalEncoder::OpticalEncoder(uint8_t topPort, uint8_t bottomPort, bool reversed)
							: pros::ADIEncoder(topPort, bottomPort, reversed) {}
OpticalEncoder::OpticalEncoder(uint8_t topPort, uint8_t bottomPort, uint8_t smartPort, bool reversed)
							: pros::ADIEncoder({smartPort, topPort, bottomPort}, reversed) {}
OpticalEncoder::OpticalEncoder(pros::ADIEncoder encoder)
							: pros::ADIEncoder(encoder) {}

double OpticalEncoder::getTicks() {
	return get_value();
}