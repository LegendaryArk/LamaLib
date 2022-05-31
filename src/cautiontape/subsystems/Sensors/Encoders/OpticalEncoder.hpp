#pragma once

#include "api.h"
#include "Encoder.hpp"

namespace lamaLib {
class OpticalEncoder : public pros::ADIEncoder, public Encoder {
	public:
	OpticalEncoder(uint8_t topPort, uint8_t bottomPort, bool reversed = false);
	OpticalEncoder(uint8_t topPort, uint8_t bottomPort, uint8_t smartPort, bool reversed = false);
	OpticalEncoder(pros::ADIEncoder encoder);

	/**
	 * @brief Gets the number of ticks of the optical encoder
	 * 
	 * @return The number of ticks
	 */
	double getTicks();

	const int tpr = 360;
};
} // namespace lamaLib