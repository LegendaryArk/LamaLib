#pragma once

#include "api.h"
#include "encoder.hpp"

namespace lamaLib {
class OpticalEncoder : private pros::ADIEncoder, public Encoder {
	public:
	/**
	 * @brief An optical shaft encoder, used for counting the ticks the low strength shaft this encoder is connected to
	 * 
	 * @param topPort The port the top 3-wire cable is connected to ('A'-'H', 'a'-'h', 1-8)
	 * @param bottomPort The port the bottom 3-wire cable is connected to ('A'-'H', 'a'-'h', 1-8)
	 * @param isReversed Whether the encoder is reversed or not
	 */
	OpticalEncoder(uint8_t topPort, uint8_t bottomPort, bool isReversed = false);
	/**
	 * @brief An optical shaft encoder, used for counting the ticks the low strength shaft this encoder is connected to
	 *
	 * This constructor should be used only when the optical encoder is connected to a 3-wire expander
	 * 
	 * @param smartPort The smart port the 3-wire expander is connected to (1-21)
	 * @param topPort The port the top 3-wire cable is connected to ('A'-'H', 'a'-'h', 1-8)
	 * @param bottomPort The port the bottom 3-wire cable is connected to ('A'-'H', 'a'-'h', 1-8)
	 * @param isReversed Whether the encoder is reversed or not
	 */
	OpticalEncoder(uint8_t smartPort, uint8_t topPort, uint8_t bottomPort, bool isReversed = false);
	/**
	 * @brief An optical shaft encoder, used for counting the ticks the low strength shaft this encoder is connected to
	 * 
	 * @param encoder The pros optical shaft encoder
	 */
	OpticalEncoder(pros::ADIEncoder encoder);

	/**
	 * @brief Gets the number of ticks of the optical encoder
	 * 
	 * @return The number of ticks
	 */
	double getTicks() override;
	/**
	 * @brief Resets the encoder ticks to 0
	 */
	void resetZero() override;
};
} // namespace lamaLib