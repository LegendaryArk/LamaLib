#pragma once

namespace lamaLib {
class Encoder {
	protected:
	int ticksPerRotation;
	
	public:
	/**
	 * @brief Gets the number of ticks of the optical encoder
	 * 
	 * @return The number of ticks
	 */
	virtual double getTicks() = 0;
	/**
	 * @brief Resets the encoder ticks to 0
	 */
	virtual void resetZero() = 0;

	/**
	 * @brief Gets the ticks per rotation of the encoder
	 * 
	 * @return The number of ticks in one rotation
	 */
	int getTicksPerRotation();
};
} // namespace lamaLib