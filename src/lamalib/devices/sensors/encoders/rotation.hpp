#pragma once

#include "api.h"
#include "encoder.hpp"

namespace lamaLib {
class Rotation : private pros::Rotation, public Encoder {
	public:
	/**
	 * @brief Rotation sensor, used to count the ticks the shaft that it is connected to
	 * 
	 * @param port The smart port that the smart cable is connected to (1-21)
	 * @param isReversed Whether the encoder is reversed or not
	 */
	Rotation(uint8_t port, bool isReversed);
	/**
	 * @brief Rotation sensor, used to count the ticks the shaft that it is connected to
	 * 
	 * @param rotation A pros rotation sensor
	 */
	Rotation(pros::Rotation rotation);

	/**
	 * @brief Gets the number of ticks of the rotation sensor
	 * 
	 * Analogous to getPosition()
	 * 
	 * @return The number of ticks
	 */
	double getTicks();

	/**
	 * @brief Resets the encoder ticks to 0
	 */
	void resetZero();

	/**
	 * @brief Gets the number of ticks of the rotation sensor
	 * 
	 * Analogous to getTicks()
	 * 
	 * @return The number of ticks
	 */
	double getPosition();
	/**
	 * @brief Sets the position or the number of ticks of the rotation sensor
	 * 
	 * @param position The new number of ticks
	 */
	void setPosition(int position);

	/**
	 * @brief Gets the angle of the current rotation in centidegrees, bounded between 0 and 36000
	 * 
	 * @return The angle of the current rotation in centidegrees
	 */
	double getAngle();

	/**
	 * @brief Gets the velocity in rpm
	 * 
	 * @return The velocity in rpm
	 */
	double getVelocity();
	
	/**
	 * @brief Gets whether the rotation sensor is reversed or not
	 * 
	 * @return true The sensor is reversed
	 * @return false The sensor is not reversed
	 */
	bool getReversed();
	/**
	 * @brief Sets the rotation sensor's reversed state
	 * 
	 * @param isReversed The new reversed state
	 */
	void setReversed(bool isReversed);

	/**
	 * @brief Sets how quickly the rotation sensor collects data
	 *
	 * The default is 10 ms, the minimum is 5 ms.
	 * Decreasing it lower than 10 ms will not increase the rate data is returned,
	 * but only to make sure the data is the latest.
	 * 
	 * @param rate The new rate in ms
	 */
	void setDataRate(int rate);
};
} // namespace lamaLib