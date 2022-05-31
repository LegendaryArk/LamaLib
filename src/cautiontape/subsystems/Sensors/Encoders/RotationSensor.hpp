#pragma once

#include "api.h"
#include "Encoder.hpp"

namespace lamaLib {
class Rotation : public pros::Rotation, public Encoder {
	Rotation(uint8_t port, bool reversed);
	Rotation(pros::Rotation rotation);

	/**
	 * @brief Gets the number of ticks of the rotation sensor
	 * 
	 * Analogous to getPos()
	 * 
	 * @return The number of ticks
	 */
	double getTicks();

	/**
	 * @brief Gets the number of ticks of the rotation sensor
	 * 
	 * Analogous to get()
	 * 
	 * @return The number of ticks
	 */
	double getPos();
	/**
	 * @brief Sets the position or the number of ticks of the rotation sensor
	 * 
	 * @param pos The new number of ticks
	 */
	void setPos(int pos);

	/**
	 * @brief Gets the angle of the current rotation in centidegrees, bounded between 0 and 36000
	 * 
	 * @return The angle of the current rotation in centidegrees
	 */
	double getAng();

	/**
	 * @brief Gets the velocity in rpm
	 * 
	 * @return The velocity in rpm
	 */
	double getVel();
	
	/**
	 * @brief Gets whether the rotation sensor is reversed or not
	 * 
	 * @return true The sensor is reversed
	 * @return false The sensor is not reversed
	 */
	bool getReversed();
	/**
	 * @brief Sets the rotation sensor's reverse state
	 * 
	 * @param state The new state
	 */
	void setReversed(bool state);

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

	const int tpr = 36000;
};
} // namespace lamaLib