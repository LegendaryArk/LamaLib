#pragma once

#include "api.h"

namespace lamaLib {
class Pneumatic {
	pros::ADIDigitalOut pneumatic;

	bool isOpen;

	public:
	/**
	 * @brief Pneumatics use compressed air to produce motion. They only have 2 states, open and closed.
	 * 
	 * @param port The port the top 3-wire cable is connected to ('A'-'H', 'a'-'h', 1-8)
	 * @param initState Whether the pneumatic starts open or close; true = open; false = close
	 */
	Pneumatic(uint8_t port, bool initState = false);
	/**
	 * @brief Pneumatics use compressed air to produce motion. They only have 2 states, open and closed.
	 *
	 * This constructor should be used only when the pneumatic is connected to a 3-wire expander
	 * 
	 * @param smartPort The smart port the 3-wire expander is connected to (1-21)
	 * @param port The port the top 3-wire cable is connected to ('A'-'H', 'a'-'h', 1-8)
	 * @param initState Whether the pneumatic starts open or close; true = open; false = close
	 */
	Pneumatic(uint8_t smartPort, uint8_t port, bool initState = false);

	/**
	 * @brief Opens the pneumatic
	 */
	void open();
	/**
	 * @brief Closes the pneumatic
	 */
	void close();
	/**
	 * @brief Toggles the pneumatic between opened and closed 
	 */
	void toggle();

	/**
	 * @brief Sets the state of the pneumatic to either opened or closed
	 * 
	 * @param state true = open; false = close
	 */
	void setState(bool state);
	/**
	 * @brief Gets the state of the pneumatic
	 * 
	 * @return true for open
	 * @return false for closed
	 */
	bool getState();
};
} // namespace lamaLib