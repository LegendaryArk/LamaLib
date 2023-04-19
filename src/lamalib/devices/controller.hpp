#pragma once

#include "okapi/api.hpp"

namespace lamaLib {
class Controller : public okapi::Controller {
	bool btnDebounce[12];
	
	public:
	Controller(okapi::ControllerId id = okapi::ControllerId::master);

	/**
	 * @brief Returns whether the digital button has been pressed. It will only return true once per press.
	 * Returns false if the controller is not connected.
	 * 
	 * @param button The button to check 
	 * @return True if the button has not be pressed before, false otherwise. 
	 */
	bool getNewDigital(okapi::ControllerDigital button);
};
} // namespace lamaLib