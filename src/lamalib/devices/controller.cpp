#include "controller.hpp"

using namespace std;
using namespace lamaLib;

Controller::Controller(okapi::ControllerId id) : okapi::Controller(id) {}

bool Controller::getNewDigital(okapi::ControllerDigital button) {
	int index = static_cast<int>(button) - 6;
	if (index < 0 || index > 11) {
		cerr << "Invalid button\n";
		return false;
	}

	if (getDigital(button) && !btnDebounce[index]) {
		btnDebounce[index] = true;
		return true;
	} else if (!getDigital(button) && btnDebounce[index]) {
		btnDebounce[index] = false;
	}
	return false;
}