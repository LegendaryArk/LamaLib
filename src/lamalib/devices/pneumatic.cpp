#include "pneumatic.hpp"

using namespace lamaLib;

Pneumatic::Pneumatic(uint8_t port, bool initState) : pneumatic(port, initState), isOpen(initState) {}
Pneumatic::Pneumatic(uint8_t smartPort, uint8_t port, bool initState) : pneumatic({smartPort, port}, initState), isOpen(initState) {}

void Pneumatic::open() {
	setState(true);
}
void Pneumatic::close() {
	setState(false);
}
void Pneumatic::toggle() {
	isOpen = !isOpen;
	setState(isOpen);
}

void Pneumatic::setState(bool state) {
	pneumatic.set_value(state);
	isOpen = state;
}
bool Pneumatic::getState() {
	return isOpen;
}