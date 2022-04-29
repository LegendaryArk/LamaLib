#include "pneumatic.hpp"

using namespace lamaLib;

Pneumatic::Pneumatic(pros::ADIDigitalOut pneumatic, bool initState) : pneumatic(pneumatic), state(initState) {}

void Pneumatic::setState(bool istate) {
    pneumatic.set_value(istate);
    state = istate;
}
void Pneumatic::open() {
    setState(true);
}
void Pneumatic::close() {
    setState(false);
}
void Pneumatic::toggle() {
    setState(state);
    state = !state;
}

bool Pneumatic::getState() {
    return state;
}