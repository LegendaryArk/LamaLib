#pragma once

#include "api.h"

namespace lamaLib {
class Pneumatic {
    public:
    Pneumatic(pros::ADIDigitalOut pneumatic);

    /**
     * @brief Sets the state of the pneumatic to either opened or closed
     * 
     * @param istate true = open; false = close
     */
    void setState(bool istate);
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
     * @brief Gets the state of the pneumatic
     * 
     * @return true 
     * @return false 
     */
    bool getState();

    private:
    pros::ADIDigitalOut pneumatic;

    bool state;
};
} // namespace lamaLib