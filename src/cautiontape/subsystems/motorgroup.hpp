#pragma once

#include "motor.hpp"
#include <vector>

namespace lamaLib {
class MotorGroup {
    public:
    /**
     * @brief A group of motors that have similar properties, such as always being powered at the same time, same gearbox, etc.
     * 
     * @param imotors A vector of the motors that are in the group
     */
    MotorGroup(std::vector<Motor> imotors);

    /**
     * @brief Sets all the motors to a specific velocity with a built-in PID
     * 
     * @param ivel The desired velocity in rpm
     */
    void moveVelocity(int ivel);
    /**
     * @brief Sets all the motors to a specific voltage
     * 
     * @param ivolt The desired voltage in mV
     */
    void moveVoltage(int ivolt);

    /**
     * @brief Gets the actual velocity of the motors
     * 
     * @return The velocity of the motors in rpm
     */
    double getActualVelocity();

    /**
     * @brief Gets the motors
     * 
     * @return A vector containing the motors in the motor group
     */
    vector<Motor> getMotors();

    /**
     * @brief Sets the brake mode of the motors
     * 
     * @param ibrakeMode The new brake mode value from okapi that is to be set to the motors
     */
    void setBrakeMode(okapi::AbstractMotor::brakeMode ibrakeMode);
    /**
     * @brief Gets the brake mode of the motors
     * 
     * @return The motors' brakemode 
     */
    okapi::AbstractMotor::brakeMode getBrakeMode();

    /**
     * @brief Sets the gearing
     * 
     * @param igearset The new gearset from okapi that is to be set to the motors
     */
    void setGearing(okapi::AbstractMotor::gearset igearset);
    /**
     * @brief Gets the gearset
     * 
     * @return the motors' internal gearset 
     */
    okapi::AbstractMotor::gearset getGearing();

    private:
    std::vector<Motor> motors;
};
} // namespace lamaLib