#pragma once

#include "motor.hpp"

namespace lamaLib {
/**
 * @brief A structure to assist in returning the motor velocities
 */
typedef struct {
    double motorVels[];
} MotorVels;
class MotorGroup {
    public:
    /**
     * @brief A group of motors that have similar properties, such as always being powered at the same time, same gearbox, etc.
     * 
     * @param motors A vector of the motors that are in the group
     */
    MotorGroup(std::vector<Motor> motors);

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
     * @brief Gets the Actual Velocity of the motors in an array
     * 
     * @return MotorVels 
     */
    MotorVels getActualVelocity();

    /**
     * @brief Sets the brake mode of the motors
     * 
     * @param brakeMode The new brake mode value from okapi that is to be set to the motors
     */
    void setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode);
    /**
     * @brief Gets the brake mode of the motors
     * 
     * @return okapi::AbstractMotor::brakeMode 
     */
    okapi::AbstractMotor::brakeMode getBrakeMode();

    private:
    std::vector<Motor> motors;
};
} // namespace lamaLib