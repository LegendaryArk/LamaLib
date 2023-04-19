#pragma once

#include "motor.hpp"
#include "../devices/sensors/encoders/encoder.hpp"
#include <vector>

namespace lamaLib {
class MotorGroup : public Encoder {
    std::vector<std::shared_ptr<Motor>> motors;

    public:
    /**
     * @brief A group of motors that have similar properties, such as always being powered at the same time, same gearbox, etc.
     * 
     * @param motors A vector of the motors that are in the group
     */
    MotorGroup(std::vector<std::shared_ptr<Motor>> motors);

    /**
     * @brief Sets all the motors to a specific velocity with a built-in PID
     * 
     * @param velocity The desired velocity in rpm
     */
    void moveVelocity(int velocity);
	/**
     * @brief Moves the robot at a certain velocity with a pid loop
     * 
     * @param velocity The desired velocity
	 * @param m The slope of the voltage to velocity equation
	 * @param b The y-intercept of the voltage to velocity equation
	 * @param pid The PID gains of the PID used to get the motor velocity up to the target velocity
     */
    void moveMotor(int velocity, double m = 1, double b = 0, PIDGains pid = {0, 0, 0, 1});
    /**
     * @brief Sets all the motors to a specific voltage
     * 
     * @param voltage The desired voltage in mV
     */
    void moveVoltage(int voltage);

	/**
	 * @brief Spins the motor to a target number of ticks relative to the zero position.
	 * 
	 * @param position The target position in ticks
	 * @param velocity The target velocity to power the motors in rpm
	 */
	void moveAbsolute(double position, double velocity);
	/**
	 * @brief Spins the motor to a target number of ticks relative to the current position.
	 * 
	 * @param position The target position in ticks
	 * @param velocity The target velocity to power the motors in rpm
	 */
	void moveRelative(double position, double velocity);

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
    std::vector<std::shared_ptr<Motor>> getMotors();

    /**
     * @brief Sets the brake mode of the motors
     * 
     * @param brakeMode The new brake mode value from okapi that is to be set to the motors
     */
    void setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode);
    /**
     * @brief Gets the brake mode of the motors
     * 
     * @return The motors' brakemode 
     */
    okapi::AbstractMotor::brakeMode getBrakeMode();

    bool isOverTemp();

    /**
     * @brief Sets the gearing
     * 
     * @param gearset The new gearset from okapi that is to be set to the motors
     */
    void setGearing(okapi::AbstractMotor::gearset gearset);
    /**
     * @brief Gets the gearset
     * 
     * @return the motors' internal gearset 
     */
    okapi::AbstractMotor::gearset getGearing();

	/**
	 * @brief Gets the number of ticks of the optical encoder
	 * 
	 * @return The number of ticks
	 */
	double getTicks() override;
	/**
	 * @brief Resets the encoder ticks to 0
	 */
	void resetZero() override;
};
} // namespace lamaLib