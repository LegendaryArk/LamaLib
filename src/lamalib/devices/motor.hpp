#pragma once

#include "okapi/api.hpp"
#include "../controller/pid.hpp"
#include "sensors/encoders/encoder.hpp"
#include "../utilities/mathhelper.hpp"

namespace lamaLib {
class Motor: public okapi::Motor, public Encoder {
	public:
	/**
	 * @brief A V5 Motor
	 * 
	 * @param port The smart port that the motor is connected to (1-21) This value can be replaced with a negative number to reverse the motor
	 */
	Motor(int8_t port);
	/**
	 * @brief A V5 Motor
	 * 
	 * @param port The smart port that the motor is connected to (1-21)
	 * @param isReversed Whether the motor is reversed or not
	 * @param gearset The internal motor cartridge
	 * @param encoderUnits The encoder units that is used to keep track of how far the motor has spun
	 */
	Motor(int8_t port, bool isReversed, AbstractMotor::gearset gearset, AbstractMotor::encoderUnits encoderUnits = AbstractMotor::encoderUnits::counts);

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
	 * @brief Gets the number of ticks of the motor's internal encoder
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