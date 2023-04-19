#pragma once

#include "chassis.hpp"
#include "../../devices/motor.hpp"

namespace lamaLib {
class Mecanum : public Chassis {
	public:
	/**
	 * @brief Gets the chassis object or creates a new one if none was made before
	 * 
	 * @param ifrontLeft The front left motor
	 * @param ifrontRight The front right motor
	 * @param irearLeft The rear left motor
	 * @param irearRight The rear right motor
	 * @param ichassisScales The measurements for the chassis (wheel diameter and the gearing)
	 * @param iodom An Odometry object to keep track of the robot's coordinate
	 * @return A pointer to the chassis
	 */
	static Mecanum* getChassis(Motor ifrontLeft, Motor ifrontRight, Motor irearLeft, Motor irearRight, ChassisScales ichassisScales, shared_ptr<Odometry> iodom);
	/**
	 * @brief Gets the chassis object or creates a new one if none was made before
	 * 
	 * @param ifrontLeft The front left motor
	 * @param ifrontRight The front right motor
	 * @param irearLeft The rear left motor
	 * @param irearRight The rear right motor
	 * @param ichassisScales The measurements for the chassis (wheel diameter and the gearing) 
	 * @param iencoders The encoder sensors used for the tracking wheels
	 * @param iencoderScales The measurements of the tracking wheels (distance to the center and wheel diameter)
	 * @return A pointer to the chassis
	 */
	static Mecanum* getChassis(Motor ifrontLeft, Motor ifrontRight, Motor irearLeft, Motor irearRight, ChassisScales ichassisScales, Encoders iencoders, EncoderScales iencoderScales);

	private:
	Mecanum(Motor ifrontLeft, Motor ifrontRight, Motor irearLeft, Motor irearRight, ChassisScales ichassisScales, shared_ptr<Odometry> iodom);

	static Mecanum* chassis;

	public:
    /**
     * @brief Basic move function
     * 
     * @param ipower The velocity that the wheel moves forward at in rpm
     * @param itheta The angle in radians the chassis strafes in with 0 rad being 90 degrees or to the right
     * @param iturn The velocity that the chassis turns in rpm
     */
    void move(double ipower, double itheta, double iturn);

	/**
	 * @brief Drive the robot with a tank style
	 * 
	 * @param ileft The left joystick value from -1 to 1
	 * @param iright The right joystick value from -1 to 1
	 * @param ideadzone The area around the center of the controller
	 * 				where the controller won't actually do anything,
	 * 				fixing controller drift
	 */
	void tank(double ileft, double iright, double ideadzone = 0);
	/**
	 * @brief Drive the robot with an arcade style
	 * 
	 * @param iforward The joystick value of the y-axis from -1 to 1
	 * @param iturn The joystick value of the x-axis from -1 to 1
	 * @param ideadzone The area around the center of the controller
	 * 				where the controller won't actually do anything,
	 * 				fixing controller drift
	 */
	void arcade(double iforward, double iturn, double ideadzone = 0);
	/**
	 * @brief Drive the robot with a split arcade style
	 * 
	 * @param iforward The joystick value of the y-axis from -1 to 1
	 * @param istrafe The joystick value of x-axis from -1 to 1
	 * @param iturn The joystick value of the other x-axis from -1 to 1
	 * @param ideadzone The area around the center of the controller
	 * 				where the controller won't actually do anything,
	 * 				fixing controller drift
	 */
	void xArcade(double iforward, double istrafe, double iturn, double ideadzone = 0);

	/**
	 * @brief Moves in a certain direction for a given amount of distance.
	 * When not moving forward, the distance is not accurate.
	 * 
	 * @param idist The distance in the unit that the wheel diameter is in.
	 * @param ipoints Starting velocity and ending velocities; used for cutoff; velocities are in the unit the wheel diameter is in per second.
	 * @param idegree The degree the robot moves in, with 0 strafing to the right and 90 being forward.
	 */
    void moveDistance(double idist, vector<MotionPoint> ipoints, double idegree);
    /**
	 * @brief Moves forwards or backwards for a given amount of distance.
	 * 
	 * @param idist The distance in the unit that the wheel diameter is in.
	 * @param ipoints Starting velocity and ending velocities; used for cutoff; velocities are in the unit the wheel diameter is in per second.
	 */
	void moveDistance(double idist, vector<MotionPoint> ipoints) override;

	/**
	 * @brief Turns the robot to a given angle
	 * 
	 * @param iangle The angle the robot should face towards in degrees.
	 * @param ivel The speed of the turn in the unit the wheel diameter is in per second.
	 * @param ipid The PID gains used for the turn
	 */
	void turnAbsolute(double iangle, double ivel, PIDGains ipid) override;
	/**
	 * @brief Turns the robot by a given angle
	 * 
	 * @param iangle The number of degrees the robot should turn
	 * @param ivel The speed of the turn in the unit the wheel diameter is in per second.
	 * @param ipid The PID gains used for the turn
	 */
	void turnRelative(double iangle, double ivel, PIDGains ipid) override;

	/**
	 * @brief Moves to a certain coordinate and turns to face an absolute angle
	 * 
	 * @param ipose The coordinate and angle to get to
	 * @param ipoints The information for cutoff.
	 *				Each point consists of the max velocity, max acceleration,
	 *				max jerk, and the distance it starts/happens
	 * @param iturnVel The velocity to turn at
	 * @param iturnPid The PID gains of the turn
	 * @param idecelRangeOffset The offset to the range around the target pose to start decelerating at
	 */
	// void moveToPose(Pose ipose, vector<MotionPoint> ipoints, double iturnVel, PIDGains iturnPid, double idecelRangeOffset) override;

	/**
	 * @brief Moves towards a certain point and turns to face an angle on the field using odometry or GPS sensor
	 * 
	 * @param itarget The target coordinate
	 * @param ipoints Starting velocity and ending velocities; used for cutoff; velocities are in the unit the wheel diameter is in per second.
	 * @param iturnVel The velocity of the turn in the unit the wheel diameter is in per second.
	 * @param iturnPid The PID gains used for the turn
	 */
	void moveToPose(Pose itarget, vector<MotionPoint> ipoints, double iturnVel, PIDGains iturnPid) override;

	/**
	 * @brief Gets the brake mode of the motors
	 * 
	* @return The brake mode of the motors
	 */
	okapi::AbstractMotor::brakeMode getBrakeMode() override;
	/**
	 * @brief Sets the brake mode of the motors
	 * 
	 * @param ibrakeMode The brake mdoe of the motors
	 */
	void setBrakeMode(okapi::AbstractMotor::brakeMode ibrakeMode) override;

	private:
	Motor frontLeft;
	Motor frontRight;
	Motor rearLeft;
	Motor rearRight;
}; 
} // namespace lamaLib