#pragma once

#include "okapi/api.hpp"
#include "../../controller/motionprofiling.hpp"
#include "../../controller/odometry.hpp"
#include "../../controller/pid.hpp"

using namespace std;

namespace lamaLib {
/**
 * @brief The measurements of the chassis: the wheel diameter, the gear ratio, the max motor rpm depending on the gear cartridge, and the max rpm of the driven wheels
 */
struct ChassisScales {
    double wheelDiameter;
	okapi::AbstractMotor::GearsetRatioPair gearset {okapi::AbstractMotor::gearset::green, 1};

	double MOTOR_RPM = gearset.internalGearset == okapi::AbstractMotor::gearset::red ? 100 :
					gearset.internalGearset == okapi::AbstractMotor::gearset::green ? 200 :
																					600;
	double MAX_RPM = MOTOR_RPM * gearset.ratio;
};

class Chassis {
	public:
	/**
	 * @brief Moves forwards or backwards for a given amount of distance.
	 * 
	 * @param idist The distance in the unit that the wheel diameter is in.
	 * @param ipoints Starting velocity and ending velocities; used for cutoff; velocities are in the unit the wheel diameter is in per second.
	 */
	virtual void moveDistance(double idist, vector<MotionPoint> ipoints) = 0;

	/**
	 * @brief Turns the robot to a given angle
	 * 
	 * @param iangle The angle the robot should face towards in degrees.
	 * @param ivel The speed of the turn in the unit the wheel diameter is in per second.
	 * @param ipid The PID gains used for the turn
	 */
	virtual void turnAbsolute(double iangle, double ivel, PIDGains ipid) = 0;
	/**
	 * @brief Turns the robot by a given angle
	 * 
	 * @param iangle The number of degrees the robot should turn
	 * @param ivel The speed of the turn in the unit the wheel diameter is in per second.
	 * @param ipid The PID gains used for the turn
	 */
	virtual void turnRelative(double iangle, double ivel, PIDGains ipid) = 0;

	/**
	 * @brief Moves towards a certain point and turns to face an angle on the field using odometry or GPS sensor
	 * 
	 * @param itarget The target coordinate
	 * @param ipoints Starting velocity and ending velocities; used for cutoff; velocities are in the unit the wheel diameter is in per second.
	 * @param iturnVel The velocity of the turn in the unit the wheel diameter is in per second.
	 * @param iturnPid The PID gains used for the turn
	 */
	virtual void moveToPose(Pose itarget, vector<MotionPoint> ipoints, double iturnVel, PIDGains iturnPid) = 0;

	/**
	 * @brief Gets the brake mode of the motors
	 * Coast: can be pushed or pulled, the robot will continue moving from momentum after a movement
	 * Brake: will stop after a movement, but will still be able to be pushed or pulled afterwards
	 * Hold: will apply force so the motor stays in place, even after a movement
	 * 
	 * @return The brake mode of the motors
	 */
	virtual okapi::AbstractMotor::brakeMode getBrakeMode() = 0;
	/**
	 * @brief Sets the brake mode of the motors
	 * Coast: can be pushed or pulled, the robot will continue moving from momentum after a movement
	 * Brake: will stop after a movement, but will still be able to be pushed or pulled afterwards
	 * Hold: will apply force so the motor stays in place, even after a movement
	 * 
	 * @param ibrakeMode The brake mode of the motors
	 */
	virtual void setBrakeMode(okapi::AbstractMotor::brakeMode ibrakeMode) = 0;

	/**
     * @brief Get the chassis scales of the robot
     * 
     * @return The wheel diameter and gear ratio of the robot
     */
    ChassisScales getChassisScales();

	/**
	 * @brief Gets the odometry object to access the current pose and calibration methods
	 * 
	 * @return The odometry object that was created for the current chassis
	 */
	shared_ptr<Odometry> getOdom();

	protected:
	shared_ptr<Odometry> odom;

	ChassisScales scales;
};
} // namespace lamalib