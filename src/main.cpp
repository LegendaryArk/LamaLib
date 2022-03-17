#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "cautiontape/subsystems/chassis.hpp"
#include "pros/adi.hpp"
#include "robotconfig.hpp"

Odometry lamaLib::odom = {
					{LEFT_TRACKING_UPPER, LEFT_TRACKING_LOWER, true},
					{RIGHT_TRACKING_UPPER, RIGHT_TRACKING_LOWER, true},
					{REAR_TRACKING_UPPER, REAR_TRACKING_LOWER, true},
					{2.75, 4.01042, 4.40573, 4.00354}, 360};

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	// pros::lcd::set_text(1, "Hello PROS User!");

	// pros::lcd::register_btn1_cb(on_center_button);
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	int8_t ports[4] = {
		TOP_LEFT_CHASSIS,
		BOTTOM_LEFT_CHASSIS,
		TOP_RIGHT_CHASSIS,
		BOTTOM_RIGHT_CHASSIS
	};
	bool reverseConfig[4] = {
		false, false, true, true
	};
	Chassis chassis(ports, reverseConfig, okapi::AbstractMotor::gearset::green);

	pros::IMU inertial(21);
	inertial.reset();
	while (inertial.is_calibrating()) pros::delay(10);

	// OdomScales calibrated = odom.calibrate(chassis, master, inertial);
	// cout << calibrated.leftRadius << " " << calibrated.rightRadius << " " << calibrated.rearRadius << "\n";

	while (true) {
		int joyY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int joyX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.move(joyY + joyX, joyY - joyX);
		pros::delay(20);
	}
}
