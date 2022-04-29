#include "main.h"

void initialize() {
	pros::lcd::initialize();
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

/*
pros::vision_signature_s_t inputs[7] {
    pros::Vision::signature_from_utility(1, 1599, 3341, 2470, -4265, -3981,
                                         -4123, 2.900, 0),
    pros::Vision::signature_from_utility(2, -2231, -1643, -1937, 7951, 9405,
                                         8678, 3.000, 0),
    pros::Vision::signature_from_utility(3, 5611, 8165, 6888, -1395, -979,
                                         -1187, 3.000, 0),
    pros::Vision::signature_from_utility(4, 0, 0, 0, 0, 0, 0, 3.000, 0),
    pros::Vision::signature_from_utility(5, 0, 0, 0, 0, 0, 0, 3.000, 0),
    pros::Vision::signature_from_utility(6, 0, 0, 0, 0, 0, 0, 3.000, 0),
    pros::Vision::signature_from_utility(7, 0, 0, 0, 0, 0, 0, 3.000, 0)
  };
  lamalib::visionSensor visSensor(1);
  visSensor.setSignatures(inputs);
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
void opcontrol() {}
