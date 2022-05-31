#include "main.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

// Inertial lamaLib::inertial(21);

// MotorGroup leftMotors({
// 	{TOP_LEFT_CHASSIS, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts},
// 	{BOTTOM_LEFT_CHASSIS, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts}
// });
// MotorGroup rightMotors({
// 	{TOP_RIGHT_CHASSIS, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts},
// 	{BOTTOM_RIGHT_CHASSIS, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts}
// });
// Encoders trackingWheels {leftMotors.getMotors().at(0).getEncoder(), rightMotors.getMotors().at(0).getEncoder(), {REAR_TRACKING_UPPER, REAR_TRACKING_LOWER}, 900, 900, 360};
// Chassis lamaLib::chassis(leftMotors, rightMotors, LEFT_WHEEL_DIAMETER, RIGHT_WHEEL_DIAMETER, REAR_WHEEL_DIAMETER, trackingWheels, 3, 5.0 / 3.0);

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

void autonomous() {
// 	pros::vision_signature_s_t inputs[7] {
//     	pros::Vision::signature_from_utility(1, 1599, 3341, 2470, -4265, -3981,
//                                          -4123, 2.900, 0),
//     	pros::Vision::signature_from_utility(2, 0, 0, 0, 0, 0, 0, 3.000, 0),
//     	pros::Vision::signature_from_utility(3, 0, 0, 0, 0, 0, 0, 3.000, 0),
//     	pros::Vision::signature_from_utility(4, 0, 0, 0, 0, 0, 0, 3.000, 0),
//     	pros::Vision::signature_from_utility(5, 0, 0, 0, 0, 0, 0, 3.000, 0),
//     	pros::Vision::signature_from_utility(6, 0, 0, 0, 0, 0, 0, 3.000, 0),
//     	pros::Vision::signature_from_utility(7, 0, 0, 0, 0, 0, 0, 3.000, 0)
//   	};
//   lamaLib::VisionSensor visSensor(VISION);
//   visSensor.setSignatures(inputs);
//   while(visSensor.getCount()==0){
// 	  leftMotors.moveVelocity(20);
// 	  rightMotors.moveVelocity(-20);
//   }
//   	leftMotors.moveVelocity(0);
// 	rightMotors.moveVelocity(0);
// 	PIDGains move = {0.1, 0, 0, 0};
// 	PIDGains turn = {0.1, 0, 0, 0};
// 	PIDGains width = {0.1, 0, 0, 0};
// 	chassis.setVisionPID(turn, move, width);
// 	int debug;
// 	while (true) {
// 		debug = chassis.moveToVision(visSensor.getMiddle(1), 200, 20, 20, 30, visSensor.getWidth(1));
// 		pros::lcd::print(1, "Debug: %f", debug);
// 		if(debug == 3 || debug == 4){
// 			break;
// 		}
// 	}
}

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

	Motor topLeft(1, false, okapi::AbstractMotor::gearset::blue);
	Motor topRight(10, true, okapi::AbstractMotor::gearset::blue);
	Motor bottomLeft(11, false, okapi::AbstractMotor::gearset::blue);
	Motor bottomRight(20, true, okapi::AbstractMotor::gearset::blue);
	
	while (true) {
		double ch3 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double ch4 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		double ch1 = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		topLeft.moveVelocity(ch3 + ch1 + ch4);
		topRight.moveVelocity(ch3 - ch1 + ch4);
		bottomLeft.moveVelocity(ch3 + ch1 - ch4);
		bottomRight.moveVelocity(ch3 - ch1 + ch4);
	}
}
