#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "robotconfig.hpp"

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

	MotorGroup leftMotors({
		{TOP_LEFT_CHASSIS, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts},
		{BOTTOM_LEFT_CHASSIS, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts}
	});
	MotorGroup rightMotors({
		{TOP_RIGHT_CHASSIS, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts},
		{BOTTOM_RIGHT_CHASSIS, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts}
	});
	Chassis chassis(leftMotors, rightMotors, 0.1016, 1);

	MotorGroup frontArm(
		{{FRONT_LEFT_ARM, true, okapi::AbstractMotor::gearset::red},
		{FRONT_RIGHT_ARM, false, okapi::AbstractMotor::gearset::red}}
	);
	Motor backArm(BACK_ARM, false, okapi::AbstractMotor::gearset::red);
	Motor conveyor(CONVEYOR, false, okapi::AbstractMotor::gearset::blue);

	Pneumatic frontClaw(pros::ADIDigitalOut({1, FRONT_CLAW}));
	Pneumatic backClaw(pros::ADIDigitalOut({1, BACK_CLAW}));

	// chassis.moveDistance({1}, {{1.5, 1}}, {{0, 0}});
	
	// MotionProfile trapezoid = lamaLib::generateTrapezoid({0.75, 0.5}, {0, 0}, {1, 0.75});
	// MotionProfile trapezoid2 = lamaLib::generateTrapezoid({0.5, 1}, {1, 0.75, trapezoid.profile.at(trapezoid.profile.size() - 1).time}, {1.5, 0});
	
	// for (MotionData movement : trapezoid.profile) {
	// 	double rpm = movement.velocity * 60 / (PI * 0.1016);
	// 	chassis.getLeftMotors().moveVelocity(rpm);
	// 	chassis.getRightMotors().moveVelocity(rpm);
	// 	cout << chassis.getLeftMotors().getActualVelocity() << ", " << chassis.getRightMotors().getActualVelocity() << ", " << rpm << "\n";
	// 	pros::delay(20);
	// }
	// for (MotionData movement : trapezoid2.profile) {
	// 	double rpm = movement.velocity * 60 / (PI * 0.1016);
	// 	chassis.getLeftMotors().moveVelocity(rpm);
	// 	chassis.getRightMotors().moveVelocity(rpm);
	// 	cout << chassis.getLeftMotors().getActualVelocity() << ", " << chassis.getRightMotors().getActualVelocity() << ", " << rpm << "\n";
	// 	pros::delay(20);
	// }

	// pros::IMU inertial(21);
	// inertial.reset();
	// while (inertial.is_calibrating()) pros::delay(10);

	// OdomScales calibrated = odom.calibrate(chassis, master, inertial);
	// cout << calibrated.leftRadius << " " << calibrated.rightRadius << " " << calibrated.rearRadius << "\n";

	int conveyorDir = 0;
	while (true) {
		int joyY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int joyX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.move(joyY + joyX, joyY - joyX);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
			frontClaw.toggle();
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
			backClaw.toggle();

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
			conveyorDir = 1;
		else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
			conveyorDir = -1;
		conveyor.moveVelocity(600 * conveyorDir);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			frontArm.moveVelocity(100);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
			frontArm.moveVelocity(-100);
		else
			frontArm.moveVelocity(0);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
			backArm.moveVelocity(100);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
			backArm.moveVelocity(-100);
		else
			backArm.moveVelocity(0);
		
		pros::delay(20);
	}
}
