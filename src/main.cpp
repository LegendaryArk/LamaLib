#include "main.h"

Inertial lamaLib::inertial(21);

MotorGroup leftMotors({
	{TOP_LEFT_CHASSIS, true, okapi::AbstractMotor::gearset::green, {0, 0, 0, 0}, okapi::AbstractMotor::encoderUnits::counts},
	{BOTTOM_LEFT_CHASSIS, true, okapi::AbstractMotor::gearset::green, {0, 0, 0, 0}, okapi::AbstractMotor::encoderUnits::counts}
});
MotorGroup rightMotors({
	{TOP_RIGHT_CHASSIS, false, okapi::AbstractMotor::gearset::green, {0, 0, 0, 0}, okapi::AbstractMotor::encoderUnits::counts},
	{BOTTOM_RIGHT_CHASSIS, false, okapi::AbstractMotor::gearset::green, {0, 0, 0, 0}, okapi::AbstractMotor::encoderUnits::counts}
});
Encoders trackingWheels {leftMotors.getMotors().at(0).getEncoder().get(), rightMotors.getMotors().at(0).getEncoder().get(), {REAR_TRACKING_UPPER, REAR_TRACKING_LOWER}, 900, 900, 360};
Chassis lamaLib::chassis(leftMotors, rightMotors, 0.1016, trackingWheels, 5.0 / 3.0);

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


	MotorGroup frontArm({{FRONT_ARM_LEFT, false, okapi::AbstractMotor::gearset::red},
						{FRONT_ARM_RIGHT, false, okapi::AbstractMotor::gearset::red}});

	Motor conveyor(CONVEYOR, false, okapi::AbstractMotor::gearset::blue);

	Motor backClaw(BACK_CLAW, false, okapi::AbstractMotor::gearset::red);

	Pneumatic frontClaw(pros::ADIDigitalOut(FRONT_CLAW));
	
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

	// Odom calibrate
	// RobotScales calibrated = chassis.calibrateOdom(master, inertial);
	// cout << calibrated.leftRadius << " " << calibrated.rightRadius << " " << calibrated.rearRadius << "\n";

	// Move velocity test
	// int count = 0;
	// double leftSum = 0, rightSum = 0;
	// while (count < 200) {
	// 	leftMotors.moveVelocity(100, 0.189, -19.191);
	// 	rightMotors.moveVelocity(100, 0.018, -20.053);
	// 	cout << leftMotors.getActualVelocity() << "\n";
	// 	count++;
	// 	pros::delay(10);
	// }
	// leftMotors.moveVelocity(0, 1, 0);
	// rightMotors.moveVelocity(0, 1, 0);
	// pros::delay(1000);
	// while (count < 400) {
	// 	leftMotors.moveVoltage(-2000);
	// 	rightMotors.moveVoltage(-2000);
	// 	count++;
	// 	pros::delay(10);
	// }
	// leftMotors.moveVelocity(0, 1, 0);
	// rightMotors.moveVelocity(0, 1, 0);
	// // Move distance test
	// chassis.moveDistance({1}, {{1.5, 1}}, {0});
	// chassis.moveDistance({-1}, {{1.5, 1}}, {0});
	// chassis.moveDistance({1, 1.5, 2.5}, {{1.5, 1}, {0.5, 0.5}, {1, 0.7}}, {0.5, 1, 0});

	// // Turn test
	// chassis.turnAbsolute(90, 1.5, {0.05, 0.001, 0.02, 1});
	// chassis.turnAbsolute(-90, 1.5, {0.05, 0.001, 0.02, 1});
	// chassis.turnRelative(90, 1.5, {0.05, 0.001, 0.02, 1});
	// chassis.turnRelative(-90, 1.5, {0.05, 0.001, 0.02, 1});

	int conveyorDir = 0;
	while (true) {
		
		int joyY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int joyX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		
		chassis.move(joyY + joyX, joyY - joyX);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
			frontClaw.toggle();
		
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			if(conveyorDir == 0)
				conveyorDir = 1;
			else
				conveyorDir = 0;
		}
		else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			if(conveyorDir == 0)
				conveyorDir = -1;
			else
				conveyorDir = 0;
		}
		conveyor.moveVelocity(600 * conveyorDir);
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			frontArm.moveVelocity(100);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
			frontArm.moveVelocity(-100);
		else
			frontArm.moveVelocity(0);
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
			backClaw.moveVelocity(100);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
			backClaw.moveVelocity(-100);
		else
			backClaw.moveVelocity(0);
		
		pros::delay(20);
	}
}
