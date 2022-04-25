#include "main.h"

void initialize() {
	pros::lcd::initialize();
	// pros::lcd::set_text(1, "Hello PROS User!");

	// pros::lcd::register_btn1_cb(on_center_button);

	// inertial.calibrate();
	// while (inertial.isCalibrating()) pros::delay(10);
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
	backClaw.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	
	// chassis.calibrateWheelDiameter(master, 2);
	// chassis.calibrateChassisDiameter(master, inertial);
	// chassis.setScales({GEAR_RATIO, LEFT_WHEEL_DIAMETER, RIGHT_WHEEL_DIAMETER, REAR_WHEEL_DIAMETER, LEFT_RADIUS, RIGHT_RADIUS, REAR_RADIUS});
	// chassis.startOdom();
	// chassis.turnAbsolute(90, 1, {0.0067, 0.002, 0.0025, 0});
	// chassis.moveToPose({0, 1}, 1, {}, {{1.45, 2.8}}, {0}, {0.0067, 0.002, 0.0025, 0});
	// chassis.moveToPose({1, 1}, 1, {}, {{1.45, 2.8}}, {0}, {0.0067, 0.002, 0.0025, 0});
	// chassis.moveToPose({1, 0}, 1, {}, {{1.45, 2.8}}, {0}, {0.0067, 0.002, 0.0025, 0});
	// chassis.moveToPose({0, 0}, 1, {}, {{1.45, 2.8}}, {0}, {0.0067, 0.002, 0.0025, 0});
	// // pros::delay(2000);
	// chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	
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

	// Move velocity test
	// int count = 0;
	// int mV = 12000;
	// double leftSum = 0, rightSum = 0;
	// while (count < 200) {
	// 	leftMotors.moveMotor(100, 0.0189, -19.191);
	// 	rightMotors.moveMotor(100, 0.018, -20.053);
	// 	// leftMotors.moveMotor(100, 0.0179, -22.772);
	// 	// rightMotors.moveMotor(100, 0.0178, -22.467);
	// 	cout << leftMotors.getActualVelocity() << "\t" << rightMotors.getActualVelocity() << "\n";
	// 	// leftMotors.moveVoltage(mV);
	// 	// rightMotors.moveVoltage(mV);
	// 	// if (count > 25) {
	// 	// 	leftSum += leftMotors.getActualVelocity();
	// 	// 	rightSum += rightMotors.getActualVelocity();
	// 	// }
	// 	count++;
	// 	pros::delay(10);
	// }
	// // cout << leftSum / 175 << "\t" << rightSum / 175 << "\n";
	// leftMotors.moveVelocity(0);
	// rightMotors.moveVelocity(0);
	// pros::delay(1000);
	// while (count < 400) {
	// 	leftMotors.moveMotor(-100, 0.0189, -19.191);
	// 	rightMotors.moveMotor(-100, 0.018, -20.053);
	// 	cout << leftMotors.getActualVelocity() << "\t" << rightMotors.getActualVelocity() << "\n";
	// 	// leftMotors.moveVoltage(-mV);
	// 	// rightMotors.moveVoltage(-mV);
	// 	count++;
	// 	pros::delay(10);
	// }
	// leftMotors.moveVelocity(0);
	// rightMotors.moveVelocity(0);

	// // Move distance test
	// chassis.addROC("0", {0.0189, -19.191, {0.05, 0.0001, 0.05, 1}}, {0.018, -20.053, {0.05, 0.0001, 0.05, 1}});
	// while (count < 200) {
	// 	leftMotors.moveMotor(100, 0.0189, -19.191, {0, 0, 0, 1});
	// 	rightMotors.moveMotor(100, 0.018, -20.053, {0.0001, 0, 0, 1});
	// 	cout << 100 << ","
	// 		<< leftMotors.getMotors().at(0).getActualVelocity() << ","
	// 		<< leftMotors.getMotors().at(1).getActualVelocity() << ","
	// 		<< rightMotors.getMotors().at(0).getActualVelocity() << ","
	// 		<< rightMotors.getMotors().at(1).getActualVelocity() << "\n";
	// 	count++;
	// 	pros::delay(10);
	// }
	// leftMotors.moveMotor(0);
	// rightMotors.moveMotor(0);
	// chassis.addROC("1", {0.0183, -22.301}, {0.0183, -23.267});
	// chassis.addROC("2", {0.0179, -22.772}, {0.0178, -22.467});
	// chassis.moveDistance({1}, {{1.45, 2.8}}, {0}, "0");//aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
	// chassis.moveDistance({-1.0 / (5.0/4.0)}, {{1.25, 2}}, {0}, "0");
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

		int leftSlew = chassis.lCalcSlew(joyY + joyX, 20);
		int rightSlew = chassis.rCalcSlew(joyY - joyX, 20);

		pros::lcd::print(1, "joyX %d", joyX);
		pros::lcd::print(2, "joyY %d", joyY);
		pros::lcd::print(3, "leftPower %d", leftSlew);
		pros::lcd::print(4, "rightPower %d", rightSlew);
		pros::lcd::print(5, "leftRPM %f", chassis.getLeftMotors().getActualVelocity());
		pros::lcd::print(6, "rightRPM %f", chassis.getRightMotors().getActualVelocity());

		// cout << leftPower << "\t" << rightPower << "\n";
		// if (armLimit.get() > 2000)
		// 	chassis.move(leftSlew, rightSlew);
		// else
		chassis.move(joyY + joyX, joyY - joyX);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
			frontClaw.toggle();
		
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			if(conveyorDir != 1)
				conveyorDir = 1;
			else
				conveyorDir = 0;
		}
		else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			if(conveyorDir != -1)
				conveyorDir = -1;
			else
				conveyorDir = 0;
		}
		conveyor.moveVelocity(600 * conveyorDir);
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			if (armLimit.get() < ARM_UPPER_LIMIT)
				frontArm.moveVelocity(100);
			else
				frontArm.moveVelocity(0);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			if (armLimit.get() > ARM_LOWER_LIMIT)
				frontArm.moveVelocity(-100);
			else
			 	frontArm.moveVelocity(0);
		} else
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
