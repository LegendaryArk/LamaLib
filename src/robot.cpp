#include "robot.hpp"

using namespace lamaLib;

MotorGroup leftMotors({
	{TOP_LEFT_CHASSIS, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts},
	{BOTTOM_LEFT_CHASSIS, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts}
});
MotorGroup rightMotors({
	{TOP_RIGHT_CHASSIS, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts},
	{BOTTOM_RIGHT_CHASSIS, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts}
});
Encoders trackingWheels {leftMotors.getMotors().at(0).getEncoder(), rightMotors.getMotors().at(0).getEncoder(), {REAR_TRACKING_UPPER, REAR_TRACKING_LOWER}, 900, 900, 360};
Chassis lamaLib::chassis(leftMotors, rightMotors, LEFT_WHEEL_DIAMETER, RIGHT_WHEEL_DIAMETER, REAR_WHEEL_DIAMETER, trackingWheels, 3, 5.0 / 3.0);

MotorGroup frontArm({{FRONT_ARM_LEFT, true, okapi::AbstractMotor::gearset::red},
					{FRONT_ARM_RIGHT, false, okapi::AbstractMotor::gearset::red}});
Pneumatic frontClaw(pros::ADIDigitalOut(FRONT_CLAW), true);
MotorGroup backClaw({{BACK_CLAW, false, okapi::AbstractMotor::gearset::red}});
Motor conveyor(CONVEYOR, false, okapi::AbstractMotor::gearset::blue);

okapi::Potentiometer armPos(ARM_POTENTIOMETER);
okapi::Potentiometer backClawPos(BACK_POTENTIOMETER);
Inertial lamaLib::inertial(INERTIAL);

pros::Controller master(pros::E_CONTROLLER_MASTER);