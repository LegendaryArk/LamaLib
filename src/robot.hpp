#pragma once

#include "robotconfig.hpp"
#include "cautiontape/lamaapi.hpp"

using namespace lamaLib;

extern MotorGroup leftMotors;
extern MotorGroup rightMotors;
extern Chassis lamaLib::chassis;

extern MotorGroup frontArm;
extern Pneumatic frontClaw;
extern MotorGroup backClaw;
extern Motor conveyor;

extern okapi::Potentiometer armPos;
extern okapi::Potentiometer backClawPos;
extern Inertial lamaLib::inertial;

extern pros::Controller master;