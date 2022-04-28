#pragma once

#include "robotconfig.hpp"
#include "cautiontape/lamaapi.hpp"

using namespace lamaLib;

extern MotorGroup leftMotors;
extern MotorGroup rightMotors;
extern Chassis lamaLib::chassis;

extern MotorGroup frontArm;
extern Pneumatic frontClaw;
extern Motor backClaw;
extern Motor conveyor;

extern okapi::Potentiometer armLimit;
extern Inertial lamaLib::inertial;

extern pros::Controller master;