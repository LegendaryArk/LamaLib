#pragma once

#include "cautiontape/lamaapi.hpp"
#include "robotconfig.hpp"

using namespace lamaLib;

extern Chassis lamaLib::chassis;

extern MotorGroup frontArm;
extern Pneumatic frontClaw;
extern Motor backClaw;
extern Motor conveyor;

extern okapi::Potentiometer armLimit;
extern lamaLib::Inertial inertial;

extern pros::Controller master;