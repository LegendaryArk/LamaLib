#pragma once

#include "robotconfig.hpp"
#include "lamalib/lamaapi.hpp"

using namespace lamaLib;

extern Motor leftMtr;
extern Motor rightMtr;
extern MotorGroup leftMtrs;
extern MotorGroup rightMtrs;

extern Rotation leftEnc;
extern Rotation rightEnc;
extern Rotation rearEnc;

extern Inline* chassis;