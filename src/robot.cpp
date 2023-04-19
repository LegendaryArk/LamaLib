#include "robot.hpp"

using namespace std;
using namespace lamaLib;

Motor leftMtr(1);
Motor rightMtr(2);
MotorGroup leftMtrs({make_shared<Motor>(leftMtr)});
MotorGroup rightMtrs({make_shared<Motor>(rightMtr)});

Rotation leftEnc(3, false);
Rotation rightEnc(4, false);
Rotation rearEnc(5, false);

Odometry odom({make_shared<Rotation>(leftEnc), make_shared<Rotation>(rightEnc), make_shared<Rotation>(rearEnc)}, {1, 1, 1, 1, 1, 1});

Inline* chassis = Inline::getChassis(make_shared<MotorGroup>(leftMtrs), make_shared<MotorGroup>(rightMtrs), {1, {okapi::AbstractMotor::gearset::blue, 1}}, make_shared<Odometry>(odom));