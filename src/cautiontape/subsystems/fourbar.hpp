#pragma once

#include "motorgroup.hpp"
#include "..\controller\pid.hpp"
#include <cmath>

namespace lamaLib {
  class FourBar {
    public:
      FourBar(MotorGroup motors, double ratio, PIDValues pidValues);
      void setBrakeMode(okapi::AbstractMotor::brakeMode brakeInput);
      void startMove(double targetAngle);
      void endMove();
      void initialize(double current);
      void moveVelocity(int velocity);
      double getEncoder();
      double currentAngle;
      double offsetAngle;
      double target;
      double kp;
      double ki;
      double kd;
      double kf;
      double gearRatio;
      double iComp;
      bool moving = false;
      PIDController pidController;
      lamaLib::MotorGroup motors;
  };
}