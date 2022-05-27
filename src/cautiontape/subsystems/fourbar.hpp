#pragma once

#include "..\controller\PID.hpp"
#include "MotorGroup.hpp"
#include <cmath>

namespace lamaLib {
class FourBar {
    public:
    FourBar(MotorGroup motors, double ratio, PIDGains pidGains);
	
	void initialize(double current);
    
    void setBrakeMode(okapi::AbstractMotor::brakeMode brakeInput);
    
	void startMove(double targetAngle);
    void endMove();
	
	void moveVelocity(int velocity);

	MotorGroup getMotors();
    double getEncoder();
	
	double getTarget();

	PIDController getPIDController();

	bool isMoving();

    private:
	MotorGroup motors;

    double currentAngle;
    double offsetAngle;
    double target;

    PIDController pidController;
    PIDGains pidGains;
    
	double gearRatio;
    double iComp;
    bool moving = false;
};
} // namespace lamaLib