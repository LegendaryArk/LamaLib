#include "pid.hpp"

using namespace std;
using namespace lamaLib;

PIDController::PIDController(PIDGains pidValues, double maximum, double minimum, double integralCompensation, double leeway)
							: pidValues(pidValues) {
	this->maximum = maximum;
	this->minimum = minimum;
	this->integralCompensation = integralCompensation;
	this->leeway = leeway;
}

double PIDController::calculatePID(double current, double target) {
	double error = target - current;
	//double a = 0; //any value from 0 - 1
	double previousEstimate = 0;
	double currentEstimate = 0;
	// cout << error << endl;

	if (fabs(error) <= leeway) {
		count++;
		if (count > 4)
			flag_c = true;
	} else {
		count = 0;
	}

	currentEstimate = (pidValues.a * previousEstimate) + (1 - pidValues.a) * error;
	
	double derivative = currentEstimate / pros::millis();
	double newIntegral = fabs(error) < integralCompensation ? integral + error : 0;

	previousError = error;
	previousEstimate = currentEstimate;

	double signal = error * pidValues.kp + integral * pidValues.ki + derivative * pidValues.kd + pidValues.kf;
	if (signal > maximum)
		signal = maximum;
	else if (signal < minimum)
		signal = minimum;
	else
		integral = newIntegral;

	return signal;
}

bool PIDController::isComplete() {
	//bool flag;//set to a value
	return flag_c;
}

void PIDController::setGains(PIDGains pidValues) {
	this->pidValues = pidValues;
}
PIDGains PIDController::getGains() {
	return pidValues;
}

double PIDController::getIntegral() {
	return integral;
}

void PIDController::setIntegralCompensation(double integralCompensation) {
	this->integralCompensation = integralCompensation;
}
double PIDController::getIntegralComp() {
	return integralCompensation;
}

void PIDController::setMaximum(double maximum) {
	this->maximum = maximum;
}
double PIDController::getMaximum() {
	return maximum;
}

void PIDController::setMinimum(double minimum) {
	this->minimum = minimum;
}
double PIDController::getMinimum() {
	return minimum;
}

double PIDController::getCount() {
	return count;
}

double PIDController::getprevError() {
	return previousError;
}

void PIDController::resetPID() { 
	integral = 0;
	previousError = 0;
	count = 0;
	flag_c = false;
}

void PIDController::PIDTuner(bool x, PIDGains pidValues) {
	if (x) {
		pidValues.kp += pidValues.kp;
		pidValues.ki += pidValues.ki;
		pidValues.kd += pidValues.kd;
		pidValues.kf += pidValues.kf;
		pidValues.a += pidValues.a;
		//cout << pidValues.kp;
	}
}