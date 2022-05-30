#include "DiffChassis.hpp"

using namespace std;
using namespace lamaLib;

Chassis::Chassis(MotorGroup leftMotors, MotorGroup rightMotors, ChassisScales chassisScales, Encoders encoders, EncoderScales encoderScales, int interval)
				: leftMotors(leftMotors), rightMotors(rightMotors), chassisScales(chassisScales), odom({encoders, encoderScales}), interval(interval) {
    leftMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    rightMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

    fbTsk = pros::c::task_create(fourBarTask, (void*)0, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "FOURBARTASK");
}

void Chassis::move(int ileft, int iright) { //uses the pros controller which goes from -127 to 127
    int aleft = abs(ileft);
    int aright = abs(iright);

    if (aleft > 127)
        aleft = 127;
    if (aright > 127)
        aright = 127;

    //getting an absolute value to use for the joymap
    int leftV = joyMap[aleft] * sign(ileft);
    int rightV = joyMap[aright] * sign(iright);

    //applying gearbox values
    switch (gearset.internalGearset) {
        case okapi::AbstractMotor::gearset::red:
            leftV *= 1;
            rightV *= 1;
            break;
        case okapi::AbstractMotor::gearset::green:
            leftV *= 2;
            rightV *= 2;
            break;
        case okapi::AbstractMotor::gearset::blue:
            leftV *= 6;
            rightV *= 6;
            break;
        default:
            cerr << "Inavlid gearbox\n";
            break;
    }

    //send these values to the motors to move
    leftMotors.moveVelocity(leftV);
    rightMotors.moveVelocity(rightV);
}

void Chassis::moveDistance(vector<double> idistances, vector<MotionLimit> imaxes, vector<double> iends) {
    if (idistances.size() != imaxes.size() || idistances.size() != iends.size()) {
        cerr << "Incorrect input sizes, vectors different sizes\n";
        return;
    }

	double wheelDiameter = chassisScales.wheelDiameter;
    double gearRatio = chassisScales.gearset.ratio;

    MotionProfile profile = lamaLib::generateTrapezoid(imaxes.at(0) / gearRatio, {0}, {idistances.at(0) / gearRatio, iends.at(0) / gearRatio});
    for (int i = 1; i < idistances.size(); i++)
        profile += lamaLib::generateTrapezoid(imaxes.at(i) / gearRatio, {idistances.at(i - 1) / gearRatio, iends.at(i - 1) / gearRatio, profile.profile.at(i - 1).time}, {idistances.at(i) / gearRatio, iends.at(i) / gearRatio});

    for (MotionData vel : profile.profile) {
        double leftRPM = vel.velocity * 60 / (M_PI * wheelDiameter);
        double rightRPM = vel.velocity * 60 / (M_PI * wheelDiameter);
        // cout << leftRPM << "," << rightRPM << "," << vel.distance << "," << leftMotors.getActualVelocity() << "," << rightMotors.getActualVelocity() << "\n";
        leftMotors.moveVelocity(leftRPM);
        rightMotors.moveVelocity(rightRPM);
        
        pros::delay(20);
    }
    leftMotors.moveVelocity(0);
    rightMotors.moveVelocity(0);
}

void Chassis::turnAbsolute(double itarget, double imaxVel, PIDGains pidVals) {
	Pose pose = odom.getPose();
	double wheelDiameter = chassisScales.wheelDiameter;

    PIDController pidControl(pidVals);
    while (fabs(itarget - pose.theta) > 1) {
        double pid = pidControl.calculatePID(pose.theta, itarget, 1);
        
        double leftRPM = imaxVel * 60 / (M_PI * wheelDiameter);
        double rightRPM = imaxVel * 60 / (M_PI * wheelDiameter);
        leftMotors.moveVelocity(leftRPM * pid);
        rightMotors.moveVelocity(-rightRPM * pid);

        pros::delay(10);
    }
    leftMotors.moveVelocity(0);
    rightMotors.moveVelocity(0);
}

void Chassis::turnRelative(double itarget, double imaxVel, PIDGains pidVals) {
    turnAbsolute(odom.getPose().theta + itarget, imaxVel, pidVals);
}

void Chassis::moveToPose(Pose itarget, double turnVel, vector<Pose> cutoffPoses, vector<MotionLimit> imaxes, vector<double> iends, PIDGains turnPID, bool reverse, bool angleWrap) {
	Pose pose = odom.getPose();

    double angle = reverse ? pose.angleTo(itarget) + 180 : pose.angleTo(itarget);
    angle = angleWrap ? angleWrap180(angle) : angle;

    turnAbsolute(angle, turnVel, turnPID);
    
    vector<double> cutoffDists;
    for (int i = 0; i < cutoffPoses.size(); i++)
        cutoffDists.emplace_back(pose.distTo(cutoffPoses.at(i)));

    double totalDist = pose.distTo(itarget);
    cutoffDists.emplace_back(totalDist);
    if (reverse) {
        for (int i = 0; i < cutoffPoses.size(); i++)
            cutoffDists.at(i) = -cutoffDists.at(i);
    }

    moveDistance(cutoffDists, imaxes, iends);
    
    turnAbsolute(itarget.theta, turnVel, turnPID);
}

MotorGroup Chassis::getLeftMotors() {
    return leftMotors;
}
MotorGroup Chassis::getRightMotors() {
    return rightMotors;
}

void Chassis::setBrakeMode(okapi::AbstractMotor::brakeMode ibrakeMode) {
    leftMotors.setBrakeMode(ibrakeMode);
    rightMotors.setBrakeMode(ibrakeMode);
}
okapi::AbstractMotor::brakeMode Chassis::getBrakeMode() {
    return leftMotors.getBrakeMode();
}

ChassisScales Chassis::getChassisScales() {
    return chassisScales;
}
void Chassis::setChassisScales(ChassisScales scales) {
    chassisScales = scales;
}

EncoderScales Chassis::calibrateWheelDiameter(pros::Controller controller, double actualDist) {
    while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(10);

    EncoderTicks ticks = odom.getEncoderTicks();
	EncoderScales encoderScales = odom.getEncoderScales();
	double tpr = encoderScales.tpr;
	double leftWheelDiam = encoderScales.leftWheelDiameter;
	double rightWheelDiam = encoderScales.rightWheelDiameter;
	double rearWheelDiam = encoderScales.rearWheelDiameter;

    double leftWheelDist = ((ticks.left / tpr) * leftWheelDiam * M_PI);
    double rightWheelDist = ((ticks.right / tpr) * rightWheelDiam * M_PI);
    double rearWheelDist = ((ticks.rear / tpr) * rearWheelDiam * M_PI);

    double leftRatio = actualDist / leftWheelDist;
    double rightRatio = actualDist / rightWheelDist;
    double rearRatio = actualDist / rearWheelDist;

	double calibLeftWheelDiam = encoderScales.leftWheelDiameter * leftRatio;
	double calibRightWheelDiam = encoderScales.rightWheelDiameter * rightRatio;
	double calibRearWheelDiam = encoderScales.rearWheelDiameter * rearRatio;

    EncoderScales calibScales = {0, 0, 0, calibLeftWheelDiam, calibRightWheelDiam, calibRearWheelDiam};
    pros::lcd::print(4, "Calibrated diameters:");
    pros::lcd::print(5, "left: %f m", calibLeftWheelDiam);
    pros::lcd::print(6, "right: %f m", calibRightWheelDiam);
    pros::lcd::print(7, "rear: %f m", calibRearWheelDiam);
    cout << calibScales.leftWheelDiameter << "," << calibScales.rightWheelDiameter  << ", " << calibScales.rearWheelDiameter << "\n";

    return calibScales;
}

EncoderScales Chassis::calibrateChassisDiameter(pros::Controller controller, Inertial iinertial) {
    while (iinertial.getRotation() < 3600) {
        leftMotors.moveVelocity(-200);
        rightMotors.moveVelocity(200);

        pros::delay(10);
    }

    leftMotors.moveVelocity(0);
    rightMotors.moveVelocity(0);

    while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(10);

    EncoderTicks ticks = odom.getEncoderTicks();
	EncoderScales encoderScales = odom.getEncoderScales();
	double tpr = encoderScales.tpr;
	double leftWheelDiam = encoderScales.leftWheelDiameter;
	double rightWheelDiam = encoderScales.rightWheelDiameter;
	double rearWheelDiam = encoderScales.rearWheelDiameter;

    double leftDiameter = ((fabs(ticks.left) / tpr) * leftWheelDiam) / 10.0;
    double rightDiameter = ((fabs(ticks.right) / tpr) * rightWheelDiam) / 10.0;
    double rearDiameter = ((fabs(ticks.rear) / tpr) * rearWheelDiam) / 10.0;

    EncoderScales calibScales = {leftDiameter / 2.0, rightDiameter / 2.0, rearDiameter / 2.0, 0, 0, 0};

    pros::lcd::print(4, "Calibrated radii:");
    pros::lcd::print(5, "left: %f m", calibScales.leftRadius);
	pros::lcd::print(6, "right: %f m", calibScales.rightRadius);
    pros::lcd::print(7, "rear: %f m", calibScales.rearRadius);
    cout << calibScales.leftRadius << ", " << calibScales.rightRadius << ", " << calibScales.rearRadius << "\n";

    return calibScales;
}

void Chassis::setVisionPID(PIDGains track, PIDGains move, PIDGains width){
    visionTrackPID = track;
    visionMovePID = move;
    visionWidthPID = width;
}
int Chassis::moveToVision(int directionalTarget, int widthTarget, int moveSpeed, int turnSpeed, int widthMin, int width){
    double turnPID;
    double movePID;
    double widthPID;
    if(directionalTarget - 156 < -15 || directionalTarget - 156 > 15){
        turnPID = visionTrackPID.calculatePID( 0, directionalTarget - 156, 0);
        move(turnSpeed * turnPID, -turnSpeed * turnPID);
        return 1;
    }
    else if(width <= widthMin){
        move(0, 0);
        return 4;
    }
    else if(width < widthTarget){
        movePID = visionMovePID.calculatePID(0, directionalTarget - 156, 5);
        widthPID = visionWidthPID.calculatePID(width, widthTarget, 10);
        move((moveSpeed * movePID) * widthPID, (moveSpeed * -movePID) * widthPID);
        return 2;
    }
    else{
        move(0, 0);
        return 3;
    }
    /*
    if(width <= widthMin){
        return false;
    }
    while(width < widthTarget){
        movePID = visionMovePID.calculatePID(0, directionalTarget, double leeway)
    }*/
}

int Chassis::lCalcSlew(int itarget, int istep){
    if(counter < interval){
        counter++;
        return previousOutputL;
    }
    else{
        int output;
        if(abs(itarget - previousOutputL) > istep){
            output = previousOutputL + (sign(itarget-previousOutputL))*istep;
        }
        else{
            output = itarget;
        }
        previousOutputL = output;
        counter = 0;
        return output;
    }
}

int Chassis::rCalcSlew(int itarget, int istep){
    if(counter < interval){
        return previousOutputR;
    }
    else{
        int output;
        if(abs(itarget - previousOutputR) > istep){
            output = previousOutputR + (sign(itarget-previousOutputR))*istep;
        }
        else{
            output = itarget;
        }
        previousOutputR = output;
        return output;
    }
}

/*
void lamaLib::fourBarTask(void *iparam) {
    while (true) {
        for (int i = 0; i < chassis.fourBarList.size(); i++) {
            FourBar* fourBar = &chassis.fourBarList.at(i);
            if (fourBar->isMoving()) {
                double value = fourBar->getPIDController().calculatePID(fourBar->getEncoder(), fourBar->getTarget(), 5);
                value *= 120;
                if (value > 12000) {
                    value = 12000;
                } else if (value < -12000) {
                    value = -12000;
                }
                fourBar->getMotors().moveVoltage(value);
            } else {
                fourBar->getPIDController().resetPID();
            }
        }
        pros::delay(10);
    }
}

void Chassis::addFourBar(MotorGroup motors, double gearRatio, PIDGains pidValues) {
    FourBar fourBar(motors, gearRatio, pidValues);
    fourBarList.push_back(fourBar);
}
*/