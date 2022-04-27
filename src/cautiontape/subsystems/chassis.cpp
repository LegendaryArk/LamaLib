#include "chassis.hpp"

using namespace std;
using namespace lamaLib;

EncoderValues EncoderValues::operator+(EncoderValues rhs) {
    return {left + rhs.left, right + rhs.right, rear + rhs.rear};
}
EncoderValues EncoderValues::operator-(EncoderValues rhs) {
    return {left - rhs.left, right - rhs.right, rear - rhs.rear};
}

Chassis::Chassis(MotorGroup ileftMotors, MotorGroup irightMotors, double ileftWheelDiameter, double irightWheelDiameter, double irearWheelDiameter, Encoders iencoders, int iinterval, double igearRatio) : leftMotors(ileftMotors), rightMotors(irightMotors), encoders(iencoders), gearset(leftMotors.getGearing(), igearRatio) {
    interval = iinterval;
    leftMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    rightMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

    scales.gearRatio = igearRatio;
    scales.leftWheelDiameter = ileftWheelDiameter;
    scales.rightWheelDiameter = irightWheelDiameter;
    scales.rearWheelDiameter = irearWheelDiameter;
}

void Chassis::move(int left, int right) { //uses the pros controller which goes from -127 to 127
    int aleft = abs(left);
    int aright = abs(right);

    if (aleft > 127)
        aleft = 127;
    if (aright > 127)
        aright = 127;

    //getting an absolute value to use for the joymap
    int leftV = joyMap[aleft] * sign(left);
    int rightV = joyMap[aright] * sign(right);

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
            cout << "Inavlid gearbox\n";
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

    double gearRatio = gearset.ratio;
    MotionProfile profile = lamaLib::generateTrapezoid(imaxes.at(0) / gearRatio, {0}, {idistances.at(0) / gearRatio, iends.at(0) / gearRatio});
    for (int i = 1; i < idistances.size(); i++)
        profile += lamaLib::generateTrapezoid(imaxes.at(i) / gearRatio, {idistances.at(i - 1) / gearRatio, iends.at(i - 1) / gearRatio, profile.profile.at(i - 1).time}, {idistances.at(i) / gearRatio, iends.at(i) / gearRatio});

    for (MotionData vel : profile.profile) {
        double leftRPM = vel.velocity * 60 / (M_PI * scales.leftWheelDiameter);
        double rightRPM = vel.velocity * 60 / (M_PI * scales.rightWheelDiameter);
        // cout << leftRPM << "," << rightRPM << "," << vel.distance << "," << leftMotors.getActualVelocity() << "," << rightMotors.getActualVelocity() << "\n";
        leftMotors.moveVelocity(leftRPM);
        rightMotors.moveVelocity(rightRPM);
        
        pros::delay(20);
    }
    leftMotors.moveVelocity(0);
    rightMotors.moveVelocity(0);
}

void Chassis::turnAbsolute(double itarget, double imaxVel, PIDValues pidVals) {
    PIDController pidControl(pidVals);
    while (fabs(itarget - pose.theta) > 1) {
        double pid = pidControl.calculatePID(pose.theta, itarget, 1);

        double leftRPM = imaxVel * 60 / (M_PI * scales.leftWheelDiameter);
        double rightRPM = imaxVel * 60 / (M_PI * scales.rightWheelDiameter);
        leftMotors.moveVelocity(leftRPM * pid);
        rightMotors.moveVelocity(-rightRPM * pid);

        pros::delay(10);
    }
    leftMotors.moveVelocity(0);
    rightMotors.moveVelocity(0);
}

void Chassis::turnRelative(double itarget, double imaxVel, PIDValues pidVals) {
    turnAbsolute(pose.theta + itarget, imaxVel, pidVals);
}

void Chassis::moveToPose(Pose itarget, double turnVel, vector<double> cutoffDists, vector<MotionLimit> imaxes, vector<double> iends, PIDValues turnPID, bool reverse) {
    double angle = reverse ? pose.angleTo(itarget) + 180 : pose.angleTo(itarget);
    turnAbsolute(angle, turnVel, turnPID);

    double totalDist = pose.distTo(itarget);
    cutoffDists.emplace_back(totalDist);
    if (reverse) {
        for (int i = 0; i < cutoffDists.size(); i++)
            cutoffDists.at(i) = -cutoffDists.at(i);
    }
    moveDistance(cutoffDists, imaxes, iends);
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

Encoders Chassis::getTrackingWheels() {
    return encoders;
}
EncoderValues Chassis::getEncoders() {
    return {encoders.left->get(), encoders.right->get(), (double) encoders.rear.get_value()};
}

Pose Chassis::getPose() {
    return pose;
}
void Chassis::setPose(Pose ipose) {
    pose = ipose;
}

RobotScales Chassis::getScales() {
    return scales;
}
void Chassis::setScales(RobotScales iscales) {
    scales = iscales;
}

RobotScales Chassis::calibrateWheelDiameter(pros::Controller controller, double actualDist) {
    uint32_t time = pros::millis();
    while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::Task::delay_until(&time, 10);

    EncoderValues encoderVals = getEncoders();
    double gearRatio = gearset.ratio;
    double leftWheelDist = ((encoderVals.left / encoders.leftTPR) * scales.leftWheelDiameter * M_PI) * gearRatio;
    double rightWheelDist = ((encoderVals.right / encoders.rightTPR) * scales.rightWheelDiameter * M_PI) * gearRatio;
    double rearWheelDist = ((encoderVals.rear / encoders.rearTPR) * scales.rearWheelDiameter * M_PI) * gearRatio;

    double leftRatio = actualDist / leftWheelDist;
    double rightRatio = actualDist / rightWheelDist;
    double rearRatio = actualDist / rearWheelDist;

    RobotScales calibratedScales = {0, scales.leftWheelDiameter * leftRatio, scales.rightWheelDiameter * rightRatio, scales.rearWheelDiameter * rearRatio, 0, 0, 0};
    pros::lcd::print(5, "Calibrated diameters:");
    pros::lcd::print(6, "left: %.2f m   right: %.2f m", calibratedScales.leftWheelDiameter, calibratedScales.rightWheelDiameter);
    pros::lcd::print(7, "rear: %.2f m", calibratedScales.rearWheelDiameter);
    cout << calibratedScales.leftWheelDiameter << "," << calibratedScales.rightWheelDiameter  << ", " << calibratedScales.rearWheelDiameter << "\n";

    return calibratedScales;
}

RobotScales Chassis::calibrateChassisDiameter(pros::Controller controller, Inertial iinertial) {
	uint32_t time = pros::millis();
    while (iinertial.getRotation() < 3600) {
        leftMotors.moveVelocity(200);
        rightMotors.moveVelocity(-200);

        pros::Task::delay_until(&time, 10);
    }

    leftMotors.moveVelocity(0);
    rightMotors.moveVelocity(0);

    while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::Task::delay_until(&time, 10);

    EncoderValues encoderVals = getEncoders();
    double gearRatio = gearset.ratio;
    double leftDiameter = ((encoderVals.left / encoders.leftTPR) * scales.leftWheelDiameter) * gearRatio / 10.0;
    double rightDiameter = ((encoderVals.right / encoders.rightTPR) * scales.rightWheelDiameter) * gearRatio / 10.0;
    double rearDiameter = ((encoderVals.rear / encoders.rearTPR) * scales.rearWheelDiameter) / 10.0;

    RobotScales calibratedScales = {0, 0, 0, 0, fabs(leftDiameter / 2.0), fabs(rightDiameter / 2.0), fabs(rearDiameter / 2.0)};

    pros::lcd::print(5, "Calibrated radii:");
    pros::lcd::print(6, "left: %.2f m   right: %.2f m", calibratedScales.leftRadius, calibratedScales.rightRadius);
    pros::lcd::print(7, "rear: %.2f m", calibratedScales.rearRadius);
    cout << calibratedScales.leftRadius << ", " << calibratedScales.rightRadius << ", " << calibratedScales.rearRadius << "\n";

    return calibratedScales;
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

void Chassis::startOdom() {
    odomTask = pros::c::task_create(odometryMain, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
}
void Chassis::endOdom() {
    pros::c::task_delete(odomTask);
}

void lamaLib::odometryMain(void *iparam) {
    Odometry odom = {};

    EncoderValues currReadings = chassis.getEncoders();
    EncoderValues prevReadings = {0, 0, 0};

    uint32_t time = pros::millis();
    while (true) {
        prevReadings = currReadings;
        currReadings = chassis.getEncoders();
        EncoderValues diffReadings = currReadings - prevReadings;

        chassis.setPose(odom.updatePose(chassis.getPose(), chassis.getScales(), chassis.getTrackingWheels(), {diffReadings.left, diffReadings.right, diffReadings.rear, 0}));

        pros::lcd::print(0, "x: %.2f in   y: %.2f in", chassis.getPose().x, chassis.getPose().y);
        pros::lcd::print(1, "theta: %.2f deg", chassis.getPose().theta);

        pros::lcd::print(2, "left: %.2f    right: %.2f", chassis.getEncoders().left, chassis.getEncoders().right);
        pros::lcd::print(3, "rear: %.2f", chassis.getEncoders().rear);
        
        pros::Task::delay_until(&time, 10);
    }
}