#include "chassis.hpp"

using namespace std;
using namespace lamaLib;

EncoderValues EncoderValues::operator-(EncoderValues rhs) {
    return {left - rhs.left, right - rhs.right, rear - rhs.rear};
}

Chassis::Chassis(MotorGroup ileftMotors, MotorGroup irightMotors, double iwheelDiameter, Encoders iencoders, int iinterval, double igearRatio) : leftMotors(ileftMotors), rightMotors(irightMotors), wheelDiameter(iwheelDiameter), encoders(iencoders), gearset(leftMotors.getGearing(), igearRatio) {
    interval = iinterval;
    leftMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    rightMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

    scales.gearRatio = igearRatio;
    scales.wheelDiameter = iwheelDiameter;
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
    // , leftROCs.at(0).slope, leftROCs.at(0).yIntercept
    leftMotors.moveVelocity(leftV);
    // , rightROCs.at(0).slope, rightROCs.at(0).yIntercept
    rightMotors.moveVelocity(rightV);
}

void Chassis::moveDistance(vector<double> idistances, vector<MotionLimit> imaxes, vector<double> iends, string rocKey) {
    if (leftROCs.find(rocKey) == leftROCs.end() || rightROCs.find(rocKey) == rightROCs.end()) {
        cerr << "Did not find key to rate of change in map\n";
        return;
    }
    if (idistances.size() != imaxes.size() || idistances.size() != iends.size()) {
        cerr << "Incorrect input sizes, vectors different sizes\n";
        return;
    }

    double gearRatio = gearset.ratio;
    MotionProfile profile = lamaLib::generateTrapezoid(imaxes.at(0) / gearRatio, {0}, {idistances.at(0) / gearRatio, iends.at(0) / gearRatio});
    for (int i = 1; i < idistances.size(); i++)
        profile += lamaLib::generateTrapezoid(imaxes.at(i) / gearRatio, {idistances.at(i - 1) / gearRatio, iends.at(i - 1) / gearRatio, profile.profile.at(i - 1).time}, {idistances.at(i) / gearRatio, iends.at(i) / gearRatio});

    for (MotionData vel : profile.profile) {
        double rpm = vel.velocity * 60 / (M_PI * wheelDiameter);
        cout << rpm << "," << vel.distance << "," << leftMotors.getActualVelocity() << "," << rightMotors.getActualVelocity() << "\n";
        // , leftROCs.at(rocKey).slope, leftROCs.at(rocKey).yIntercept, leftROCs.at(rocKey).pid
        // , rightROCs.at(rocKey).slope, rightROCs.at(rocKey).yIntercept, rightROCs.at(rocKey).pid
        leftMotors.moveVelocity(rpm);
        rightMotors.moveVelocity(rpm);
        
        pros::delay(20);
    }
    leftMotors.moveVelocity(0);
    rightMotors.moveVelocity(0);
}

void Chassis::turnAbsolute(double itarget, double imaxVel, string rocKey, PIDValues pidVals) {
    if (leftROCs.find(rocKey) == leftROCs.end() || rightROCs.find(rocKey) == rightROCs.end()) {
        cerr << "Did not find key to rate of change in map\n";
        return;
    }

    PIDController pidControl(pidVals, 1);
    while (fabs(itarget - pose.theta) > 2) {
        double pid = pidControl.calculatePID(pose.theta, itarget, 2);

        double rpm = imaxVel  * 60 / (M_PI * wheelDiameter);
        leftMotors.moveMotor(rpm * pid, leftROCs.at(rocKey).slope, leftROCs.at(rocKey).yIntercept, leftROCs.at(rocKey).pid);
        rightMotors.moveMotor(-rpm * pid, rightROCs.at(rocKey).slope, rightROCs.at(rocKey).yIntercept, rightROCs.at(rocKey).pid);

        pros::delay(10);
    }
}

void Chassis::turnRelative(double itarget, double imaxVel, string rocKey, PIDValues pidVals) {
    turnAbsolute(pose.theta + itarget, imaxVel, rocKey, pidVals);
}

void Chassis::moveToPose(Pose itarget, double turnVel, vector<double> cutoffDists, vector<MotionLimit> imaxes, vector<double> iends, string rocKey, PIDValues turnPID, bool reverse) {
    double angle = reverse ? pose.angleTo(itarget) + 180 : pose.angleTo(itarget);
    turnRelative(angle, turnVel, rocKey, turnPID);

    double totalDist = pose.distTo(itarget);
    cutoffDists.emplace_back(totalDist);
    if (reverse) {
        for (int i = 0; i < cutoffDists.size(); i++)
            cutoffDists.at(i) = -cutoffDists.at(i);
    }
    moveDistance(cutoffDists, imaxes, iends, rocKey);
}

void Chassis::addROC(string key, MotorROC leftROC, MotorROC rightROC) {
    leftROCs.emplace(key, leftROC);
    rightROCs.emplace(key, rightROC);
}

MotorGroup Chassis::getLeftMotors() {
    return leftMotors;
}
MotorGroup Chassis::getRightMotors() {
    return rightMotors;
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

RobotScales Chassis::calibrateOdom(pros::Controller controller, Inertial iinertial) {
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
    double leftDiameter = ((encoderVals.left / encoders.leftTPR) * getScales().wheelDiameter) * gearset.ratio / 10.0;
    double rightDiameter = ((encoderVals.right / encoders.rightTPR) * getScales().wheelDiameter) * gearset.ratio / 10.0;
    double rearDiameter = ((encoderVals.rear / encoders.rearTPR) * getScales().wheelDiameter) / 10.0;

    RobotScales calibratedScales = {getScales().wheelDiameter, fabs(leftDiameter / 2.0), fabs(rightDiameter / 2.0), fabs(rearDiameter / 2.0)};
    pros::lcd::print(5, "Calibrated radii:");
    pros::lcd::print(6, "left: %.2f in   right: %.2f in", calibratedScales.leftRadius, calibratedScales.rightRadius);
    pros::lcd::print(7, "rear: %.2f in", calibratedScales.rearRadius);
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

        cout << diffReadings.left << "\t" << diffReadings.right << "\t" << diffReadings.rear << "\n";
        chassis.setPose(odom.updatePose(chassis.getPose(), chassis.getScales(), chassis.getTrackingWheels(), {diffReadings.left, diffReadings.right, diffReadings.rear, 0}));

        pros::lcd::print(0, "x: %.2f in   y: %.2f in", chassis.getPose().x, chassis.getPose().y);
        pros::lcd::print(1, "theta: %.2f deg", chassis.getPose().theta);

        pros::lcd::print(2, "left: %.2f    right: %.2f", chassis.getEncoders().left, chassis.getEncoders().right);
        pros::lcd::print(3, "rear: %.2f", chassis.getEncoders().rear);
        
        pros::Task::delay_until(&time, 10);
    }
}