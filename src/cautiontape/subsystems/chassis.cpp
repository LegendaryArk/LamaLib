#include "chassis.hpp"

using namespace std;
using namespace lamaLib;

Chassis::Chassis(MotorGroup ileftMotors, MotorGroup irightmotors, double iwheelDiameter, Encoders iencoders, double igearRatio) : leftMotors(ileftMotors), rightMotors(irightmotors), wheelDiameter(iwheelDiameter), encoders(iencoders), gearset(leftMotors.getGearing(), igearRatio) {
    leftMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    rightMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
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
    leftMotors.moveVelocity(leftV, leftROCs.at(0).slope, leftROCs.at(0).yIntercept);
    rightMotors.moveVelocity(rightV, rightROCs.at(0).slope, rightROCs.at(0).yIntercept);
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

    MotionProfile profile = lamaLib::generateTrapezoid(imaxes.at(0) / gearset.ratio, {0}, {idistances.at(0) / gearset.ratio, iends.at(0) / gearset.ratio});
    for (int i = 1; i < idistances.size(); i++)
        profile += lamaLib::generateTrapezoid(imaxes.at(i) / gearset.ratio, {idistances.at(i - 1) / gearset.ratio, iends.at(i - 1), profile.profile.at(i - 1).time / gearset.ratio}, {idistances.at(i) / gearset.ratio, iends.at(i) / gearset.ratio});

    for (MotionData vel : profile.profile) {
        double rpm = vel.velocity * 60 / (M_PI * wheelDiameter);
        cout << rpm << "\n";

        leftMotors.moveVelocity(rpm, leftROCs.at(0).slope, leftROCs.at(0).yIntercept);
        rightMotors.moveVelocity(rpm, rightROCs.at(0).slope, rightROCs.at(0).yIntercept);
        
        pros::delay(20);
    }
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
        leftMotors.moveVelocity(rpm * pid, leftROCs.at(0).slope, leftROCs.at(0).yIntercept);
        rightMotors.moveVelocity(-rpm * pid, rightROCs.at(0).slope, rightROCs.at(0).yIntercept);

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

void Chassis::addLeftROC(string key, MotorROC roc) {
    leftROCs.emplace(key, roc);
}
void Chassis::addRightROC(string key, MotorROC roc) {
    rightROCs.emplace(key, roc);
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
    while (iinertial.getYaw() < 3600) {
        move(200, -200);

        pros::Task::delay_until(&time, 10);
    }

    move(0, 0);

    while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        pros::Task::delay_until(&time, 10);
    }

    double leftDiameter = ((getEncoders().left / encoders.leftTPR) * getScales().wheelDiameter) / 10;
    double rightDiameter = ((getEncoders().right / encoders.rightTPR) * getScales().wheelDiameter) / 10;
    double rearDiameter = ((getEncoders().rear / encoders.rearTPR) * getScales().wheelDiameter) / 10;

    RobotScales calibratedScales = {getScales().wheelDiameter, leftDiameter / 2, rightDiameter / 2, rearDiameter / 2};
    pros::lcd::print(5, "Calibrated radii:");
    pros::lcd::print(6, "left: %f in   right: %f in", calibratedScales.leftRadius, calibratedScales.rightRadius);
    pros::lcd::print(7, "rear: %f in", calibratedScales.rearRadius);
    setScales(calibratedScales);
    return calibratedScales;
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

        chassis.setPose(odom.updatePose(chassis.getPose(), chassis.getScales(), chassis.getTrackingWheels(), {currReadings.left, currReadings.right, currReadings.rear, chassis.getPose().theta}));

        pros::lcd::print(0, "x: %.2f in   y: %.2f in", chassis.getPose().x, chassis.getPose().y);
        pros::lcd::print(1, "theta: %.2f deg", chassis.getPose().theta);

        pros::lcd::print(2, "left: %.2f    right: %.2f", chassis.getEncoders().left, chassis.getEncoders().right);
        pros::lcd::print(3, "rear: %.2f", chassis.getEncoders().rear);
        
        pros::Task::delay_until(&time, 10);
    }
}