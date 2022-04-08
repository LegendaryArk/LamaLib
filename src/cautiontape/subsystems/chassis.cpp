#include "chassis.hpp"

using namespace std;
using namespace lamaLib;

Chassis::Chassis(MotorGroup ileftMotors, MotorGroup irightmotors, double igearRatio) : leftMotors(ileftMotors), rightMotors(irightmotors), gearset(leftMotors.getGearing(), igearRatio) {
    leftMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    rightMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
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

void Chassis::moveDistance(vector<Pose> itargets, vector<MotionLimit> imaxes, vector<MotionLimit> iends) {
    if (itargets.size() != imaxes.size() || itargets.size() != iends.size()) {
        cerr << "Incorrect input sizes, vectors too large\n";
        return;
    }

    double dist;
    double prevDists = dist;
    MotionProfile profile = lamaLib::generateTrapezoid(imaxes.at(0), {0}, {dist, iends.at(0).maxVelocity});
    for (int i = 1; i < itargets.size(); i++) {
        dist;

        profile += lamaLib::generateTrapezoid(imaxes.at(i), {prevDists, iends.at(i - 1).maxVelocity, profile.profile.at(i - 1).time}, {dist, iends.at(i).maxVelocity});

        prevDists += dist;
        pros::delay(20);
    }

    for (MotionData vel : profile.profile) {
        double rpm = vel.velocity * 60 / (PI);

        leftMotors.moveVelocity(rpm);
        rightMotors.moveVelocity(rpm);
    }
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
    return {encoders.left->get(), encoders.right->get(), encoders.rear->get()};
}

Pose Chassis::getPose() {
    return pose;
}
void Chassis::setPose(Pose ipose) {
    pose = ipose;
}

RobotScales Chassis::calibrate(Chassis ichassis, pros::Controller controller, pros::IMU iinertial) {
    uint32_t time = pros::millis();
    while (iinertial.get_rotation() < 3600) {
        ichassis.move(200, -200);

        pros::Task::delay_until(&time, 10);
    }

    ichassis.move(0, 0);

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
    return calibratedScales;
}

RobotScales Chassis::getScales() {
    return scales;
}
void Chassis::setScales(RobotScales iscales) {
    scales = iscales;
}

void Chassis::startOdom() {
    odomTask = pros::c::task_create(odometryMain, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odometry");
}
void Chassis::endOdom() {
    pros::c::task_delete(odomTask);
}

void odometryMain(void *iparam) {
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