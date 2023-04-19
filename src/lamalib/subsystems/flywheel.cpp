#include "flywheel.hpp"

using namespace lamaLib;

/**
 * @brief Construct a new Flywheel:: Flywheel object
 * 
 * @param imotors Motor Group
 * @param pid kp, ki, kd and kf
 * @param iComp max integral addition value
 */
Flywheel::Flywheel(MotorGroup imotors, PIDGains pid, double iComp, double ivoltageM, double ivoltageB)
                : motors(imotors), pidController(pid, 1, -1, iComp), FLYWHEEL_M(ivoltageM), FLYWHEEL_B(ivoltageB) {}

/**
 * @brief set the velocity for the flywheel
 * 
 * @param ivel speed in RPM for the motor (0-600)
 */
void Flywheel::setVelocity(int ivel){
    velocity = ivel;
}

/**
 * @brief start the flywheel using a task
 * 
 */
void Flywheel::startFlywheel(){
    flywheelTask = pros::c::task_create(flywheelMain, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel");
    //odomTask = pros::c::task_create(odometryMain, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odoemtry");
}

/**
 * @brief stop the task controlling the flywheel
 * 
 */
void Flywheel::stopFlywheel(){
    pros::c::task_delete(flywheelTask);
}

std::pair<double, double> Flywheel::getVoltSlope() {
    return std::make_pair(FLYWHEEL_M, FLYWHEEL_B);
}

/**
 * @brief used to control the flywheel with a PID and given RPM
 * 
 * @param flywheel pointer to the flywheel
 */
void lamaLib::flywheelMain(void* flywheel){
    // define the pointer for the flywheel
    Flywheel* fly = (Flywheel*) flywheel;
    
    // calculate the signal
    double signal = fly -> pidController.calculatePID(fly -> motors.getActualVelocity(), fly -> velocity);
    uint32_t time = pros::millis();
    while(true){
        // calculate signal
        signal = fly -> pidController.calculatePID(fly -> motors.getActualVelocity(), fly ->velocity);
        
        // if the velocity is really low, disregard signal
        // if (abs(fly -> velocity) < 5) {
        //     signal = 0;
        // }

        // actually move the flywheel using move voltage
        std::pair<double, double> equation = fly->getVoltSlope();
        fly -> motors.moveVoltage((signal * 2000) + ((fly -> velocity * equation.first) + equation.second));

        //cout << fly -> motors.getActualVelocity() << "\n";

        // wait
        pros::Task::delay_until(&time, 10);
    }
}