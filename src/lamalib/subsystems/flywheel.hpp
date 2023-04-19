#pragma once

#include "../controller/pid.hpp"
#include "../devices/motorgroup.hpp"
#include <cmath>

namespace lamaLib {
class Flywheel {
    public:
    //define what motors will be used in flywheel class as well as pid values
    Flywheel(MotorGroup imotors, PIDGains pid, double iComp, double ivoltageM, double ivoltageB);

    int velocity;

     /**
     * @brief set velocity for motors used in flywheel
     * 
     * @param ivelocity power on a scale of 1-100%
     */
    //void moveVelocity(int velocity);
    
    /**
     * @brief Set the Velocity object
     * 
     * @param ivel the speed at which the motor will spin
     */
    void setVelocity(int ivel);

    /**
     * @brief start the flywheel task
     * 
     */
    void startFlywheel();

    /**
     * @brief end the flywheel task
     * 
     */
    void stopFlywheel();
    //void flywheelMain(double signal);

    std::pair<double, double> getVoltSlope();

    MotorGroup motors;

    PIDController pidController;

    private:
    pros::task_t flywheelTask;

    double FLYWHEEL_M;
    double FLYWHEEL_B;
};

/**
 * @brief 
 * 
 * @param flywheel 
 */
void flywheelMain(void* flywheel);
} // namespace lamaLib