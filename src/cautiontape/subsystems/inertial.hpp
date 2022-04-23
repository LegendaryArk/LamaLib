#pragma once

#include "api.h"
#include "pros/imu.h"
#include "pros/rtos.h"

namespace lamaLib {

/**
 * @brief The orientation of the inertial sensor along the x, y, and z axis, in degrees
 */
typedef struct {
    double x;
    double y;
    double z;
} Angles;

class Inertial : pros::IMU {
    public:
    /**
     * @brief The inertial sensor to keep track of the robot's orientation
     * 
     * @param port The port that the inertial sensor is connected to
     */
    Inertial(int port);

    /**
     * @brief Reset all the readings back to 0
     */
    void resetAll();

    /**
     * @brief Gets the delta of the readings
     * 
     * @return pros::c::imu_gyro_s_t 
     */
    pros::c::imu_gyro_s_t getDeltaAngles();

    /**
     * @brief Gets the roll, or the orientation about the x axis in degrees
     * 
     * @return double 
     */
    double getRoll();
    /**
     * @brief Sets the roll, or the orientation about the x axis in degrees
     * 
     * @param iangle The new angle in degrees
     */
    void setRoll(double iangle);

    /**
     * @brief Gets the pitch, or the orientation about the y axis in degrees
     * 
     * @return double 
     */
    double getPitch();
    /**
     * @brief Sets the pitch, or the orientation about the y axis in degrees
     * 
     * @param iangle The new angle in degrees
     */
    void setPitch(double iangle);

    /**
     * @brief Gets the yaw, or the orientation about the z axis in degrees, clamped between -180 and 180
     *
     * Better known as the heading
     * 
     * @return double 
     */
    double getYaw();
    /**
     * @brief Sets the yaw, or the orientation about the z axis in degrees, clamped between -180 and 180
     *
     * Better known as the heading
     * 
     * @param iangle The new angle in degrees
     */
    void setYaw(double iangle);
    /**
     * @brief Gets the yaw, or the orientation about the z axis in degrees, clamped between 0 and 360
     *
     * Better known as the heading
     * 
     * @return double 
     */
    double getHeading();
    /**
     * @brief Sets the yaw, or the orientation about the z axis in degrees, clamped between 0 and 360
     * 
     * @param iangle The new angle in degrees
     */
    void setHeading(double iangle);
    /**
     * @brief Gets the yaw, or the orientation about the z axis in degrees
     * 
     * @return double 
     */
    double getRotation();
    /**
     * @brief Sets the yaw, or the orientation about the z axis in degrees
     * 
     * @param iangle The new angle in degrees
     */
    void setRotation(double iangle);

    /**
     * @brief Calibrates the inertial sensor, finding the max drifts about each axis
     * 
     * @return Angles 
     */
    Angles calibrate();
    /**
     * @brief Returns whether the sensor is calibrating or not
     * 
     * @return true 
     * @return false 
     */
    bool isCalibrating();

    /**
     * @brief Starts the drift correction task
     */
    void startCorrection();
    /**
     * @brief Ends the drift correction task
     */
    void endCorrection();

    private:
    pros::task_t inertialTask {};

    bool calibrating;
};

extern Inertial inertial;

/**
 * @brief The function that corrects the readings from the inertial sensor in the case of gyro drift
 * 
 * @param iparam 
 */
void driftCompensation(void* iparam);
} // namespace lamaLib