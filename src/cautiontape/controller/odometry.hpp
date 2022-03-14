#include "api.h"
#include "pros/rtos.h"

namespace lamaLib {
using namespace pros;

/**
 * @brief The coordinate position of the robot in inches and degrees
 *
 * Includes the time of the position
 */
typedef struct {
    double x;
    double y;
    double theta;
    uint time;
} Pose;

/**
 * @brief The encoder values of all 3 tracking wheels in ticks
 */
typedef struct {
    double left;
    double right;
    double rear;
    double theta;
} EncoderValues;

/**
 * @brief The measurements of the tracking wheels in inches
 * 
 * Left and right are separate in the case that they are different
 */
typedef struct {
    double wheelDiameter;
    double leftRadius;
    double rightRadius;
    double rearRadius;
} OdomScales;

class Odometry {
    public:
    /**
     * @brief Odometry to keep track of the robot's position
     * 
     * @param leftEncoder The left encoder sensor
     * @param rightEncoder The right encoder sensor
     * @param rearEncoder The rear encoder sensor
     * @param scales The measurements of the tracking wheels in inches
     */
    Odometry(ADIEncoder leftEncoder, ADIEncoder rightEncoder, ADIEncoder rearEncoder, OdomScales scales, int tpr);

    /**
     * @brief Get the left encoder tick counts
     * 
     * @return int 
     */
    EncoderValues getEncoders();

    // Move get and set pose to private and then create a setStartPose for setting the start position
    /**
     * @brief Get the current coordinates
     * 
     * @return Pose 
     */
    Pose getPose();
    /**
     * @brief Set the pose to a new coordinate position
     * 
     * @param ipose The new coordinate position
     */
    void setPose(Pose ipose);

    /**
     * @brief Get the current tracking wheel measurements
     * 
     * @return OdomScales 
     */
    OdomScales getScales();
    /**
     * @brief Set the tracking wheel measurements with new measurements
     * 
     * @param iscales the new measurements of the tracking wheel
     */
    void setScales(OdomScales iscales);

    /**
     * @brief Calibrate the tracking wheels
     *
     * @return OdomScales
     */
    OdomScales calibrate();

    /**
     * @brief Starts the odometry task
     */
    void startOdom();
    /**
     * @brief Ends the odometry task
     */
    void endOdom();

    int tpr;

    private:
    ADIEncoder leftEncoder;
    ADIEncoder rightEncoder;
    ADIEncoder rearEncoder;

    Pose pose;
    OdomScales scales;

    task_t odomTask {};
};

extern Odometry odom;

/**
 * @brief Main function of odometry
 */
void odometryMain(void* param);
} // namespace lamaLib