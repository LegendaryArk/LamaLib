#include "api.h"
#include "pros/vision.h"
#include "pros/vision.hpp"

namespace lamalib {
    class visionSensor{
    public:
    pros::Vision vSensor;
    pros::vision_signature_s_t SIG_1;
    pros::vision_signature_s_t SIG_2;
    pros::vision_signature_s_t SIG_3;
    pros::vision_signature_s_t SIG_4;
    pros::vision_signature_s_t SIG_5;
    pros::vision_signature_s_t SIG_6;
    pros::vision_signature_s_t SIG_7;

    visionSensor(int vPort, pros::vision_signature_s_t inputSigs[7]);
    /**
    * @brief Input the vision sensor port number and an array of seven utility colour signatures.
    *
    * @param vPort
    * @param inputSigs 
    **/
    void setSignatures(pros::vision_signature_s_t inputSigs[]);
    /**
     * @brief Sets signatures in vision sensor memory to the signatures in the brain memory.
     * 
     * @param signature
     */
    int getMiddle(int signature);
    /**
     * @brief Gets the middle coordinate of the largest object with the specified colour signature. 
     * Returns coordinate out of 312(?) total camera resolution, e.g. middle coordinate would be 156
     * 
     * @param signature 
     * @return int 
     */
    int getWidth(int signature);
    /**
     * @brief Returns the width of the largest object with the specified colour signature.
     * 
     * @param signature
     * @return int
     */
    };
}