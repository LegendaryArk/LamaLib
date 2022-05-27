#pragma once

#include "api.h"
#include "pros/vision.h"
#include "pros/vision.hpp"

namespace lamaLib {
class VisionSensor : pros::Vision {
    public:
    /**
    * @brief The vision sensor is a low quality camera that can see "blobs" of colour specified through colour signatures, calibrated with the V5 Vision Utlitiy
    *
    * @param vPort The port of the vision sensor
    **/
    VisionSensor(int vPort);

    /**
     * @brief Sets signatures in vision sensor memory to the signatures in the brain memory.
     * 
     * @param inputSigs An array of pros::vision_signature_s_t of size 7 to be assigned into the vision sensor
     */
	void setSignatures(pros::vision_signature_s_t inputSigs[]);
    
	/**
     * @brief Gets the middle coordinate of the largest object with the specified colour signature. 
     * Returns coordinate out of 312(?) total camera resolution, e.g. middle coordinate would be 156
     * 
     * @param signature The colour signature to be found
     * @return The middle coordinate of the largest object of the colour signature
     */
    int getMiddle(int signature);
    /**
     * @brief Returns the width of the largest object with the specified colour signature.
     * 
     * @param signature The colour signature to be found
     * @return THe width of the largest object of the colour signature
     */
    int getWidth(int signature);
    /**
     * @brief Get the number of objects in the view of the vision sensor.
     * 
     * @return The number of objects in the camera's FOV
     */
    int getCount();

	private:
    pros::vision_signature_s_t SIG_1;
    pros::vision_signature_s_t SIG_2;
    pros::vision_signature_s_t SIG_3;
    pros::vision_signature_s_t SIG_4;
    pros::vision_signature_s_t SIG_5;
    pros::vision_signature_s_t SIG_6;
    pros::vision_signature_s_t SIG_7;
};
}