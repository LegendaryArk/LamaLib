#include "visionSensor.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"
#include <iterator>

namespace lamalib {
    visionSensor::visionSensor(int vPort) : vSensor(vPort) {
        //Configures colour signatures
        //See: https://pros.cs.purdue.edu/v5/tutorials/topical/vision.html#setting-signatures
    }

    void visionSensor::setSignatures(pros::vision_signature_s_t inputSigs[]){
        for(int i = 0; i<7; i++){
            vSensor.set_signature(i+1, &inputSigs[i]);
        }
}

    int visionSensor::getMiddle(int signature){
        //Gets the middle coordinate of the largest object with the specified colour signature
        //Returns coordinate out of 312(?) total camera resolution, e.g. middle coordinate would be 156
        pros::vision_object_s_t inp = vSensor.get_by_sig(0, signature);
        return inp.x_middle_coord;
    }

    int visionSensor::getWidth(int signature){
        //Returns the width of the largest object with the specified colour signature
        pros::vision_object_s_t inp = vSensor.get_by_sig(0, signature);
        return inp.width;
    }
}