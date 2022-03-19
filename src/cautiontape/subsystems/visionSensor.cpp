#include "visionSensor.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"
#include <iterator>

namespace lamalib {
    visionSensor::visionSensor(int vPort, pros::vision_signature_s_t inputSigs[7]) : vSensor(vPort), SIG_1(inputSigs[0]), SIG_2(inputSigs[1]),
                                SIG_3(inputSigs[2]), SIG_4(inputSigs[3]), SIG_5(inputSigs[4]), SIG_6(inputSigs[5]), SIG_7(inputSigs[6]) {
        //Configures colour signatures
        //See: https://pros.cs.purdue.edu/v5/tutorials/topical/vision.html#setting-signatures
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