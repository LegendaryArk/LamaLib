#include "vision.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"

namespace lamalib {
    visionSensor::visionSensor(pros::Vision vSensor) : vSensor (pros::Vision(1)) {
        //Configures colour signatures
        //See: https://pros.cs.purdue.edu/v5/tutorials/topical/vision.html#setting-signatures
    pros::vision_signature_s_t YELLOW_SIG =
        pros::Vision::signature_from_utility(1, 2495, 2875, 2684, -3881, -3593, -3738, 3.000, 0);
    pros::vision_signature_s_t BLUE_SIG =
        pros::Vision::signature_from_utility(2, -3111, -2329, -2720, 7769, 12829, 10300, 3.100, 0);
    pros::vision_signature_s_t RED_SIG =
        pros::Vision::signature_from_utility(3, 9997, 10649, 10324, -1157, -797, -978, 6.200, 0);
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