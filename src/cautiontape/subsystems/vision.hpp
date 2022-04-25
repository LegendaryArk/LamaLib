#include "api.h"
#include "pros/vision.h"
#include "pros/vision.hpp"

namespace lamaLib {
    class visionSensor{
    private:
    pros::Vision vSensor;
    pros::vision_signature_s_t YELLOW_SIG;
    pros::vision_signature_s_t BLUE_SIG;
    pros::vision_signature_s_t RED_SIG;

    public:
    visionSensor(pros::Vision vSensor);
    int getMiddle(int signature);
    int getWidth(int signature);
    };
}