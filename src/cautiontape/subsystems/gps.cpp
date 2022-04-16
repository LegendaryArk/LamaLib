#include "gps.hpp"
#include "pros/gps.h"

namespace lamaLib{
    gpsSystem::gpsSystem(int gpsPort):gpsSensor(gpsPort){
    }

    void gpsSystem::gpsInitialize(int startingX, int startingY, int startingHeading, int offsetX, int offsetY){
    gpsSensor.initialize_full(startingX, startingY, startingHeading, offsetX, offsetY);
    }

    int gpsSystem::getRotation(){
        return gpsSensor.get_rotation();
    }

    pros::c::gps_status_s_t gpsSystem::getStatus(){
        return gpsSensor.get_status();
    }
}