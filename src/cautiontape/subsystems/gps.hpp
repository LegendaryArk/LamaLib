#include "api.h"
#include "pros/gps.hpp"

namespace lamalib {
class gpsSystem {
public:
  pros::Gps gpsSensor;
  /**
  * @brief Input the GPS sensor port number for the constructor.
  * 
  * @param gpsPort
  */
  gpsSystem(int gpsPort);
  /**
   * @brief Initialize all the GPS sensor parameters.
   * 
   * @param startingX
   * @param startingY
   * @param startingHeading
   * @param offsetX
   * @param offsetY 
   */
  void gpsInitialize(int startingX, int startingY, int startingHeading, int offsetX, int offsetY);
  /**
   * @brief Returns robot rotation.
   * 
   * @return int 
   */
  int getRotation();
  /**
   * @brief Returns GPS status (the coordinates of the GPS).
   * 
   * @return pros::c::gps_status_s_t 
   */
  pros::c::gps_status_s_t getStatus();
};
} // namespace lamalib