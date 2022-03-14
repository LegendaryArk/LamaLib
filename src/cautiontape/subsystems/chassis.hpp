#include "okapi/api.hpp"
#include "motor.hpp"

using namespace std;
using namespace okapi;
namespace lamaLib {
    class Chassis {
        public:
            void move(int left, int right);
            lamaLib::Motor topLeft;
            lamaLib::Motor bottomLeft;
            lamaLib::Motor topRight;
            lamaLib::Motor bottomRight;
            AbstractMotor::gearset gearBox;
            Chassis(int8_t motorPorts[4], bool reverseConfig[4], AbstractMotor::gearset igearset);
    };
}