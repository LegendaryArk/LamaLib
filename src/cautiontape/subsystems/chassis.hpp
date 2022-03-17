#include "okapi/api.hpp"
#include "motor.hpp"

using namespace std;
namespace lamaLib {
    class Chassis {
        public:
            void move(int left, int right);
            lamaLib::Motor topLeft;
            lamaLib::Motor bottomLeft;
            lamaLib::Motor topRight;
            lamaLib::Motor bottomRight;
            okapi::AbstractMotor::gearset gearBox;
            Chassis(int8_t motorPorts[4], bool reverseConfig[4], okapi::AbstractMotor::gearset igearset);
    };
}