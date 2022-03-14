#include "okapi/api.hpp"
using namespace std;

namespace lamaLib {
    class Motor: public okapi::Motor {
        public:
            Motor(int8_t port);
            Motor(int8_t port,
                bool reverse,
                AbstractMotor::gearset igearset,
                AbstractMotor::encoderUnits encoderUnits = AbstractMotor::encoderUnits::degrees);
    };
}