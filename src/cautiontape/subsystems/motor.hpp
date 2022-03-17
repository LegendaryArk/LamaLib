#include "okapi/api.hpp"
using namespace std;

namespace lamaLib {
    class Motor: public okapi::Motor {
        public:
            Motor(int8_t port);
            Motor(int8_t port,
                bool reverse,
                okapi::AbstractMotor::gearset igearset,
                okapi::AbstractMotor::encoderUnits encoderUnits = AbstractMotor::encoderUnits::degrees);
    };
}