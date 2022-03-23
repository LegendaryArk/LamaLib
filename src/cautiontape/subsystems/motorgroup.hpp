#include "motor.hpp"

namespace lamaLib {
class MotorGroup : Motor {
    public:
    MotorGroup(std::vector<Motor> motors);

    private:
    std::vector<Motor> motors;
};
} // namespace lamaLib