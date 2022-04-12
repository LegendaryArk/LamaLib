#include <cmath>

using namespace std;

namespace lamaLib {
class PIDController {
    public:
    double calculatePID(double current, double target, double leeway);
    void asyncPID(double current, double target, double leeway);

    void updatePID(double kp, double ki, double kd, double kf);
    void resetPID();

    PIDController(double kp, double ki, double kd, double kf, double max, double iComp = 10);
    
    private:
    double integral;
    double integralComp;
    double prevError;
    double kp;
    double ki;
    double kd;
    double kf;
    double max;
    double count;
};
}