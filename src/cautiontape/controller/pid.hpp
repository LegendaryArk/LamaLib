namespace lamaLib {
    class PIDController {
         public:
            double calculatePID(double current, double target, double leeway);
            void resetPID();
            double integral;
            double derivative;
            double prevError;
            double kp;
            double ki;
            double kd;
            double minimum;
            double maximum;
            double difference;
            double count;
            PIDController(double p, double i, double d, double min, double max);   
    };
}