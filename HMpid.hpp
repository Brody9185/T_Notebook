#include "main.h"

namespace Belt {
    class pid{
        private:
        double kP;
        double kI;
        double kD;
        double kIStart = 0;
        double dt = 20;
        double OutputMax;
        double OutputMin;
        std::string name = "Pid";
        inline pid(double kP, double kI, double kD, double kIStart, double dt, double OutputMax,  double OutputMin, std::string name);
        inline pid(double kP, double kI, double kD, double kIStart, double dt, std::string name);
        inline pid(double kP, double kI, double kD, double kIStart, std::string name);
        inline pid(double kP, double kI, double kD, double kIStart);
        inline pid(double kP, double kI, double kD, std::string name);
        inline pid(double kP, double kI, double kD);
        inline pid(double kP, double kI, std::string name);
        inline pid(double kP, double kI);
        inline pid(double kP, std::string name);
        inline pid(double kP);
        inline pid(std::string name);
        inline pid();
        inline void setConstants(double kP, double kI, double kD, double kIStart, double dt, double OutputMax,  double OutputMin, std::string name);
        inline void setConstants(double kP, double kI, double kD, double kIStart, double dt, std::string name);
        inline void setConstants(double kP, double kI, double kD, double kIStart, std::string name);
        inline void setConstants(double kP, double kI, double kD, double kIStart);
        inline void setConstants(double kP, double kI, double kD, std::string name);
        inline void setConstants(double kP, double kI, double kD);
        inline void setConstants(double kP, double kI, std::string name);
        inline void setConstants(double kP, double kI);
        inline void setConstants(double kP, std::string name);
        inline void setConstants(double kP);
        inline void setConstants(std::string name);
        double Encoder;
        double Target = 0;
        double Current;
        double Error;
        double lastError = 0;
        double integral;
        double derivative;
        double Output;

        // Computes the PID output based on the current input value
inline double Compute(double Current) {
    while (true) {
        // Calculate the error between the target and current value
        Error = Target - Current;

        // Only accumulate integral if error is above the integral activation threshold
        if (std::abs(Error) > kIStart) {
            // Accumulate the integral term to address steady-state error
            integral = integral + Error * dt;
        } else {
            // Integral windup prevention: do not accumulate when error is small
        }

        // Calculate the rate of change of the error
        derivative = (Error - lastError) / dt;

        // Compute the final PID output using proportional, integral, and derivative terms
        Output = kP * Error + kI * integral + kD * derivative;

        // Save current error for next derivative calculation
        lastError = Error;

        // Return the calculated output
        return Output;
    }
}
    };

}