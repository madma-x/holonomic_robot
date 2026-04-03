#ifndef ARUCO_ALIGNMENT_PID_CONTROLLER_HPP
#define ARUCO_ALIGNMENT_PID_CONTROLLER_HPP

#include <cmath>
#include <algorithm>

class PIDController {
public:
    struct Gains {
        double kp, ki, kd;
    };

    struct Limits {
        double max_integral;
        double max_output;
    };

    PIDController(const Gains& gains, const Limits& limits)
        : gains_(gains), limits_(limits),
          integral_error_(0.0), last_error_(0.0), initialized_(false) {}

    // Update PID with current error and dt (seconds)
    // Returns velocity output command
    double update(double error, double dt) {
        if (dt <= 0.0) return 0.0;

        // Derivative term: rate of change of error
        double derivative = 0.0;
        if (initialized_) {
            derivative = (error - last_error_) / dt;
        }
        initialized_ = true;
        last_error_ = error;

        // Integral term with anti-windup
        integral_error_ += error * dt;
        integral_error_ = std::clamp(integral_error_, 
                                     -limits_.max_integral, 
                                     limits_.max_integral);

        // PID output: velocity command
        double output = gains_.kp * error 
                      + gains_.ki * integral_error_ 
                      + gains_.kd * derivative;

        // Clamp output to max velocity
        output = std::clamp(output, -limits_.max_output, limits_.max_output);

        return output;
    }

    void reset() {
        integral_error_ = 0.0;
        last_error_ = 0.0;
        initialized_ = false;
    }

private:
    Gains gains_;
    Limits limits_;
    double integral_error_;
    double last_error_;
    bool initialized_;
};

#endif // ARUCO_ALIGNMENT_PID_CONTROLLER_HPP
