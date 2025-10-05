#include <algorithm>
#include <limits>
// ...
class PID
{
public:
    double calculate(double setpoint, double measurement, double dt)
    {
        if (!prev_meas_valid_)
        {
            prev_meas_ = measurement;
            prev_meas_valid_ = true;
            const double error = setpoint - measurement;
            return clampOutput(kp_ * error);
        }
        const double error = setpoint - measurement;

        if (!use_integral_zone_ || std::abs(error) <= integral_zone_)
        {
            integral_ += error * dt;
            if (use_integral_limits_)
            {
                if (integral_ > max_integral_)
                    integral_ = max_integral_;
                if (integral_ < min_integral_)
                    integral_ = min_integral_;
            }
        }

        if (dt < 1e-9)
            dt = 1e-9; // replaces std::max(dt, 1e-9)
        const double dmeas = measurement - prev_meas_;
        double deriv_raw = -dmeas / dt;
        prev_meas_ = measurement;
        prev_meas_valid_ = true;

        if (use_deriv_lp_)
        {
            const double rc = 1.0 / (2.0 * M_PI * std::max(deriv_cutoff_hz_, 1e-6));
            const double a = dt / (rc + dt);
            deriv_state_ += a * (deriv_raw - deriv_state_);
            deriv_raw = deriv_state_;
        }

        double output = kp_ * error + ki_ * integral_ + kd_ * deriv_raw;
        return clampOutput(output);
    }

    void reset(double integral = 0.0)
    {
        integral_ = integral;
        prev_meas_valid_ = false;
        prev_meas_ = 0.0;
        deriv_state_ = 0.0;
    }

private:
    // ...
    bool prev_meas_valid_{false};
    double prev_meas_{0.0};
};
