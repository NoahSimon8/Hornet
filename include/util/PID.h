#pragma once
#include <limits>
#include <cmath>
#include "util/Math.h"
#ifdef abs
#undef abs // allow std::abs despite Arduino macro
#endif

class PID
{
public:
    PID(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

    // Calculate control output given a setpoint and a measurement.
    // dt is computed internally from steady_clock; on the first call,
    // derivative and integral are not applied to avoid a large transient.
    double calculate(double setpoint, double measurement, double dt)
    {

        if (std::isnan(prev_meas_))
        {
            prev_meas_ = measurement;
            // First call: just proportional action.
            const double error = setpoint - measurement;
            return clampOutput(kp_ * error);
        }

        const double error = setpoint - measurement;

        // Integral (respect integral zone if enabled)
        if (!use_integral_zone_ || std::abs(error) <= integral_zone_)
        {
            integral_ += error * dt;
        }

        // Derivative on measurement to reduce derivative kick
        double deriv_raw = 0.0;
        if (!std::isnan(prev_meas_))
        {
            const double dmeas = measurement - prev_meas_;
            deriv_raw = -dmeas / std::max(dt, 1e-9); // negative sign: d(error)/dt = -d(meas)/dt
        }
        prev_meas_ = measurement;

        // Optional 1stâ€‘order lowâ€‘pass on derivative: y += a*(x - y)
        double deriv_term = deriv_raw;
        if (use_deriv_lp_)
        {
            const double rc = 1.0 / (2.0 * M_PI * std::max(deriv_cutoff_hz_, 1e-6));
            const double a = dt / (rc + dt);
            deriv_state_ += a * (deriv_raw - deriv_state_);
            deriv_term = deriv_state_;
        }

        double integral_output = integral_ * ki_;
        if (use_integral_limits_)
        {
            integral_output = util::clamp(integral_output, min_integral_, max_integral_);
        }
        // prevent division by 0
        if (ki_ > 0.000001)
        {
            integral_ = integral_output / ki_; // store back clamped integral
        }

        double output = kp_ * error + integral_output + kd_ * deriv_term;

        return clampOutput(output);
    }

    // Reset internal state (integral, derivative filter). Optionally set a new integral value.
    void reset(double integral = 0.0)
    {
        integral_ = integral;
        prev_meas_ = std::numeric_limits<double>::quiet_NaN();
        deriv_state_ = 0.0;
    }

    // --- Configuration helpers ---
    void setGains(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        reset();
    }

    void setP(double kp) { kp_ = kp; }
    void setI(double ki) { ki_ = ki; }
    void setD(double kd) { kd_ = kd; }

    double getP() const { return kp_; }
    double getI() const { return ki_; }
    double getD() const { return kd_; }

    void setOutputLimits(double min_out, double max_out)
    {
        min_out_ = min_out;
        max_out_ = max_out;
        use_output_limits_ = true;
        if (min_out_ > max_out_)
            std::swap(min_out_, max_out_);
    }
    void clearOutputLimits() { use_output_limits_ = false; }

    void setIntegralLimits(double min_i, double max_i)
    {
        min_integral_ = min_i;
        max_integral_ = max_i;
        use_integral_limits_ = true;
        if (min_integral_ > max_integral_)
            std::swap(min_integral_, max_integral_);
    }
    void clearIntegralLimits() { use_integral_limits_ = false; }

    void setIntegralZone(double zone_abs_error)
    {
        integral_zone_ = std::max(0.0, zone_abs_error);
        use_integral_zone_ = true;
    }
    void clearIntegralZone() { use_integral_zone_ = false; }

    // Enable a lowâ€‘pass filter on the derivative term. Example: cutoff_hz = 30.0
    void setDerivativeLowPass(double cutoff_hz)
    {
        deriv_cutoff_hz_ = cutoff_hz;
        use_deriv_lp_ = true;
    }
    void clearDerivativeLowPass()
    {
        use_deriv_lp_ = false;
        deriv_state_ = 0.0;
    }

private:
    double clampOutput(double u) const
    {
        if (!use_output_limits_)
            return u;
        if (u > max_out_)
            return max_out_;
        if (u < min_out_)
            return min_out_;
        return u;
    }

    // Gains
    double kp_{};
    double ki_{};
    double kd_{};

    // State
    double integral_ = 0.0;
    double prev_meas_ = std::numeric_limits<double>::quiet_NaN();

    // Derivative lowâ€‘pass state
    bool use_deriv_lp_ = false;
    double deriv_cutoff_hz_ = 0.0;
    double deriv_state_ = 0.0;

    // Limits
    bool use_output_limits_ = false;
    double min_out_ = -1.0, max_out_ = 1.0;
    bool use_integral_limits_ = false;
    double min_integral_ = -1e6, max_integral_ = 1e6;
    bool use_integral_zone_ = false;
    double integral_zone_ = std::numeric_limits<double>::infinity();
};
