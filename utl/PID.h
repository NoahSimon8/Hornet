#pragma once
#include <cmath>
#include <optional>

// Lightweight PID controller converted from the provided Java version.
// Usage:
//   PID pid(0.8, 0.05, 0.1);
//   pid.setOutputLimits(-1.0, 1.0);
//   double u = pid.calculate(setpoint, measurement);  // dt inferred via steady_clock
//
// Notes:
// - Integral windup protection via integral clamping and optional integral zone.
// - Derivative is taken on measurement (less noise on setpoint steps) and can be low‑pass filtered.
// - Time step is automatic using std::chrono::steady_clock on each call.
// - Call reset() before reuse if needed.
class PID {
public:
    PID(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

    // Calculate control output given a setpoint and a measurement.
    // derivative and integral are not applied to avoid a large transient.
    double calculate(double setpoint, double measurement, double dt) {

        if (!prev_meas_) {
            prev_meas_ = measurement;
            // First call: just proportional action.
            const double error = setpoint - measurement;
            return clampOutput(kp_ * error);
        }

        const double error = setpoint - measurement;

        // Integral (respect integral zone if enabled)
        if (!use_integral_zone_ || std::abs(error) <= integral_zone_) {
            integral_ += error * dt;
            if (use_integral_limits_) {
                if (integral_ > max_integral_) integral_ = max_integral_;
                if (integral_ < min_integral_) integral_ = min_integral_;
            }
        }

        // Derivative on measurement to reduce derivative kick
        double deriv_raw = 0.0;
        if (prev_meas_) {
            const double dmeas = measurement - *prev_meas_;
            deriv_raw = -dmeas / std::max(dt, 1e-9); // negative sign: d(error)/dt = -d(meas)/dt
        }
        prev_meas_ = measurement;

        // Optional 1st‑order low‑pass on derivative: y += a*(x - y)
        double deriv_term = deriv_raw;
        if (use_deriv_lp_) {
            const double rc = 1.0 / (2.0 * M_PI * std::max(deriv_cutoff_hz_, 1e-6));
            const double a = dt / (rc + dt);
            deriv_state_ += a * (deriv_raw - deriv_state_);
            deriv_term = deriv_state_;
        }

        double output = kp_ * error + ki_ * integral_ + kd_ * deriv_term;
        return clampOutput(output);
    }

    // Reset internal state (integral, derivative filter, timers). Optionally set a new integral value.
    void reset(double integral = 0.0) {
        integral_ = integral;
        prev_meas_.reset();
        deriv_state_ = 0.0;
    }

    // --- Configuration helpers ---
    void setGains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
    void getGains(double& kp, double& ki, double& kd) const { kp = kp_; ki = ki_; kd = kd_; }

    void setOutputLimits(double min_out, double max_out) {
        min_out_ = min_out; max_out_ = max_out; use_output_limits_ = true;
        if (min_out_ > max_out_) std::swap(min_out_, max_out_);
    }
    void clearOutputLimits() { use_output_limits_ = false; }

    void setIntegralLimits(double min_i, double max_i) {
        min_integral_ = min_i; max_integral_ = max_i; use_integral_limits_ = true;
        if (min_integral_ > max_integral_) std::swap(min_integral_, max_integral_);
    }
    void clearIntegralLimits() { use_integral_limits_ = false; }

    void setIntegralZone(double zone_abs_error) { integral_zone_ = std::max(0.0, zone_abs_error); use_integral_zone_ = true; }
    void clearIntegralZone() { use_integral_zone_ = false; }

    // Enable a low‑pass filter on the derivative term. Example: cutoff_hz = 30.0
    void setDerivativeLowPass(double cutoff_hz) { deriv_cutoff_hz_ = cutoff_hz; use_deriv_lp_ = true; }
    void clearDerivativeLowPass() { use_deriv_lp_ = false; deriv_state_ = 0.0; }

private:
    double clampOutput(double u) const {
        if (!use_output_limits_) return u;
        if (u > max_out_) return max_out_;
        if (u < min_out_) return min_out_;
        return u;
    }

    // Gains
    double kp_{}; double ki_{}; double kd_{};

    // State
    double integral_ = 0.0;
    std::optional<double> prev_meas_{};

    // Derivative low‑pass state
    bool use_deriv_lp_ = false;
    double deriv_cutoff_hz_ = 0.0;
    double deriv_state_ = 0.0;

    // Limits
    bool use_output_limits_ = false; double min_out_ = -1.0, max_out_ = 1.0;
    bool use_integral_limits_ = false; double min_integral_ = -1e6, max_integral_ = 1e6;
    bool use_integral_zone_ = false; double integral_zone_ = std::numeric_limits<double>::infinity();
};
