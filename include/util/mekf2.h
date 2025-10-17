#pragma once
/**
 * @file mekf.hpp
 * @brief Header-only Multiplicative EKF for drones with delayed-measurement handling (OOSM).
 *
 * State (nominal):
 *   - p_w  (3): world position ENU, +Z up
 *   - v_w  (3): world velocity ENU
 *   - q_bw (4): body→world quaternion (unit), Hamilton convention
 *   - b_g  (3): gyro bias in body frame [rad/s]
 *   - b_a  (3): accel bias in body frame [m/s^2]
 *
 * Error-state (15): [δp, δv, δθ, δb_g, δb_a] where δθ is the small-angle attitude error.
 *
 * Prediction:
 *   Uses raw IMU gyro/accel at high rate. Accel is treated as specific force in body,
 *   rotated to world via q_bw, and +g is added to get world linear acceleration.
 *
 * Updates (any order, any reasonable delay):
 *   - GNSS position (ENU, m)
 *   - GNSS velocity / Doppler (ENU, m/s)
 *   - LiDAR altitude z (m)
 *   - Vision position (e.g., AprilTags) (ENU, m)
 *   - Magnetometer (body-frame field vector)
 *
 * Delayed / Out-of-Sequence Measurements (OOSM):
 *   The filter stores a small history of state+covariance snapshots after each IMU and the IMU
 *   samples themselves. When a measurement timestamp t_meas < current filter time, the filter:
 *     1) Restores the latest snapshot ≤ t_meas,
 *     2) Replays IMU forward to t_meas, applies the update,
 *     3) Replays IMU to the present (rewinds-and-replays).
 *   This aligns measurements to where the vehicle actually was when they were taken, not when received.
 *
 * Conventions & Assumptions:
 *   - Frames: ENU world with +Z up. Gravity g_world≈[0,0,-9.80665].
 *   - IMU: gyro in rad/s, accel as specific force (what most accelerometers output), both in body frame.
 *   - Quaternion: Hamilton (w,x,y,z). q_bw rotates body vectors into world: v_w = R(q_bw) * v_b.
 *   - Integration: nominal state uses simple constant-accel/constant-rate integration (p += v*dt + 0.5*a*dt^2,
 *     v += a*dt, q ← q ⊗ exp(0.5*ω*dt)). Covariance uses first-order discretization (Φ ≈ I + F dt, Qd ≈ GQcGᵀ dt).
 *
 * Build:
 *   - Requires Eigen (headers). Example: g++ -O3 -DEIGEN_NO_DEBUG main.cpp -I/path/to/eigen
 *
 * Notes:
 *   - Feed IMU in chronological order with addImu().
 *   - Call update* with the MEASUREMENT TIMESTAMP (when the sensor sampled). For convenience, wrappers
 *     are provided that take RECEIVE TIME and subtract a fixed configured latency per sensor.
 *   - Increase Config::history_window_sec if your delays exceed the default.
 */

#if __has_include(<ArduinoEigenDense.h>)
#include <ArduinoEigenDense.h>
#elif __has_include(<Eigen/Dense>)
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>
#else
#error "Eigen headers not found. Install Eigen or ArduinoEigen."
#endif
#include <cmath>
#include <limits>
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Quaternion;

#include <optional>
#include <deque>
#include <stdexcept>
#include <cstddef>
#include <cmath>

namespace mekf
{

    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;
    using Quat = Eigen::Quaterniond;
    using Vec15 = Eigen::Matrix<double, 15, 1>;
    using Mat15 = Eigen::Matrix<double, 15, 15>;

    // ============================== Small helpers ================================
    /** @brief Skew-symmetric matrix [v]_x such that [v]_x w = v×w */
    inline Mat3 Skew(const Vec3 &v)
    {
        Mat3 S;
        S << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return S;
    }

    /** @brief Small-angle quaternion dq ≈ [1, 0.5*dθ] */
    inline Quat SmallAngleQuat(const Vec3 &dtheta)
    {
        Quat dq(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
        return dq.normalized();
    }

    /** @brief Integrate body angular rate over dt for body→world quaternion.
     *  Uses multiplicative first-order increment: q_new = q ⊗ dq, dq≈exp(0.5*ω*dt).
     */
    inline Quat IntegrateQuatBodyToWorld(const Quat &q_bw, const Vec3 &omega_b, double dt)
    {
        Vec3 dtheta = omega_b * dt;
        Quat dq = SmallAngleQuat(dtheta);
        Quat q_new = (q_bw * dq).normalized(); // Hamilton convention
        return q_new;
    }

    // ============================== Configuration ================================
    /** @brief Runtime configuration and noise/latency parameters. */
    struct Config
    {
        // --- Physics & site ---
        Vec3 g_world = Vec3(0, 0, -9.80665); ///< Gravity in world ENU (+Z up)
        Vec3 mag_world = Vec3(1, 0, 0);      ///< World magnetic field (ENU). Prefer unit-norm.

        // --- IMU continuous-time noise densities (1/√Hz) ---
        double gyro_noise = 0.015;    ///< rad/s/√Hz (white gyro noise)
        double accel_noise = 0.25;    ///< m/s²/√Hz (white accel noise)
        double gyro_bias_rw = 0.0005; ///< rad/s²/√Hz (gyro bias random walk)
        double accel_bias_rw = 0.01;  ///< m/s³/√Hz (accel bias random walk)

        // --- Measurement standard deviations (SI units) ---
        Vec3 gnss_pos_std = Vec3(0.02, 0.02, 0.04);   ///< m (RTK)
        Vec3 gnss_vel_std = Vec3(0.05, 0.05, 0.05);   ///< m/s (Doppler)
        double lidar_z_std = 0.02;                    ///< m
        Vec3 vision_pos_std = Vec3(0.03, 0.03, 0.03); ///< m
        double mag_std = 0.05;                        ///< unitless (if mag_world unit-norm)

        // --- Known fixed sensor latencies (optional convenience) ---
        double gnss_latency_s = 0.0;   ///< seconds delay; used by *WithRxTime wrappers
        double vision_latency_s = 0.0; ///< seconds delay
        double lidar_latency_s = 0.0;  ///< seconds delay
        double mag_latency_s = 0.0;    ///< seconds delay

        // --- History / bridging policy ---
        double history_window_sec = .2; ///< seconds kept for OOSM rewind.
        double zoh_bridge_limit = 0.02; ///< forward bridge limit if timestamp slightly ahead of last IMU
    };

    // ============================== State containers =============================
    /** @brief Nominal (manifold) state. */
    struct NominalState
    {
        double t = 0.0;            ///< filter time [s]
        Vec3 p = Vec3::Zero();     ///< world position ENU [m]
        Vec3 v = Vec3::Zero();     ///< world velocity ENU [m/s]
        Quat q = Quat::Identity(); ///< body→world quaternion
        Vec3 bg = Vec3::Zero();    ///< gyro bias [rad/s] (body)
        Vec3 ba = Vec3::Zero();    ///< accel bias [m/s²] (body)
    };

    /** @brief Snapshot for users (state + covariance). */
    struct StateEstimate
    {
        double t;
        Vec3 p, v;
        Quat q;
        Vec3 bg, ba;
        Mat15 P;
    };

    // ============================== Sensor packets ===============================
    /** @brief IMU sample at time t (gyro & accel in body; optional co-timed magnetometer). */
    struct ImuSample
    {
        double t;                ///< timestamp [s] when IMU was sampled
        Vec3 gyro;               ///< rad/s (raw)
        Vec3 accel;              ///< m/s² specific force (raw)
        std::optional<Vec3> mag; ///< optional body-frame magnetic vector
        std::size_t id = 0;      ///< absolute history index, assigned by filter
    };

    /** @brief Internal snapshot: state+P after processing a specific IMU. */
    struct Snapshot
    {
        double t;
        NominalState x;
        Mat15 P;
        std::size_t next_imu_abs_idx;
    };

    // ============================== Filter class =================================
    class MEKF
    {
    public:
        /** @brief Construct filter with configuration/noise. */
        explicit MEKF(const Config &cfg) : cfg_(cfg)
        {
            x_.t = 0.0;
            x_.q = Quat::Identity();
            P_.setIdentity();
            P_ *= 1e-2; // modest default uncertainty
        }

        /** @brief Set initial state and covariance (also seeds the history). */
        void setInitial(const NominalState &x0, const Mat15 &P0)
        {
            x_ = x0;
            P_ = P0;
            have_time_ = (x_.t > 0.0);
            snaps_.clear();
            imu_hist_.clear();
            snaps_.push_back(Snapshot{x_.t, x_, P_, next_abs_index_});
        }

        /** @brief Get current state snapshot. */
        StateEstimate getState() const { return StateEstimate{x_.t, x_.p, x_.v, x_.q, x_.bg, x_.ba, P_}; }

        // -------------------------- IMU ingestion ---------------------------------
        /** @brief Add an IMU sample (must be in chronological order).
         *  - Propagates state/cov to the sample time using the provided sample (one step),
         *  - Pushes a history snapshot,
         *  - If a co-timed magnetometer is present, applies the mag update *at that time*.
         */
        void addImu(ImuSample s)
        {
            s.id = next_abs_index_++;
            imu_hist_.push_back(s);
            if (!have_time_)
            {
                x_.t = s.t;
                have_time_ = true;
            }
            double dt = s.t - x_.t;
            if (dt < 0)
                return; // ignore out-of-order IMU
            if (dt > 0)
                propagateOnce(s.gyro, s.accel, dt);
            x_.t = s.t;
            pushSnapshotAfterImu(s.id);
            pruneHistory();
            if (s.mag)
                updateMag(s.t - cfg_.mag_latency_s, *s.mag); // honor configured mag latency
        }

        // ------------------ Convenience wrappers with RECEIVE TIME -----------------
        // These subtract configured fixed latency to derive measurement time.
        void updateGnssPositionWithRxTime(double t_rx, const Vec3 &p_meas) { updateGnssPosition(t_rx - cfg_.gnss_latency_s, p_meas); }
        void updateGnssVelocityWithRxTime(double t_rx, const Vec3 &v_meas) { updateGnssVelocity(t_rx - cfg_.gnss_latency_s, v_meas); }
        void updateVisionPositionWithRxTime(double t_rx, const Vec3 &p_meas) { updateVisionPosition(t_rx - cfg_.vision_latency_s, p_meas); }
        void updateLidarZWithRxTime(double t_rx, double z_meas) { updateLidarZ(t_rx - cfg_.lidar_latency_s, z_meas); }

        // ---------------------- Measurement updates (OOSM) -------------------------
        /** @brief GNSS Position update at measurement time t (ENU meters). */
        void updateGnssPosition(double t, const Vec3 &p_meas)
        {
            applyDelayedVec3(t, [&](auto &H, auto &r, auto &R)
                             { H = H_pos(); r = p_meas - x_.p; R = R_from_std(cfg_.gnss_pos_std); });
        }
        /** @brief GNSS Velocity (Doppler) update at measurement time t (ENU m/s). */
        void updateGnssVelocity(double t, const Vec3 &v_meas)
        {
            applyDelayedVec3(t, [&](auto &H, auto &r, auto &R)
                             { H = H_vel(); r = v_meas - x_.v; R = R_from_std(cfg_.gnss_vel_std); });
        }
        /** @brief LiDAR altitude update at measurement time t (z in meters). */
        void updateLidarZ(double t, double z_meas)
        {
            applyDelayedScalar(t, [&](auto &H, auto &r, auto &R)
                               { H.setZero(); H(0,2) = 1.0; r(0,0) = z_meas - x_.p.z(); R(0,0) = cfg_.lidar_z_std * cfg_.lidar_z_std; });
        }
        /** @brief Vision position update at measurement time t (ENU meters). */
        void updateVisionPosition(double t, const Vec3 &p_meas, const std::optional<Vec3> &custom_std = std::nullopt)
        {
            Vec3 std = custom_std.value_or(cfg_.vision_pos_std);
            applyDelayedVec3(t, [&](auto &H, auto &r, auto &R)
                             { H = H_pos(); r = p_meas - x_.p; R = R_from_std(std); });
        }
        /** @brief Magnetometer vector update at measurement time t (body-frame). */
        void updateMag(double t, const Vec3 &mag_b_meas)
        {
            applyDelayedVec3(t, [&](auto &H, auto &r, auto &R)
                             {
            const Mat3 R_bw = x_.q.toRotationMatrix();
            Vec3 m_b_pred = R_bw.transpose() * cfg_.mag_world; // world→body
            r = mag_b_meas - m_b_pred;
            H.setZero();
            H.template block<3,3>(0,6) = -Skew(m_b_pred); // d(m_b)/d(dθ) = -[m_b]_x
            R = Mat3::Identity() * (cfg_.mag_std * cfg_.mag_std); });
        }

    private:
        // ============================== Members ===================================
        Config cfg_;
        NominalState x_{}; // nominal state
        Mat15 P_{};        // 15×15 error covariance

        // IMU/state history for delayed measurements
        std::deque<ImuSample> imu_hist_{}; ///< recent IMU samples
        std::deque<Snapshot> snaps_{};     ///< snapshots after IMU application
        bool have_time_ = false;
        std::size_t next_abs_index_ = 0;  ///< absolute ID assigned to IMU samples
        std::size_t imu_hist_offset_ = 0; ///< number of IMUs popped from front

        // ================== Delayed measurement scaffolding =======================
        template <typename BuildFn>
        void applyDelayedVec3(double t_meas, BuildFn build)
        {
            rewindRepropagate(t_meas, [&]()
                              {
            Eigen::Matrix<double,3,15> H; Eigen::Vector3d r; Eigen::Matrix3d R;
            build(H, r, R); doKalmanUpdate(H, r, R); });
        }
        template <typename BuildFn>
        void applyDelayedScalar(double t_meas, BuildFn build)
        {
            rewindRepropagate(t_meas, [&]()
                              {
            Eigen::Matrix<double,1,15> H; Eigen::Matrix<double,1,1> r; Eigen::Matrix<double,1,1> R;
            build(H, r, R); doKalmanUpdate(H, r, R); });
        }

        /** @brief Core rewind-replay engine for a measurement at t_meas.
         *  Restores snapshot ≤ t_meas, replays IMU to t_meas, runs update, then replays to present.
         *  Throws if t_meas is older than Config::history_window_sec.
         */
        template <typename Fn>
        void rewindRepropagate(double t_meas, Fn at_time_update)
        {
            if (!have_time_)
            {
                x_.t = t_meas;
                have_time_ = true;
            }

            // Slightly-ahead timestamps: allow short forward bridge using last IMU
            if (t_meas > x_.t + 1e-12)
            {
                double gap = t_meas - x_.t;
                if (gap > cfg_.zoh_bridge_limit)
                    return; // ignore too-far-ahead measurement
                if (imu_hist_.empty())
                {
                    x_.t = t_meas;
                }
                else
                {
                    const ImuSample &last = imu_hist_.back();
                    propagateOnce(last.gyro, last.accel, gap);
                    x_.t = t_meas;
                    pushSnapshotAfterImu(last.id);
                }
                at_time_update();
                return;
            }

            // On-time (no rewind)
            if (std::abs(t_meas - x_.t) <= 1e-12)
            {
                at_time_update();
                return;
            }

            // Delayed measurement: find snapshot ≤ t_meas
            if (snaps_.empty())
                return; // no history
            int s_idx = static_cast<int>(snaps_.size()) - 1;
            while (s_idx >= 0 && snaps_[s_idx].t > t_meas)
                --s_idx;
            if (s_idx < 0)
                return; // measurement older than history window

            // Save present
            const double t_present = x_.t;

            // Restore base snapshot
            Snapshot base = snaps_[s_idx];
            x_ = base.x;
            P_ = base.P;
            snaps_.erase(snaps_.begin() + s_idx + 1, snaps_.end());

            // Compute local IMU index to replay from
            std::size_t start_abs_idx = base.next_imu_abs_idx; // first IMU to apply after snapshot
            std::size_t i_local = (start_abs_idx >= imu_hist_offset_) ? (start_abs_idx - imu_hist_offset_) : 0;

            // Replay to t_meas at IMU boundaries
            double t_cur = x_.t;
            while (i_local < imu_hist_.size() && imu_hist_[i_local].t <= t_meas)
            {
                const ImuSample &s = imu_hist_[i_local];
                double dt = s.t - t_cur;
                if (dt > 0)
                    propagateOnce(s.gyro, s.accel, dt);
                t_cur = s.t;
                x_.t = t_cur;
                pushSnapshotAfterImu(s.id);
                ++i_local;
            }
            // Partial step if t_meas is between IMU times
            if (x_.t < t_meas)
            {
                const ImuSample &s_next = (i_local < imu_hist_.size()) ? imu_hist_[i_local] : imu_hist_.back();
                double dtp = t_meas - x_.t;
                if (dtp > 0)
                    propagateOnce(s_next.gyro, s_next.accel, dtp);
                x_.t = t_meas; // no snapshot for fractional step
            }

            // Apply measurement at t_meas
            at_time_update();

            // Replay forward to present
            while (i_local < imu_hist_.size() && imu_hist_[i_local].t <= t_present)
            {
                const ImuSample &s = imu_hist_[i_local];
                double dt = s.t - x_.t;
                if (dt > 0)
                    propagateOnce(s.gyro, s.accel, dt);
                x_.t = s.t;
                pushSnapshotAfterImu(s.id);
                ++i_local;
            }
            if (x_.t < t_present)
            {
                // Bridge small tail if present > last IMU time
                const ImuSample &last = imu_hist_.back();
                double dtp = t_present - x_.t;
                if (dtp > 0)
                    propagateOnce(last.gyro, last.accel, dtp);
                x_.t = t_present;
                pushSnapshotAfterImu(last.id);
            }

            pruneHistory();
        }

        /** @brief Push a snapshot after consuming IMU with absolute id 'imu_abs_id_just_applied'. */
        void pushSnapshotAfterImu(std::size_t imu_abs_id_just_applied)
        {
            snaps_.push_back(Snapshot{x_.t, x_, P_, imu_abs_id_just_applied + 1});
        }

        /** @brief Prune history to respect history_window_sec and keep buffers aligned. */
        void pruneHistory()
        {
            const double t_keep = x_.t - cfg_.history_window_sec;
            while (snaps_.size() > 1 && snaps_.front().t < t_keep)
                snaps_.pop_front();
            // Align IMU history to earliest snapshot time
            double t_anchor = snaps_.empty() ? x_.t : snaps_.front().t;
            while (!imu_hist_.empty() && imu_hist_.front().t < t_anchor - 1e-9)
            {
                imu_hist_.pop_front();
                ++imu_hist_offset_;
            }
        }

        // ======================== Propagation / Linearization =====================
        /** @brief One propagation step using one IMU sample over dt. */
        void propagateOnce(const Vec3 &gyro_b_raw, const Vec3 &accel_b_raw, double dt)
        {
            if (dt <= 0)
                return;
            // Bias-correct IMU
            const Vec3 w_b = gyro_b_raw - x_.bg;  // rad/s
            const Vec3 f_b = accel_b_raw - x_.ba; // m/s² (specific force)

            // Attitude
            x_.q = IntegrateQuatBodyToWorld(x_.q, w_b, dt);
            const Mat3 R_bw = x_.q.toRotationMatrix();

            // World linear acceleration (a_w = R*f_b + g)
            const Vec3 a_w = R_bw * f_b + cfg_.g_world;

            // Kinematics (constant-acceleration position update)
            x_.p += x_.v * dt + 0.5 * a_w * dt * dt;
            x_.v += a_w * dt;

            // Error-state dynamics δx_dot = F δx + G w
            Mat15 F = Mat15::Zero();
            // δp_dot = δv
            F.block<3, 3>(0, 3) = Mat3::Identity();
            // δv_dot = -R*[f]_x δθ - R δb_a
            F.block<3, 3>(3, 6) = -R_bw * Skew(f_b);
            F.block<3, 3>(3, 12) = -R_bw;
            // δθ_dot = -[w_b]_x δθ - δb_g
            F.block<3, 3>(6, 6) = -Skew(w_b);
            F.block<3, 3>(6, 9) = -Mat3::Identity();
            // biases are random walks → zeros elsewhere

            // Noise mapping G for [n_g, n_a, n_bg, n_ba]
            Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
            G.block<3, 3>(6, 0) = Mat3::Identity();  // gyro noise → attitude
            G.block<3, 3>(3, 3) = R_bw;              // accel noise → velocity
            G.block<3, 3>(9, 6) = Mat3::Identity();  // gyro bias RW
            G.block<3, 3>(12, 9) = Mat3::Identity(); // accel bias RW

            // Continuous noise covariance Qc
            Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
            Qc.block<3, 3>(0, 0) = Mat3::Identity() * (cfg_.gyro_noise * cfg_.gyro_noise);
            Qc.block<3, 3>(3, 3) = Mat3::Identity() * (cfg_.accel_noise * cfg_.accel_noise);
            Qc.block<3, 3>(6, 6) = Mat3::Identity() * (cfg_.gyro_bias_rw * cfg_.gyro_bias_rw);
            Qc.block<3, 3>(9, 9) = Mat3::Identity() * (cfg_.accel_bias_rw * cfg_.accel_bias_rw);

            // Discretization (first-order)
            const Mat15 Phi = Mat15::Identity() + F * dt;
            const Mat15 Qd = (G * Qc * G.transpose()) * dt;
            P_ = Phi * P_ * Phi.transpose() + Qd;

            // Keep quaternion normalized (guards numerical drift)
            x_.q.normalize();
        }

        // ======================== Measurement utilities ===========================
        static Eigen::Matrix<double, 3, 15> H_pos()
        {
            Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
            H.block<3, 3>(0, 0) = Mat3::Identity();
            return H;
        }
        static Eigen::Matrix<double, 3, 15> H_vel()
        {
            Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
            H.block<3, 3>(0, 3) = Mat3::Identity();
            return H;
        }

        static Eigen::Matrix3d R_from_std(const Vec3 &std)
        {
            Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
            R(0, 0) = std.x() * std.x();
            R(1, 1) = std.y() * std.y();
            R(2, 2) = std.z() * std.z();
            return R;
        }

        /** @brief Core EKF update (error-state) that also applies multiplicative attitude correction. */
        template <typename HType, typename RType, typename ResType>
        void doKalmanUpdate(const HType &H, const ResType &r, const RType &R)
        {
            // S = H P Hᵀ + R
            auto Ssym = (H * P_ * H.transpose()).template selfadjointView<Eigen::Upper>();
            Eigen::Matrix<typename RType::Scalar, RType::RowsAtCompileTime, RType::ColsAtCompileTime> S = Ssym.toDenseMatrix();
            S += R;
            // K = P Hᵀ S⁻¹ (for small dims direct inverse is acceptable; use LDLT if needed)
            Eigen::Matrix<double, 15, HType::RowsAtCompileTime> K = P_ * H.transpose() * S.inverse();
            // Error-state update
            Vec15 dx = K * r;
            applyError(dx);
            // Joseph-form covariance update for symmetry/PSD
            const Mat15 I = Mat15::Identity();
            const Mat15 KH = K * H;
            P_ = (I - KH) * P_ * (I - KH).transpose() + K * R * K.transpose();
        }

        /** @brief Inject error-state and renormalize quaternion. */
        void applyError(const Vec15 &dx)
        {
            x_.p += dx.segment<3>(0);
            x_.v += dx.segment<3>(3);
            const Vec3 dtheta = dx.segment<3>(6);
            x_.q = (SmallAngleQuat(dtheta) * x_.q).normalized();
            x_.bg += dx.segment<3>(9);
            x_.ba += dx.segment<3>(12);
        }
    };

} // namespace mekf
