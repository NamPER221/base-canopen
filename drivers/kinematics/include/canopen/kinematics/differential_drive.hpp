/**
 * @file differential_drive.hpp
 * @brief Differential Drive Kinematics (robot-level, CANopen-independent)
 *
 * Forward Kinematics:  encoder deltas -> robot velocities + odometry pose
 * Inverse Kinematics:  cmd_vel (v, omega) -> wheel RPM (for drives that
 *                      take RPM targets, e.g., ZLAC8015D)
 */

#ifndef CANOPEN_KINEMATICS_DIFFERENTIAL_DRIVE_HPP
#define CANOPEN_KINEMATICS_DIFFERENTIAL_DRIVE_HPP

#include <cstdint>
#include <cmath>

namespace canopen {
namespace kinematics {

/**
 * @brief Differential drive kinematics
 */
class DifferentialDriveKinematics {
public:
    /**
     * @brief Robot pose (odometry)
     */
    struct Pose {
        double x{0.0};
        double y{0.0};
        double theta{0.0};
    };

    /**
     * @brief Wheel velocities as RPM (for RPM-target drives)
     */
    struct WheelRPM {
        int16_t left{0};
        int16_t right{0};
    };

    /**
     * @brief Wheel velocities (linear m/s + angular rad/s)
     */
    struct WheelVelocity {
        double left{0.0};
        double right{0.0};
    };

    /**
     * @brief Robot body velocities
     */
    struct BodyVelocity {
        double linear{0.0};   // m/s
        double angular{0.0};  // rad/s
    };

    /**
     * @param wheel_radius_m Wheel radius in meters
     * @param wheelbase_m Distance between wheel centers in meters
     * @param max_rpm Absolute RPM limit applied in velocity_to_rpm()
     */
    DifferentialDriveKinematics(double wheel_radius_m, double wheelbase_m,
                                double max_rpm = 1000.0);

    // ==================== Parameters ====================

    void set_wheel_parameters(double wheel_radius_m, double wheelbase_m);
    double wheel_radius() const { return wheel_radius_; }
    double wheelbase() const { return wheelbase_; }
    void set_max_rpm(double max_rpm) { max_rpm_ = max_rpm; }
    double max_rpm() const { return max_rpm_; }

    // ==================== Forward Kinematics ====================

    /**
     * @brief Update odometry from encoder deltas
     * @param dp_left  Left wheel angle change since last update (radians)
     * @param dp_right Right wheel angle change since last update (radians)
     * @param dt       Time elapsed since last update (seconds)
     */
    void update(double dp_left, double dp_right, double dt);

    /**
     * @brief Current odometry pose
     */
    Pose get_pose() const { return pose_; }

    /**
     * @brief Current body velocity (from last update)
     */
    BodyVelocity get_body_velocity() const { return last_velocity_; }

    /**
     * @brief Reset pose to (x, y, theta)
     */
    void reset_pose(double x = 0.0, double y = 0.0, double theta = 0.0);

    /**
     * @brief Reset accumulated encoder history
     */
    void reset_encoders();

    // ==================== Inverse Kinematics ====================

    /**
     * @brief Convert body velocity to wheel linear velocities
     *   v_L = v - omega * L / 2
     *   v_R = v + omega * L / 2
     */
    WheelVelocity velocity_to_wheels(double v, double omega) const;

    /**
     * @brief Convert body velocity to wheel RPM (clamped to ±max_rpm)
     *   RPM = omega_wheel * 60 / (2 * PI)
     */
    WheelRPM velocity_to_rpm(double v, double omega) const;

    /**
     * @brief Static conversion: linear velocity (m/s) to RPM given radius
     */
    static double linear_to_rpm(double v, double wheel_radius_m);

    /**
     * @brief Static conversion: RPM to linear velocity (m/s) given radius
     */
    static double rpm_to_linear(double rpm, double wheel_radius_m);

private:
    double wheel_radius_;
    double wheelbase_;
    double max_rpm_;

    Pose pose_;
    BodyVelocity last_velocity_;
    bool first_update_{true};
};

} // namespace kinematics
} // namespace canopen

#endif // CANOPEN_KINEMATICS_DIFFERENTIAL_DRIVE_HPP
