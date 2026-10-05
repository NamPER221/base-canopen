/**
 * @file differential_drive.cpp
 * @brief Differential Drive Kinematics implementation
 */

#include <canopen/kinematics/differential_drive.hpp>
#include <algorithm>

namespace canopen {
namespace kinematics {

DifferentialDriveKinematics::DifferentialDriveKinematics(double wheel_radius_m,
                                                         double wheelbase_m,
                                                         double max_rpm)
    : wheel_radius_(wheel_radius_m), wheelbase_(wheelbase_m), max_rpm_(max_rpm) {}

void DifferentialDriveKinematics::set_wheel_parameters(double wheel_radius_m,
                                                       double wheelbase_m) {
    wheel_radius_ = wheel_radius_m;
    wheelbase_ = wheelbase_m;
}

// ==================== Forward Kinematics ====================

void DifferentialDriveKinematics::update(double dp_left, double dp_right, double dt) {
    // Distance traveled by each wheel along the arc
    const double ds_left = wheel_radius_ * dp_left;
    const double ds_right = wheel_radius_ * dp_right;

    // Robot center displacement and rotation
    const double d_s = (ds_right + ds_left) / 2.0;
    const double d_theta = (ds_right - ds_left) / wheelbase_;

    // Instantaneous velocities
    const double dt_safe = (dt > 0.0) ? dt : 1e-9;
    const double fi_left = dp_left / dt_safe;
    const double fi_right = dp_right / dt_safe;
    last_velocity_.linear = wheel_radius_ * (fi_right + fi_left) / 2.0;
    last_velocity_.angular = wheel_radius_ * (fi_right - fi_left) / wheelbase_;

    // Pose update (midpoint integration)
    pose_.theta += d_theta;
    pose_.x += d_s * std::cos(pose_.theta);
    pose_.y += d_s * std::sin(pose_.theta);
}

void DifferentialDriveKinematics::reset_pose(double x, double y, double theta) {
    pose_.x = x;
    pose_.y = y;
    pose_.theta = theta;
}

void DifferentialDriveKinematics::reset_encoders() {
    first_update_ = true;
    last_velocity_ = BodyVelocity{};
}

// ==================== Inverse Kinematics ====================

DifferentialDriveKinematics::WheelVelocity
DifferentialDriveKinematics::velocity_to_wheels(double v, double omega) const {
    WheelVelocity w;
    w.left = v - omega * wheelbase_ / 2.0;
    w.right = v + omega * wheelbase_ / 2.0;
    return w;
}

DifferentialDriveKinematics::WheelRPM
DifferentialDriveKinematics::velocity_to_rpm(double v, double omega) const {
    const WheelVelocity w = velocity_to_wheels(v, omega);

    // rad/s -> RPM: omega * 60 / (2 * PI)
    double rpm_left = w.left / wheel_radius_ * 60.0 / (2.0 * M_PI);
    double rpm_right = w.right / wheel_radius_ * 60.0 / (2.0 * M_PI);

    // Clamp to drive limits
    rpm_left = std::clamp(rpm_left, -max_rpm_, max_rpm_);
    rpm_right = std::clamp(rpm_right, -max_rpm_, max_rpm_);

    WheelRPM out;
    out.left = static_cast<int16_t>(std::lround(rpm_left));
    out.right = static_cast<int16_t>(std::lround(rpm_right));
    return out;
}

double DifferentialDriveKinematics::linear_to_rpm(double v, double wheel_radius_m) {
    return v / wheel_radius_m * 60.0 / (2.0 * M_PI);
}

double DifferentialDriveKinematics::rpm_to_linear(double rpm, double wheel_radius_m) {
    return rpm * 2.0 * M_PI * wheel_radius_m / 60.0;
}

} // namespace kinematics
} // namespace canopen
