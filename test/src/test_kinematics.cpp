/**
 * @file test_kinematics.cpp
 * @brief Differential Drive Kinematics library tests (no hardware)
 *
 * Tests the canopen_kinematics library (Forward/Inverse Kinematics)
 * with the ZLAC8015D robot parameters (r = 0.0865 m, L = 0.400 m).
 */

#include <canopen/kinematics/differential_drive.hpp>
#include <iostream>
#include <cmath>

using namespace canopen::kinematics;

static int tests_passed = 0;
static int tests_total = 0;

#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        tests_total++;                                                          \
        if (cond) {                                                             \
            std::cout << "  [PASS] " << msg << "\n";                            \
            tests_passed++;                                                     \
        } else {                                                                \
            std::cout << "  [FAIL] " << msg << " (line " << __LINE__ << ")\n";  \
        }                                                                       \
    } while (0)

int main() {
    std::cout << "=== Differential Drive Kinematics Tests ===\n";

    // ZLAC8015D robot parameters
    const double R = 0.0865;
    const double L = 0.400;

    // ==================== FK: straight line ====================
    {
        DifferentialDriveKinematics kin(R, L);
        kin.update(0.1, 0.1, 0.1);  // same encoder delta
        auto pose = kin.get_pose();
        CHECK(std::abs(pose.theta) < 1e-9, "FK straight line: theta = 0");
        // d_s = r*(dpR + dpL)/2 = 0.0865 * 0.1 = 0.00865
        CHECK(std::abs(pose.x - 0.00865) < 1e-9, "FK straight line: x = r*dp/2*2");
        CHECK(std::abs(pose.y) < 1e-9, "FK straight line: y = 0");
        // v = r*(fiR + fiL)/2 = 0.0865 * 1.0 = 0.0865 m/s
        CHECK(std::abs(kin.get_body_velocity().linear - 0.0865) < 1e-6,
              "FK straight line: v = 0.0865 m/s");
    }

    // ==================== FK: rotation in place ====================
    {
        DifferentialDriveKinematics kin(R, L);
        kin.update(-0.1, 0.1, 0.1);  // opposite deltas
        auto pose = kin.get_pose();
        CHECK(std::abs(pose.x) < 1e-9 && std::abs(pose.y) < 1e-9,
              "FK rotation: position unchanged");
        const double expected_theta = R * 0.2 / L;  // r*(dpR-dpL)/L
        CHECK(std::abs(pose.theta - expected_theta) < 1e-9,
              "FK rotation: theta = r*(dpR-dpL)/L");
    }

    // ==================== IK: straight line ====================
    {
        DifferentialDriveKinematics kin(R, L);
        auto rpm = kin.velocity_to_rpm(0.5, 0.0);
        CHECK(rpm.left == rpm.right, "IK straight: same RPM both wheels");

        // 0.5 m/s / 0.0865 m * 60/(2pi) ~ 55.2 RPM
        CHECK(std::abs(rpm.left - 55.2) < 1.0, "IK straight: ~55 RPM");
    }

    // ==================== IK: point turn ====================
    {
        DifferentialDriveKinematics kin(R, L);
        auto rpm = kin.velocity_to_rpm(0.0, 1.57);
        CHECK(rpm.left == -rpm.right, "IK point turn: L = -R");
    }

    // ==================== IK: arc ====================
    {
        DifferentialDriveKinematics kin(R, L);
        auto rpm = kin.velocity_to_rpm(0.3, 0.5);
        CHECK(rpm.right > rpm.left, "IK arc: right > left");
    }

    // ==================== IK: clamping ====================
    {
        DifferentialDriveKinematics kin(R, L, 1000.0);
        auto rpm = kin.velocity_to_rpm(10.0, 0.0);  // way over max
        CHECK(rpm.left == 1000 && rpm.right == 1000, "IK clamp at max_rpm");
    }

    // ==================== Round-trip: FK(IK(v)) consistency ====================
    {
        DifferentialDriveKinematics kin(R, L);
        const double v = 0.4;
        const double omega = 0.3;

        auto rpm = kin.velocity_to_rpm(v, omega);
        // Feed the RPM back as encoder deltas for 0.1 s
        const double dpL = (rpm.left * 2.0 * M_PI / 60.0) * 0.1;
        const double dpR = (rpm.right * 2.0 * M_PI / 60.0) * 0.1;

        DifferentialDriveKinematics kin2(R, L);
        kin2.update(dpL, dpR, 0.1);
        auto body = kin2.get_body_velocity();
        CHECK(std::abs(body.linear - v) < 0.01, "Round-trip: linear ~ v");
        CHECK(std::abs(body.angular - omega) < 0.02, "Round-trip: angular ~ omega");
    }

    // ==================== Static conversions ====================
    {
        CHECK(std::abs(DifferentialDriveKinematics::linear_to_rpm(0.865, R) - 95.49) < 0.01,
              "linear_to_rpm(0.865, R) = 95.49");
        CHECK(std::abs(DifferentialDriveKinematics::rpm_to_linear(95.49, R) - 0.865) < 1e-3,
              "rpm_to_linear(95.49, R) = 0.865");
    }

    // ==================== Reset pose ====================
    {
        DifferentialDriveKinematics kin(R, L);
        kin.update(1.0, 1.0, 0.5);
        kin.reset_pose(1.5, 2.5, 0.7);
        auto pose = kin.get_pose();
        CHECK(pose.x == 1.5 && pose.y == 2.5 && std::abs(pose.theta - 0.7) < 1e-9,
              "reset_pose works");
    }

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_total
              << " passed ===\n";
    return tests_passed == tests_total ? 0 : 1;
}
