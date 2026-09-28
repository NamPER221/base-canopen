/**
 * @file consumer_kin.cpp
 * @brief Kiểm tra target canopen_kinematics từ bản cài đặt
 */

#include <canopen/kinematics/differential_drive.hpp>

#include <cmath>
#include <cstdio>

int main() {
    // Bánh xe Ø173mm, khoảng cách 400mm, giới hạn 205 RPM (ZLLG65ASM250)
    canopen::kinematics::DifferentialDriveKinematics kin(0.0865, 0.400, 205.0);

    int failures = 0;
    auto check = [&failures](bool ok, const char* what) {
        std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
        if (!ok) ++failures;
    };

    // Tiến thẳng 0.5 m/s: 0.5/(2π*0.0865)*60 = 55.2 RPM
    const auto fwd = kin.velocity_to_rpm(0.5, 0.0);
    check(std::abs(fwd.left - fwd.right) < 0.01, "tiến: 2 bánh cùng chiều");
    check(fwd.left > 54.0 && fwd.left < 56.0, "tiến 0.5 m/s ≈ 55 RPM");

    // Quay tại chỗ: v_L = v - omega*L/2, v_R = v + omega*L/2
    // → omega > 0 (quay trái) thì bánh trái lùi, bánh phải tiến
    const auto rot = kin.velocity_to_rpm(0.0, 1.0);
    check(rot.left == -rot.right, "quay tại chỗ: 2 bánh ngược dấu cùng độ lớn");
    check(rot.right > 0 && rot.left < 0,
          "omega>0 (quay trái): bánh phải tiến, bánh trái lùi");

    // Clamp theo giới hạn động cơ
    const auto over = kin.velocity_to_rpm(10.0, 10.0);
    check(std::abs(over.left) <= 205.0 && std::abs(over.right) <= 205.0,
          "lệnh vượt giới hạn bị clamp về ±205 RPM");

    // Quy đổi ngược lại phải khớp
    const double v_back = canopen::kinematics::DifferentialDriveKinematics::rpm_to_linear(
        static_cast<double>(fwd.left), 0.0865);
    check(std::abs(v_back - 0.5) < 0.01, "rpm_to_linear khớp ngược");

    // Hàm tĩnh
    // WheelRPM là int16_t nên bị cắt bỏ phần thập phân — sai số tối đa 0.5
    check(std::abs(canopen::kinematics::DifferentialDriveKinematics::linear_to_rpm(0.5, 0.0865)
                   - static_cast<double>(fwd.left)) < 0.5,
          "linear_to_rpm khớp với velocity_to_rpm (sai số do int16_t)");

    // Odometry: đẩy 1 vòng bánh xe = 2π*0.0865 m
    kin.reset_pose();
    kin.reset_encoders();
    kin.update(2.0 * 3.14159265358979, 2.0 * 3.14159265358979, 1.0);
    const auto pose = kin.get_pose();
    check(std::abs(pose.x - 0.5434) < 0.01, "odometry: 1 vòng bánh ≈ 0.543 m");
    check(std::abs(pose.y) < 0.001, "odometry: không trượt ngang khi 2 bánh đều nhau");

    if (failures == 0) {
        std::printf("consumer_kin: OK — kinematics dùng được từ bên ngoài\n");
        return 0;
    }
    std::printf("consumer_kin: %d kiểm tra thất bại\n", failures);
    return 1;
}
