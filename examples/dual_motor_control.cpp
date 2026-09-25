/**
 * @file dual_motor_control.cpp
 * @brief Dual Motor Control với Inverse Kinematics
 *
 * Chương trình điều khiển 2 động cơ ZLAC8015D đồng thời sử dụng:
 * - Inverse Kinematics: cmd_vel (v, omega) → wheel RPM
 * - Forward Kinematics: encoder feedback → odometry
 *
 * Hardware: ZLAC8015D Dual-Axis Servo Driver
 * - Wheel diameter: 173mm → radius = 0.0865m
 * - Wheelbase: 400mm → L = 0.400m
 * - CAN Node ID: 1
 *
 * Usage:
 *   ./dual_motor_control <can_interface> [node_id]
 *   Example: ./dual_motor_control can0 1
 */

#include <canopen/can/raw/socket_can.hpp>
#include <canopen/kinematics/differential_drive.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <atomic>

using namespace canopen;

// ============================================================================
// Configuration
// ============================================================================

struct Config {
    std::string can_interface = "can0";
    uint8_t motor_node_id = 1;
    uint32_t sdo_timeout_ms = 200;
    double control_rate_hz = 20.0;  // 20 Hz control loop
} config;

// ============================================================================
// Kinematics Class
// ============================================================================

// Differential drive kinematics — dùng thư viện canopen_kinematics
class DifferentialKinematics {
public:
    // Robot parameters (from ZLAC8015D specs)
    static constexpr double WHEEL_RADIUS = 0.0865;   // meters (173mm diameter)
    static constexpr double WHEELBASE = 0.400;        // meters (400mm)
    static constexpr int16_t MAX_RPM = 1000;
    static constexpr int16_t MIN_RPM = -1000;

    // Current pose (library type)
    using Pose = kinematics::DifferentialDriveKinematics::Pose;
    Pose pose;

    // Previous encoder values
    double prev_phi_L_{0.0}, prev_phi_R_{0.0};
    bool first_update_{true};

    // Library implementation
    kinematics::DifferentialDriveKinematics lib{WHEEL_RADIUS, WHEELBASE, MAX_RPM};

    // INVERSE KINEMATICS: cmd_vel → wheel RPM
    struct WheelRPM {
        int16_t left;
        int16_t right;
    };

    WheelRPM velocityToRPM(double v, double omega) const {
        auto rpm = lib.velocity_to_rpm(v, omega);
        return {rpm.left, rpm.right};
    }

    // FORWARD KINEMATICS: encoder → odometry
    void updateFromEncoders(double phi_L, double phi_R, double dt) {
        if (first_update_) {
            prev_phi_L_ = phi_L;
            prev_phi_R_ = phi_R;
            first_update_ = false;
            return;
        }

        if (dt <= 0) return;

        lib.update(phi_L - prev_phi_L_, phi_R - prev_phi_R_, dt);
        prev_phi_L_ = phi_L;
        prev_phi_R_ = phi_R;

        pose = lib.get_pose();

        // Normalize theta to [-PI, PI]
        while (pose.theta > M_PI) pose.theta -= 2.0 * M_PI;
        while (pose.theta < -M_PI) pose.theta += 2.0 * M_PI;
    }

    // Convert wheel RPM to linear/angular velocity
    std::pair<double, double> rpmToVelocity(int16_t left_rpm, int16_t right_rpm) const {
        const double omega_L = left_rpm * (2.0 * M_PI) / 60.0;
        const double omega_R = right_rpm * (2.0 * M_PI) / 60.0;
        const double v_L = omega_L * WHEEL_RADIUS;
        const double v_R = omega_R * WHEEL_RADIUS;
        return {(v_R + v_L) / 2.0, (v_R - v_L) / WHEELBASE};
    }

    void resetPose(double x = 0, double y = 0, double theta = 0) {
        lib.reset_pose(x, y, theta);
        pose = lib.get_pose();
        first_update_ = true;
    }
};


// ============================================================================
// CANopen Helpers
// ============================================================================

bool sendNMT(SocketCAN& can, uint8_t node_id, uint8_t command) {
    struct can_frame frame;
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = command;
    frame.data[1] = node_id;
    return can.write(frame) >= 0;
}

bool enableMotor(SocketCAN& can, uint8_t node_id) {
    struct can_frame frame;
    frame.can_id = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;  // Expedited download, 8 bytes
    frame.data[1] = 0x40;  // Index 0x6040
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = 0x0F;  // Enable: shutdown → switch on → enable
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return can.write(frame) >= 0;
}

bool disableMotor(SocketCAN& can, uint8_t node_id) {
    struct can_frame frame;
    frame.can_id = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0x40;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;  // Disable
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return can.write(frame) >= 0;
}

bool setVelocityCombined(SocketCAN& can, uint8_t node_id, int16_t left_rpm, int16_t right_rpm) {
    struct can_frame frame;
    frame.can_id = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;  // Expedited download
    frame.data[1] = 0xFF;  // Index 0x60FF
    frame.data[2] = 0x60;
    frame.data[3] = 0x03;  // Subindex 3: Combined 32-bit
    frame.data[4] = static_cast<uint8_t>(left_rpm & 0xFF);
    frame.data[5] = static_cast<uint8_t>((left_rpm >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>(right_rpm & 0xFF);
    frame.data[7] = static_cast<uint8_t>((right_rpm >> 8) & 0xFF);
    return can.write(frame) >= 0;
}

bool readStatusword(SocketCAN& can, uint8_t node_id, uint16_t& statusword) {
    struct can_frame req;
    req.can_id = 0x600 + node_id;
    req.can_dlc = 8;
    req.data[0] = 0x40;
    req.data[1] = 0x41;
    req.data[2] = 0x60;
    req.data[3] = 0x00;
    req.data[4] = req.data[5] = req.data[6] = req.data[7] = 0x00;

    if (can.write(req) < 0) return false;

    struct can_frame resp;
    struct timespec timeout = {0, config.sdo_timeout_ms * 1000000};

    for (int i = 0; i < 10; i++) {
        ssize_t n = can.read(resp, &timeout);
        if (n > 0 && resp.can_id == (0x580u + node_id)) {
            statusword = resp.data[4] | (resp.data[5] << 8);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

bool readVelocity(SocketCAN& can, uint8_t node_id, int16_t& left_rpm, int16_t& right_rpm) {
    // Read left motor velocity (0x606C:01)
    struct can_frame req;
    req.can_id = 0x600 + node_id;
    req.can_dlc = 8;
    req.data[0] = 0x40;
    req.data[1] = 0x6C;
    req.data[2] = 0x60;
    req.data[3] = 0x01;  // Left
    req.data[4] = req.data[5] = req.data[6] = req.data[7] = 0x00;

    if (can.write(req) < 0) return false;

    struct can_frame resp;
    struct timespec timeout = {0, config.sdo_timeout_ms * 1000000};

    bool left_ok = false, right_ok = false;

    for (int i = 0; i < 10 && (!left_ok || !right_ok); i++) {
        ssize_t n = can.read(resp, &timeout);
        if (n > 0 && resp.can_id == (0x580u + node_id)) {
            if (resp.data[3] == 0x01) {
                left_rpm = resp.data[4] | (resp.data[5] << 8);
                left_ok = true;
            } else if (resp.data[3] == 0x02) {
                right_rpm = resp.data[4] | (resp.data[5] << 8);
                right_ok = true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return left_ok && right_ok;
}

bool readPosition(SocketCAN& can, uint8_t node_id, int32_t& left_pos, int32_t& right_pos) {
    // Read left motor position (0x6064:01)
    struct can_frame req;
    req.can_id = 0x600 + node_id;
    req.can_dlc = 8;
    req.data[0] = 0x40;
    req.data[1] = 0x64;
    req.data[2] = 0x60;
    req.data[3] = 0x01;  // Left
    req.data[4] = req.data[5] = req.data[6] = req.data[7] = 0x00;

    if (can.write(req) < 0) return false;

    struct can_frame resp;
    struct timespec timeout = {0, config.sdo_timeout_ms * 1000000};

    bool left_ok = false, right_ok = false;

    for (int i = 0; i < 10 && (!left_ok || !right_ok); i++) {
        ssize_t n = can.read(resp, &timeout);
        if (n > 0 && resp.can_id == (0x580u + node_id)) {
            if (resp.data[3] == 0x01) {
                left_pos = resp.data[4] | (resp.data[5] << 8) |
                          (resp.data[6] << 16) | (resp.data[7] << 24);
                left_ok = true;
            } else if (resp.data[3] == 0x02) {
                right_pos = resp.data[4] | (resp.data[5] << 8) |
                           (resp.data[6] << 16) | (resp.data[7] << 24);
                right_ok = true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return left_ok && right_ok;
}

// ============================================================================
// Control Commands
// ============================================================================

struct Command {
    double v{0.0};      // Linear velocity (m/s)
    double omega{0.0};   // Angular velocity (rad/s)
};

std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
}

void printHelp() {
    std::cout << "\n╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║           DUAL MOTOR CONTROL - KEYBOARD COMMANDS              ║\n";
    std::cout << "╠═══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  w: Forward        s: Backward       a: Turn Left            ║\n";
    std::cout << "║  d: Turn Right     q: Stop           e: Emergency Stop       ║\n";
    std::cout << "║  1-9: Speed (1=slow, 9=fast)   0: Reset Odometry           ║\n";
    std::cout << "║  r: Read Status    p: Print Parameters                      ║\n";
    std::cout << "║  h: Help           x: Exit                                    ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n\n";
}

void printStatus(const DifferentialKinematics::Pose& pose,
                 int16_t cmd_L, int16_t cmd_R,
                 int16_t fb_L, int16_t fb_R) {
    std::cout << "\r"
              << "Pose: x=" << std::setw(6) << std::fixed << std::setprecision(3) << pose.x
              << " y=" << std::setw(6) << pose.y
              << " θ=" << std::setw(6) << std::fixed << std::setprecision(2) << (pose.theta * 180.0 / M_PI) << "°"
              << " | CMD: L=" << std::setw(5) << cmd_L << " R=" << std::setw(5) << cmd_R
              << " | FB: L=" << std::setw(5) << fb_L << " R=" << std::setw(5) << fb_R
              << std::flush;
}

void printParameters() {
    std::cout << "\n═══════════════════════════════════════════════════════\n";
    std::cout << "Robot Parameters:\n";
    std::cout << "  Wheel radius: " << DifferentialKinematics::WHEEL_RADIUS << " m\n";
    std::cout << "  Wheelbase: " << DifferentialKinematics::WHEELBASE << " m\n";
    std::cout << "  Max RPM: " << DifferentialKinematics::MAX_RPM << "\n";
    std::cout << "  Max linear velocity: "
              << (DifferentialKinematics::MAX_RPM * 2 * M_PI / 60.0 * DifferentialKinematics::WHEEL_RADIUS)
              << " m/s\n";
    std::cout << "  Max angular velocity: "
              << (2.0 * DifferentialKinematics::MAX_RPM * 2 * M_PI / 60.0 * DifferentialKinematics::WHEEL_RADIUS / DifferentialKinematics::WHEELBASE)
              << " rad/s\n";
    std::cout << "═══════════════════════════════════════════════════════\n";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    // Parse arguments
    if (argc >= 2) config.can_interface = argv[1];
    if (argc >= 3) config.motor_node_id = std::stoi(argv[2]);

    std::signal(SIGINT, signalHandler);

    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║        DUAL MOTOR CONTROL with INVERSE KINEMATICS           ║\n";
    std::cout << "╠═══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  CAN Interface: " << std::setw(46) << std::left << config.can_interface << "║\n";
    std::cout << "║  Motor Node ID: " << std::setw(46) << std::left << (int)config.motor_node_id << "║\n";
    std::cout << "║  Control Rate: " << std::setw(46) << std::left << config.control_rate_hz << " Hz║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n";

    // Create CAN interface
    SocketCAN can(config.can_interface);

    std::cout << "\nOpening CAN interface...\n";
    if (can.open() < 0) {
        std::cerr << "ERROR: Cannot open CAN interface\n";
        return 1;
    }

    // Send NMT operational
    std::cout << "Sending NMT operational...\n";
    sendNMT(can, config.motor_node_id, 0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Enable motors
    std::cout << "Enabling motors...\n";
    if (!enableMotor(can, config.motor_node_id)) {
        std::cerr << "ERROR: Cannot enable motors\n";
        can.close();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Check status
    uint16_t statusword = 0;
    if (readStatusword(can, config.motor_node_id, statusword)) {
        std::cout << "Motor status: 0x" << std::hex << statusword << std::dec;
        if (statusword & 0x0004) std::cout << " (Enabled)";
        if (statusword & 0x0008) std::cout << " (FAULT!)";
        std::cout << "\n";
    }

    // Initialize kinematics
    DifferentialKinematics kinematics;
    kinematics.resetPose();

    // Command state
    Command cmd;
    Command prev_cmd;
    int16_t cmd_L = 0, cmd_R = 0;
    int16_t fb_L = 0, fb_R = 0;
    double speed_scale = 0.5;  // Default 50% speed

    printHelp();
    printParameters();

    std::cout << "\nReady! Use keyboard commands to control motors.\n";
    std::cout << "Press 'h' for help, 'x' to exit.\n\n";

    auto last_control = std::chrono::steady_clock::now();
    auto last_print = std::chrono::steady_clock::now();

    // Main control loop
    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        // Check for keyboard input (non-blocking simulation)
        // In real usage, this would use ncurses or similar
        // For now, demonstrate pre-defined motions

        // Control loop at configured rate
        auto control_dt = std::chrono::duration<double>(1.0 / config.control_rate_hz).count();
        auto elapsed = std::chrono::duration<double>(now - last_control).count();

        if (elapsed >= control_dt) {
            last_control = now;

            // Calculate RPM from cmd_vel using Inverse Kinematics
            double v_cmd = cmd.v * speed_scale;
            double omega_cmd = cmd.omega * speed_scale;

            auto rpm = kinematics.velocityToRPM(v_cmd, omega_cmd);
            cmd_L = rpm.left;
            cmd_R = rpm.right;

            // Send velocity command to both motors
            if (cmd.v != prev_cmd.v || cmd.omega != prev_cmd.omega) {
                setVelocityCombined(can, config.motor_node_id, cmd_L, cmd_R);
                prev_cmd = cmd;
            }

            // Read feedback
            readVelocity(can, config.motor_node_id, fb_L, fb_R);
        }

        // Print status at 5 Hz
        auto print_dt = std::chrono::duration<double>(now - last_print).count();
        if (print_dt >= 0.2) {
            last_print = now;
            printStatus(kinematics.pose, cmd_L, cmd_R, fb_L, fb_R);
        }

        // Small sleep to prevent CPU spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Cleanup
    std::cout << "\n\nStopping motors...\n";
    setVelocityCombined(can, config.motor_node_id, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    disableMotor(can, config.motor_node_id);

    can.close();

    std::cout << "Final pose: x=" << std::fixed << std::setprecision(3) << kinematics.pose.x
              << " y=" << kinematics.pose.y
              << " θ=" << (kinematics.pose.theta * 180.0 / M_PI) << "°\n";
    std::cout << "Exiting.\n";

    return 0;
}
