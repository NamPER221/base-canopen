/**
 * @file robot_move.cpp
 * @brief Điều khiển robot 2 bánh theo động học vi sai (dùng thư viện)
 *
 * Điều khiển robot di chuyển với vận tốc (v, omega) chuyển đổi qua
 * Inverse Kinematics thành RPM, kèm odometry từ encoder feedback
 * (Forward Kinematics).
 *
 * Ví dụ: di chuyển thẳng v = 0.1 m/s, gia tốc 800 RPM/s:
 *   ./robot_move can0 1 --v 0.1 --omega 0.0 --accel 800 --time 5
 *
 * Quay tại chỗ 90°/s:
 *   ./robot_move can0 1 --v 0.0 --omega 1.57 --accel 800 --time 1
 *
 * Vòng cung:
 *   ./robot_move can0 1 --v 0.1 --omega 0.3 --accel 800 --time 5
 */

#include <canopen/drivers/zlac8015/zlac8015_driver.hpp>
#include <canopen/kinematics/differential_drive.hpp>
#include <canopen/can/raw/socket_can_bus.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <string>
#include <fstream>

#if HAVE_YAML
#include <yaml-cpp/yaml.h>
#endif

using namespace canopen;
using namespace canopen::drivers;
using namespace canopen::kinematics;

// ============================================================================
// Configuration
// ============================================================================

struct MoveConfig {
    std::string can_interface = "can0";
    uint8_t node_id = 1;

    double v = 0.1;          // m/s (linear velocity)
    double omega = 0.0;      // rad/s (angular velocity)
    uint32_t accel = 800;    // RPM/s (profile acceleration)
    uint32_t decel = 800;    // RPM/s (profile deceleration)
    double run_time_s = 5.0; // seconds to run

    double wheel_radius = 0.0865;  // meters
    double wheelbase = 0.400;      // meters
    double max_rpm = 1000.0;

    double control_rate_hz = 20.0;  // odometry loop rate

    void loadFromFile(const std::string& filename) {
#if HAVE_YAML
        try {
            YAML::Node cfg = YAML::LoadFile(filename);

            if (cfg["canopen"]) {
                const auto& c = cfg["canopen"];
                can_interface = c["interface"].as<std::string>(can_interface);
                node_id = c["node_id"].as<uint8_t>(node_id);
            }

            if (cfg["motor"]) {
                const auto& m = cfg["motor"];
                wheel_radius = m["wheel_radius_m"].as<double>(wheel_radius);
                wheelbase = m["wheelbase_m"].as<double>(wheelbase);
                max_rpm = m["max_rpm"].as<double>(max_rpm);
            }

            if (cfg["control"]) {
                const auto& c = cfg["control"];
                control_rate_hz = c["control_rate_hz"].as<double>(control_rate_hz);
            }

            std::cout << "[Config] Loaded from: " << filename << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[Config] Error loading " << filename << ": "
                      << e.what() << "\n";
        }
#else
        (void)filename;
#endif
    }

    /**
     * @brief Positional args OVERRIDE the YAML config:
     *   arg1 = CAN interface (bất kỳ tên nào)
     *   arg2 = node ID
     */
    void loadFromArgs(int argc, char* argv[]) {
        int positional = 0;
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--v" && i + 1 < argc) {
                v = std::stod(argv[++i]);
            } else if (arg == "--omega" && i + 1 < argc) {
                omega = std::stod(argv[++i]);
            } else if (arg == "--accel" && i + 1 < argc) {
                accel = static_cast<uint32_t>(std::stoul(argv[++i]));
            } else if (arg == "--decel" && i + 1 < argc) {
                decel = static_cast<uint32_t>(std::stoul(argv[++i]));
            } else if (arg == "--time" && i + 1 < argc) {
                run_time_s = std::stod(argv[++i]);
            } else if (arg == "--wheel-radius" && i + 1 < argc) {
                wheel_radius = std::stod(argv[++i]);
            } else if (arg == "--wheelbase" && i + 1 < argc) {
                wheelbase = std::stod(argv[++i]);
            } else if (arg == "--rate" && i + 1 < argc) {
                control_rate_hz = std::stod(argv[++i]);
            } else if (arg == "--config" || arg == "-c") {
                if (i + 1 < argc) config_file_ = argv[++i];
            } else if (!arg.empty() && arg[0] != '-') {
                if (positional == 0) {
                    can_interface = arg;
                } else if (positional == 1) {
                    node_id = static_cast<uint8_t>(std::stoi(arg));
                }
                positional++;
            }
        }
    }

    void load() {
        // 1. YAML (default file hoặc --config)
        std::string path = config_file_;
        if (path.empty()) {
            path = CANOPEN_DEFAULT_CONFIG;
        }
        if (!path.empty()) {
            loadFromFile(path);
        }
        // 2. Command-line args override YAML
        loadFromArgs(last_argc_, last_argv_);
    }

    int last_argc_ = 0;
    char** last_argv_ = nullptr;
    std::string config_file_;
} config;

std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    config.last_argc_ = argc;
    config.last_argv_ = argv;
    config.load();  // YAML trước, args override sau

    std::cout << "=====================================================\n";
    std::cout << "        ROBOT MOVE - Differential Drive             \n";
    std::cout << "=====================================================\n";
    std::cout << "  CAN:       " << config.can_interface
              << " (node " << static_cast<int>(config.node_id) << ")\n";
    std::cout << "  Command:   v=" << config.v << " m/s, omega="
              << config.omega << " rad/s\n";
    std::cout << "  Profile:   accel=" << config.accel
              << " RPM/s, decel=" << config.decel << " RPM/s\n";
    std::cout << "  Run time:  " << config.run_time_s << " s\n";
    std::cout << "  Wheel:     r=" << config.wheel_radius
              << " m, L=" << config.wheelbase << " m\n";
    std::cout << "=====================================================\n\n";

    // 1. Kinematics (từ thư viện canopen_kinematics)
    DifferentialDriveKinematics kin(config.wheel_radius, config.wheelbase,
                                    config.max_rpm);

    // 2. CAN bus + ZLAC driver (từ thư viện canopen_zlac)
    SocketCanBus bus(config.can_interface);
    if (bus.open() < 0 || !bus.is_up()) {
        const int eno = bus.socket().get_error_num();
        std::cerr << "ERROR: cannot open " << config.can_interface
                  << " (errno " << eno << ": "
                  << bus.socket().get_error_str() << ")\n";
        if (eno == 93) {
            std::cerr << "  => CAN kernel module chua load. Fix:\n"
                      << "     sudo modprobe can\n"
                      << "     sudo modprobe can_raw\n"
                      << "     sudo ip link set " << config.can_interface
                      << " up type can bitrate 500000\n";
        } else {
            std::cerr << "  Hint: sudo ip link set " << config.can_interface
                      << " up type can bitrate 500000\n";
        }
        return 1;
    }

    ZLAC8015Driver driver(config.node_id, &bus);
    driver.set_sdo_timeout(200);
    driver.set_wheel_parameters(config.wheel_radius, config.wheelbase);

    // 3. Bootup + enable motor (CiA 402 sequence)
    std::cout << "[1] Bootup + enable motor...\n";
    if (!driver.init()) {
        std::cerr << "ERROR: motor init failed (check wiring/power/node-id)\n";
        driver.disable();
        bus.close();
        return 1;
    }
    std::cout << "    Statusword: 0x" << std::hex << driver.read_status()
              << std::dec << " (enabled)\n\n";

    // 4. Set motion profile (0x6081/0x6083/0x6084 per axis)
    std::cout << "[2] Set profile: accel=" << config.accel << " RPM/s...\n";
    if (!driver.set_profile(static_cast<uint32_t>(config.max_rpm),
                            config.accel, config.decel)) {
        std::cerr << "WARNING: some profile writes failed\n";
    }

    // 5. Inverse kinematics: (v, omega) → wheel RPM
    std::cout << "[3] Inverse kinematics:\n";
    auto rpm = kin.velocity_to_rpm(config.v, config.omega);
    std::cout << "    v=" << config.v << " m/s, omega=" << config.omega
              << " rad/s\n";
    std::cout << "    -> RPM L=" << rpm.left << " R=" << rpm.right << "\n\n";

    // 6. Control loop: gửi lệnh + đọc encoder → odometry (FK)
    std::cout << "[4] Moving... (Ctrl+C to stop early)\n";
    std::cout << "    " << std::setw(8) << "t(s)"
              << "  " << std::setw(8) << "x(m)"
              << "  " << std::setw(8) << "y(m)"
              << "  " << std::setw(8) << "theta"
              << "  " << std::setw(9) << "v(m/s)"
              << "  enc(L/R)\n";

    driver.set_velocity_rpm(rpm.left, rpm.right);

    const auto start = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / config.control_rate_hz));

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start).count();
        if (t >= config.run_time_s) break;

        std::this_thread::sleep_for(period);

        // Read encoder positions (0x6064:01/02) → Forward kinematics
        const int32_t enc_L = driver.get_position_left();
        const int32_t enc_R = driver.get_position_right();

        // Encoder counts → radians (giả định 1024 counts/rev, điều chỉnh
        // theo encoder thực tế nếu cần)
        const double rad_per_count = 2.0 * M_PI / 1024.0;
        static int32_t prev_L = enc_L, prev_R = enc_R;
        static bool first = true;

        if (!first) {
            kin.update((enc_L - prev_L) * rad_per_count,
                       (enc_R - prev_R) * rad_per_count, 0.05);
        }
        prev_L = enc_L;
        prev_R = enc_R;
        first = false;

        auto pose = kin.get_pose();
        auto body = kin.get_body_velocity();

        std::cout << "    " << std::setw(8) << std::fixed << std::setprecision(2) << t
                  << "  " << std::setw(8) << std::setprecision(3) << pose.x
                  << "  " << std::setw(8) << pose.y
                  << "  " << std::setw(8) << std::setprecision(2) << pose.theta
                  << "  " << std::setw(9) << std::setprecision(3) << body.linear
                  << "  " << enc_L << "/" << enc_R << "\n";
    }

    // 7. Stop (deceleration profile) + disable
    std::cout << "\n[5] Stopping (decel=" << config.decel << " RPM/s)...\n";
    driver.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Final odometry
    auto pose = kin.get_pose();
    std::cout << "\nFinal odometry:\n";
    std::cout << "  x = " << std::fixed << std::setprecision(3) << pose.x << " m\n";
    std::cout << "  y = " << pose.y << " m\n";
    std::cout << "  theta = " << std::setprecision(2) << pose.theta << " rad ("
              << pose.theta * 180.0 / M_PI << " deg)\n";

    driver.disable();
    bus.close();

    std::cout << "\nDone.\n";
    return 0;
}
