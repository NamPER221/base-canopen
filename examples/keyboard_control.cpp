/**
 * @file keyboard_control.cpp
 * @brief Điều khiển robot bằng bàn phím kiểu "giữ phím để di chuyển"
 *
 * GIỮ phím → robot di chuyển; NHẢ phím → giảm tốc về 0 (ramp mượt).
 *
 * Cách hoạt động: terminal tự lặp phím khi giữ (~30 lần/s). Chương trình
 * đọc phím non-blocking; nếu phím được lặp trong khoảng hold_timeout_ms
 * → coi như đang giữ → gửi vận tốc; hết thời gian không có phím → coi như
 * nhả → ramp về 0.
 *
 * Keys:
 *   w: tiến          s: lùi
 *   a: quay trái     d: quay phải
 *   space: dừng khẩn cấp (0 ngay lập tức)
 *   1-9: tốc độ 10%-90%      0: 100%
 *   q / Ctrl+C: thoát
 *
 * Usage:
 *   ./keyboard_control can0 1
 *   ./keyboard_control can0 1 --max-v 0.3 --accel 1.5
 */

#include <canopen/drivers/zlac8015/zlac8015_driver.hpp>
#include <canopen/kinematics/differential_drive.hpp>
#include <canopen/can/raw/socket_can_bus.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>
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

struct TeleopConfig {
    std::string can_interface = "can0";
    uint8_t node_id = 1;

    double max_v = 0.2;           // m/s tại scale 100%
    double max_omega = 1.0;       // rad/s tại scale 100%
    double speed_scale = 0.5;     // 0.0 - 1.0 (điều chỉnh bằng 1-9)

    double accel_limit = 0.8;     // m/s² (ramp tiến)
    double decel_limit = 1.6;     // m/s² (ramp khi nhả phím — dừng nhanh hơn)
    double ang_accel_limit = 2.0; // rad/s²

    int hold_timeout_ms = 200;    // không nhận phím trong khoảng này = nhả

    double wheel_radius = 0.0865;
    double wheelbase = 0.400;
    double max_rpm = 1000.0;

    double control_rate_hz = 50.0;

    uint32_t profile_accel = 800; // RPM/s (drive profile)

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
                max_v = c["max_linear_velocity"].as<double>(max_v);
                max_omega = c["max_angular_velocity"].as<double>(max_omega);
                speed_scale = c["default_speed_scale"].as<double>(speed_scale);
                accel_limit = c["acceleration_limit"].as<double>(accel_limit);
                decel_limit = c["deceleration_limit"].as<double>(decel_limit);
            }

            if (cfg["safety"]) {
                const auto& s = cfg["safety"];
                // command_timeout drives the hold timeout fallback
                const double cmd_timeout = s["command_timeout"].as<double>(1.0);
                hold_timeout_ms = static_cast<int>(cmd_timeout * 1000.0);
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
     *   arg1 = CAN interface (bất kỳ tên nào, ví dụ can0/vcan0/slc0)
     *   arg2 = node ID
     */
    void loadFromArgs(int argc, char* argv[]) {
        int positional = 0;
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--max-v" && i + 1 < argc) {
                max_v = std::stod(argv[++i]);
            } else if (arg == "--max-omega" && i + 1 < argc) {
                max_omega = std::stod(argv[++i]);
            } else if (arg == "--accel" && i + 1 < argc) {
                accel_limit = std::stod(argv[++i]);
            } else if (arg == "--decel" && i + 1 < argc) {
                decel_limit = std::stod(argv[++i]);
            } else if (arg == "--hold" && i + 1 < argc) {
                hold_timeout_ms = std::stoi(argv[++i]);
            } else if (arg == "--wheel-radius" && i + 1 < argc) {
                wheel_radius = std::stod(argv[++i]);
            } else if (arg == "--wheelbase" && i + 1 < argc) {
                wheelbase = std::stod(argv[++i]);
            } else if (arg == "--config" || arg == "-c") {
                if (i + 1 < argc) config_file_ = argv[++i];
            } else if (!arg.empty() && arg[0] != '-') {
                // positional: interface name or node id
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
// Terminal helpers (raw mode, non-blocking)
// ============================================================================

class Terminal {
public:
    Terminal() {
        tcgetattr(STDIN_FILENO, &old_);
        new_ = old_;
        new_.c_lflag &= ~(ICANON | ECHO);  // raw: không line buffer, không echo
        tcsetattr(STDIN_FILENO, TCSANOW, &new_);

        // Non-blocking stdin
        const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    ~Terminal() {
        restore();
    }

    /**
     * @brief Đọc 1 phím nếu có (non-blocking)
     * @return phím, hoặc -1 nếu không có phím
     */
    int read_key() {
        char ch = 0;
        const ssize_t n = ::read(STDIN_FILENO, &ch, 1);
        return (n == 1) ? static_cast<int>(static_cast<unsigned char>(ch)) : -1;
    }

    void restore() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    }

private:
    struct termios old_{};
    struct termios new_{};
};

// ============================================================================
// Velocity ramping (giữ phím → tăng dần; nhả phím → giảm dần về 0)
// ============================================================================

struct RampState {
    double v{0.0};      // m/s command hiện tại
    double omega{0.0};  // rad/s command hiện tại

    void ramp_toward(double target_v, double target_omega, double dt,
                     double accel, double decel, double ang_accel) {
        // Linear velocity ramp
        const double dv = target_v - v;
        const double limit = (std::abs(target_v) > std::abs(v)) ? accel : decel;
        const double max_dv = limit * dt;
        v += std::clamp(dv, -max_dv, max_dv);

        // Angular velocity ramp
        const double dw = target_omega - omega;
        const double max_dw = ang_accel * dt;
        omega += std::clamp(dw, -max_dw, max_dw);
    }
};

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    config.last_argc_ = argc;
    config.last_argv_ = argv;
    config.load();  // YAML trước, args override sau

    std::cout << "=====================================================\n";
    std::cout << "     KEYBOARD CONTROL - giu phim de di chuyen        \n";
    std::cout << "=====================================================\n";
    std::cout << "  w: tien    s: lui    a: quay trai    d: quay phai  \n";
    std::cout << "  space: dung khan cap                       \n";
    std::cout << "  1-9: toc do 10%-90%    0: 100%             \n";
    std::cout << "  q / Ctrl+C: thoat                      \n";
    std::cout << "=====================================================\n";
    std::cout << "  max_v=" << config.max_v << " m/s  max_w=" << config.max_omega
              << " rad/s  scale=" << config.speed_scale << "\n";
    std::cout << "  accel=" << config.accel_limit << " decel=" << config.decel_limit
              << " m/s^2  hold_timeout=" << config.hold_timeout_ms << " ms\n";
    std::cout << "=====================================================\n\n";

    // 1. Kinematics + bus + driver (tu thu vien)
    DifferentialDriveKinematics kin(config.wheel_radius, config.wheelbase,
                                    config.max_rpm);

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
    driver.set_sdo_timeout(100);
    driver.set_wheel_parameters(config.wheel_radius, config.wheelbase);
    driver.logger = [](const std::string& msg) {
        std::cout << "    " << msg << "\n";
    };

    std::cout << "[1] Bootup + enable motor...\n";
    if (!driver.init()) {
        std::cerr << "ERROR: motor init failed (check wiring/power/node-id)\n";
        driver.disable();
        bus.close();
        return 1;
    }
    std::cout << "    Statusword: 0x" << std::hex << driver.read_status()
              << std::dec << "\n";

    // Set Profile Velocity mode (0x6060 = 3) — bắt buộc trước khi đặt tốc độ
    // (giống bản lely-core hoạt động: 0x601#2F600003000000)
    if (!driver.set_operation_mode(3)) {
        std::cerr << "WARNING: set operation mode (0x6060=3) failed\n";
    }

    // 2. Drive profile acceleration (0x6083/0x6084 per axis)
    driver.set_profile(static_cast<uint32_t>(config.max_rpm),
                       config.profile_accel, config.profile_accel);

    // 3. Keyboard teleop loop
    std::cout << "[2] Ready! giu phim w/s/a/d de di chuyen...\n\n";

    Terminal term;
    RampState ramp;
    int held_key = -1;
    auto last_key_time = std::chrono::steady_clock::now() -
                         std::chrono::milliseconds(config.hold_timeout_ms + 1);
    bool key_pressed_now = false;

    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / config.control_rate_hz));

    while (g_running) {
        const auto loop_start = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(period).count();

        // ---- Đọc phím (non-blocking, xử lý auto-repeat) ----
        int ch;
        while ((ch = term.read_key()) != -1) {
            switch (ch) {
                case 'w': case 'W':
                case 's': case 'S':
                case 'a': case 'A':
                case 'd': case 'D':
                    held_key = ch;
                    last_key_time = std::chrono::steady_clock::now();
                    key_pressed_now = true;
                    break;

                case ' ':  // dừng khẩn cấp
                    held_key = -1;
                    ramp.v = 0.0;
                    ramp.omega = 0.0;
                    break;

                case '1': case '2': case '3': case '4': case '5':
                case '6': case '7': case '8': case '9':
                    config.speed_scale = (ch - '0') / 10.0;
                    std::cout << "\r[Speed] " << (ch - '0') << "0%  \n";
                    break;
                case '0':
                    config.speed_scale = 1.0;
                    std::cout << "\r[Speed] 100%  \n";
                    break;

                case 'q': case 'Q': case 'x': case 'X':
                    g_running = false;
                    break;
            }
        }

        // ---- Kiểm tra "đang giữ" hay "đã nhả" ----
        const auto since_key = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_key_time).count();

        double target_v = 0.0;
        double target_omega = 0.0;
        bool held = (held_key != -1) && (since_key < config.hold_timeout_ms);

        if (held) {
            const double scale = config.speed_scale;
            switch (held_key) {
                case 'w': case 'W': target_v = config.max_v * scale; break;
                case 's': case 'S': target_v = -config.max_v * scale; break;
                case 'a': case 'A': target_omega = config.max_omega * scale; break;
                case 'd': case 'D': target_omega = -config.max_omega * scale; break;
            }
        } else {
            held_key = -1;  // nhả phím → target = 0 (ramp về 0)
        }

        // ---- Ramp vận tốc mượt ----
        ramp.ramp_toward(target_v, target_omega, dt,
                         config.accel_limit, config.decel_limit,
                         config.ang_accel_limit);

        // ---- Inverse kinematics → RPM → gửi ----
        const auto rpm = kin.velocity_to_rpm(ramp.v, ramp.omega);
        driver.set_velocity_rpm(rpm.left, rpm.right);

        // ---- Hiển thị ----
        if (key_pressed_now || held || std::abs(ramp.v) > 0.005 ||
            std::abs(ramp.omega) > 0.005) {
            std::cout << "\r  key: " << (held ? std::string(1, static_cast<char>(held_key)) : std::string("-"))
                      << "  v=" << std::fixed << std::setprecision(2) << std::showpos << ramp.v
                      << "  w=" << std::setprecision(2) << ramp.omega
                      << std::noshowpos
                      << "  RPM L=" << rpm.left << " R=" << rpm.right
                      << "        " << std::flush;
        }
        key_pressed_now = false;

        // ---- Chu kỳ ----
        std::this_thread::sleep_until(loop_start + period);
    }

    // ---- Kết thúc: ramp về 0 + disable ----
    std::cout << "\n\n[3] Stopping...\n";
    for (int i = 0; i < 20 && (std::abs(ramp.v) > 0.001 || std::abs(ramp.omega) > 0.001); i++) {
        ramp.ramp_toward(0.0, 0.0, 0.02, config.accel_limit, config.decel_limit,
                         config.ang_accel_limit);
        const auto rpm = kin.velocity_to_rpm(ramp.v, ramp.omega);
        driver.set_velocity_rpm(rpm.left, rpm.right);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    driver.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    driver.disable();
    bus.close();

    std::cout << "Done.\n";
    return 0;
}
