/**
 * @file dual_motor_keyboard.cpp
 * @brief Dual Motor Control với Keyboard Input và Config File
 *
 * Điều khiển 2 động cơ ZLAC8015D bằng bàn phím với:
 * - Inverse Kinematics: cmd_vel → wheel RPM
 * - Forward Kinematics: encoder → odometry
 * - Config từ file YAML
 *
 * Compile:
 *   cd build && cmake .. && make dual_motor_keyboard
 *
 * Run:
 *   sudo ./examples/dual_motor_keyboard
 *   sudo ./examples/dual_motor_keyboard --config ../config/zlac8015_config.yaml
 *   sudo ./examples/dual_motor_keyboard can0 1  # Quick start (interface, node_id)
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
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <fstream>
#include <sstream>

#if HAVE_YAML
#include <yaml-cpp/yaml.h>
#endif

using namespace canopen;

// =============================================================================
// Configuration Structure
// =============================================================================

struct Config {
    // CANopen
    std::string can_interface = "can0";
    uint8_t motor_node_id = 1;
    uint32_t sdo_timeout_ms = 200;

    // Motor specs
    double wheel_radius = 0.0865;     // meters
    double wheelbase = 0.400;         // meters
    int16_t max_rpm = 1000;
    int16_t min_rpm = -1000;

    // Control
    double control_rate_hz = 50.0;
    double max_linear_velocity = 0.5;  // m/s
    double max_angular_velocity = 2.0; // rad/s
    double default_speed_scale = 0.5;

    // Debug
    bool verbose = false;
    double odom_print_rate_hz = 5.0;

    // Load from YAML config file
    void loadFromFile(const std::string& filename) {
#if HAVE_YAML
        try {
            YAML::Node config = YAML::LoadFile(filename);

            if (config["canopen"]) {
                const auto& can = config["canopen"];
                can_interface = can["interface"].as<std::string>(can_interface);
                motor_node_id = can["node_id"].as<uint8_t>(motor_node_id);
                sdo_timeout_ms = can["sdo_timeout_ms"].as<uint32_t>(sdo_timeout_ms);
            }

            if (config["motor"]) {
                const auto& motor = config["motor"];
                wheel_radius = motor["wheel_radius_m"].as<double>(wheel_radius);
                wheelbase = motor["wheelbase_m"].as<double>(wheelbase);
                max_rpm = motor["max_rpm"].as<int16_t>(max_rpm);
                min_rpm = motor["min_rpm"].as<int16_t>(min_rpm);
            }

            if (config["control"]) {
                const auto& ctrl = config["control"];
                control_rate_hz = ctrl["control_rate_hz"].as<double>(control_rate_hz);
                max_linear_velocity = ctrl["max_linear_velocity"].as<double>(max_linear_velocity);
                max_angular_velocity = ctrl["max_angular_velocity"].as<double>(max_angular_velocity);
                default_speed_scale = ctrl["default_speed_scale"].as<double>(default_speed_scale);
            }

            if (config["debug"]) {
                const auto& dbg = config["debug"];
                verbose = dbg["verbose"].as<bool>(verbose);
                odom_print_rate_hz = dbg["odom_print_rate_hz"].as<double>(odom_print_rate_hz);
            }

            std::cout << "[Config] Loaded from: " << filename << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[Config] Error loading " << filename << ": " << e.what() << "\n";
        }
#else
        (void)filename;
        std::cout << "[Config] YAML support not available, using defaults\n";
#endif
    }

    void print() const {
        std::cout << "\n";
        std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                    CONFIGURATION                               ║\n";
        std::cout << "╠═══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║  CAN Interface:     " << std::setw(40) << std::left << can_interface << "║\n";
        std::cout << "║  Node ID:          " << std::setw(40) << std::left << (int)motor_node_id << "║\n";
        std::cout << "║  SDO Timeout:      " << std::setw(40) << std::left << sdo_timeout_ms << " ms║\n";
        std::cout << "╠═══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║  Wheel Radius:     " << std::setw(40) << std::left << wheel_radius << " m║\n";
        std::cout << "║  Wheelbase:        " << std::setw(40) << std::left << wheelbase << " m║\n";
        std::cout << "║  Max RPM:          " << std::setw(40) << std::left << max_rpm << "║\n";
        std::cout << "╠═══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║  Max Linear Vel:   " << std::setw(40) << std::left << max_linear_velocity << " m/s║\n";
        std::cout << "║  Max Angular Vel:  " << std::setw(40) << std::left << max_angular_velocity << " rad/s║\n";
        std::cout << "║  Control Rate:     " << std::setw(40) << std::left << control_rate_hz << " Hz║\n";
        std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";
    }
} config;

// =============================================================================
// Kinematics Class
// =============================================================================

// Differential drive kinematics — dùng thư viện canopen_kinematics
class DifferentialKinematics {
public:
    DifferentialKinematics(const Config& cfg)
        : cfg_(cfg),
          lib_(cfg.wheel_radius, cfg.wheelbase,
               static_cast<double>(cfg.max_rpm)) {}

    using Pose = kinematics::DifferentialDriveKinematics::Pose;

    Pose& pose() { return pose_; }
    const Pose& pose() const { return pose_; }

    struct WheelRPM { int16_t left, right; };

    WheelRPM velocityToRPM(double v, double omega) const {
        auto rpm = lib_.velocity_to_rpm(v, omega);
        return {rpm.left, rpm.right};
    }

    void updateFromEncoders(double phi_L, double phi_R, double dt) {
        if (first_update_) {
            prev_phi_L_ = phi_L;
            prev_phi_R_ = phi_R;
            first_update_ = false;
            return;
        }
        if (dt <= 0) return;

        lib_.update(phi_L - prev_phi_L_, phi_R - prev_phi_R_, dt);
        prev_phi_L_ = phi_L;
        prev_phi_R_ = phi_R;

        const auto p = lib_.get_pose();
        pose_.x = p.x;
        pose_.y = p.y;
        pose_.theta = p.theta;

        while (pose_.theta > M_PI) pose_.theta -= 2.0 * M_PI;
        while (pose_.theta < -M_PI) pose_.theta += 2.0 * M_PI;
    }

    void resetPose(double x = 0, double y = 0, double theta = 0) {
        lib_.reset_pose(x, y, theta);
        const auto p = lib_.get_pose();
        pose_.x = p.x;
        pose_.y = p.y;
        pose_.theta = p.theta;
        first_update_ = true;
    }

private:
    const Config& cfg_;
    kinematics::DifferentialDriveKinematics lib_;
    Pose pose_;
    double prev_phi_L_{0.0}, prev_phi_R_{0.0};
    bool first_update_{true};
};


// =============================================================================
// CANopen Helpers
// =============================================================================

bool sendNMT(SocketCAN& can, uint8_t node_id, uint8_t cmd) {
    struct can_frame frame;
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = cmd;
    frame.data[1] = node_id;
    for (int i = 2; i < 8; i++) frame.data[i] = 0;
    return can.write(frame) >= 0;
}

bool enableMotor(SocketCAN& can, uint8_t node_id) {
    struct can_frame frame;
    frame.can_id = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x23; frame.data[1] = 0x40; frame.data[2] = 0x60;
    frame.data[3] = 0x00; frame.data[4] = 0x0F;
    for (int i = 5; i < 8; i++) frame.data[i] = 0;
    return can.write(frame) >= 0;
}

bool disableMotor(SocketCAN& can, uint8_t node_id) {
    struct can_frame frame;
    frame.can_id = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x23; frame.data[1] = 0x40; frame.data[2] = 0x60;
    frame.data[3] = 0x00; frame.data[4] = 0x00;
    for (int i = 5; i < 8; i++) frame.data[i] = 0;
    return can.write(frame) >= 0;
}

bool setVelocity(SocketCAN& can, uint8_t node_id, int16_t left_rpm, int16_t right_rpm) {
    struct can_frame frame;
    frame.can_id = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x23; frame.data[1] = 0xFF; frame.data[2] = 0x60;
    frame.data[3] = 0x03;
    frame.data[4] = static_cast<uint8_t>(left_rpm & 0xFF);
    frame.data[5] = static_cast<uint8_t>((left_rpm >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>(right_rpm & 0xFF);
    frame.data[7] = static_cast<uint8_t>((right_rpm >> 8) & 0xFF);
    return can.write(frame) >= 0;
}

bool readStatus(SocketCAN& can, uint8_t node_id, uint16_t& status) {
    struct can_frame req;
    req.can_id = 0x600 + node_id;
    req.can_dlc = 8;
    req.data[0] = 0x40; req.data[1] = 0x41; req.data[2] = 0x60;
    req.data[3] = 0x00;
    for (int i = 4; i < 8; i++) req.data[i] = 0;
    if (can.write(req) < 0) return false;

    struct can_frame resp;
    struct timespec timeout{0, config.sdo_timeout_ms * 1000000};
    for (int i = 0; i < 5; i++) {
        ssize_t n = can.read(resp, &timeout);
        if (n > 0 && resp.can_id == (0x580u + node_id)) {
            status = resp.data[4] | (resp.data[5] << 8);
            return true;
        }
    }
    return false;
}

bool readVelocity(SocketCAN& can, uint8_t node_id, int16_t& left, int16_t& right) {
    struct can_frame req;
    req.can_id = 0x600 + node_id;
    req.can_dlc = 8;
    req.data[0] = 0x40; req.data[1] = 0x6C; req.data[2] = 0x60; req.data[3] = 0x01;
    for (int i = 4; i < 8; i++) req.data[i] = 0;
    if (can.write(req) < 0) return false;

    struct can_frame resp;
    struct timespec timeout{0, config.sdo_timeout_ms * 1000000};
    bool left_ok = false, right_ok = false;

    for (int i = 0; i < 10 && (!left_ok || !right_ok); i++) {
        ssize_t n = can.read(resp, &timeout);
        if (n > 0 && resp.can_id == (0x580u + node_id)) {
            if (resp.data[3] == 0x01) {
                left = resp.data[4] | (resp.data[5] << 8);
                left_ok = true;
            } else if (resp.data[3] == 0x02) {
                right = resp.data[4] | (resp.data[5] << 8);
                right_ok = true;
            }
        }
    }
    return left_ok && right_ok;
}

// =============================================================================
// Keyboard Input
// =============================================================================

int kbhit() {
    struct termios oldt, newt;
    int ch, oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

char getChar() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    char ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

void enableRawMode() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &term);
}

void disableRawMode() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= ICANON | ECHO;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &term);
}

// =============================================================================
// Main
// =============================================================================

std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
}

void printHelp() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║            KEYBOARD CONTROL - DUAL MOTOR                     ║\n";
    std::cout << "╠═══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║                                                           ║\n";
    std::cout << "║           [w]            FORWARD                         ║\n";
    std::cout << "║        [a] [s] [d]     LEFT   BACKWARD  RIGHT          ║\n";
    std::cout << "║                                                           ║\n";
    std::cout << "║   [1-9]: Speed (1=slowest, 9=fastest, 5=50%)           ║\n";
    std::cout << "║   [0]: Reset odometry                                   ║\n";
    std::cout << "║   [r]: Read motor status                                ║\n";
    std::cout << "║   [c]: Show config                                       ║\n";
    std::cout << "║   [q]: Quit                                             ║\n";
    std::cout << "║                                                           ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";
}

void printStatusBar(double v, double omega, int16_t cmd_L, int16_t cmd_R,
                    int16_t fb_L, int16_t fb_R, const DifferentialKinematics::Pose& pose) {
    std::cout << "\r";
    std::cout << "v=" << std::setw(5) << std::fixed << std::setprecision(2) << v << " m/s "
              << "ω=" << std::setw(5) << std::setprecision(2) << omega << " rad/s | "
              << "CMD: L=" << std::setw(5) << cmd_L << " R=" << std::setw(5) << cmd_R << " | "
              << "FB: L=" << std::setw(5) << fb_L << " R=" << std::setw(5) << fb_R << " | "
              << "x=" << std::setw(6) << std::setprecision(2) << pose.x
              << " y=" << std::setw(6) << pose.y
              << " θ=" << std::setw(6) << std::setprecision(1) << (pose.theta * 180.0 / M_PI) << "°  "
              << std::flush;
}

void printBanner() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║      DUAL MOTOR CONTROL with INVERSE KINEMATICS             ║\n";
    std::cout << "║      Real-time Keyboard Control                               ║\n";
    std::cout << "╠═══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  Interface: " << std::setw(48) << std::left << config.can_interface << "║\n";
    std::cout << "║  Node ID:   " << std::setw(48) << std::left << (int)config.motor_node_id << "║\n";
    std::cout << "║  Max v: " << config.max_linear_velocity << " m/s, Max ω: " << config.max_angular_velocity << " rad/s    ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);

    // Parse arguments
    std::string config_file;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--config" || arg == "-c") {
            if (i + 1 < argc) config_file = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --config, -c <file>   Config file (YAML)\n";
            std::cout << "  --help, -h            Show this help\n";
            std::cout << "\nQuick start:\n";
            std::cout << "  " << argv[0] << " can0 1\n";
            std::cout << "  " << argv[0] << " --config config/zlac8015_config.yaml\n";
            return 0;
        } else if (arg == "can0" || arg == "can1" || arg == "vcan0") {
            config.can_interface = arg;
            if (i + 1 < argc && isdigit(argv[i + 1][0])) {
                config.motor_node_id = std::stoi(argv[++i]);
            }
        } else if (isdigit(arg[0])) {
            config.motor_node_id = std::stoi(arg);
        }
    }

    // Load config file if specified
    if (!config_file.empty()) {
        config.loadFromFile(config_file);
    }

    printBanner();
    config.print();

    // Open CAN
    SocketCAN can(config.can_interface);
    if (can.open() < 0) {
        std::cerr << "ERROR: Cannot open " << config.can_interface << "\n";
        return 1;
    }

    // Initialize
    sendNMT(can, config.motor_node_id, 0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!enableMotor(can, config.motor_node_id)) {
        std::cerr << "ERROR: Cannot enable motors\n";
        can.close();
        return 1;
    }

    // Check status
    uint16_t status;
    if (readStatus(can, config.motor_node_id, status)) {
        std::cout << "Motor status: 0x" << std::hex << status << std::dec;
        if (status & 0x0004) std::cout << " (Enabled)";
        if (status & 0x0008) std::cout << " (FAULT!)";
        std::cout << "\n";
    }

    // Initialize kinematics
    DifferentialKinematics kinematics(config);
    kinematics.resetPose();

    // Control state
    double v_cmd = 0.0;
    double omega_cmd = 0.0;
    double speed_scale = config.default_speed_scale;
    int16_t cmd_L = 0, cmd_R = 0;
    int16_t fb_L = 0, fb_R = 0;
    bool new_command = true;

    // Enable raw mode
    enableRawMode();
    printHelp();

    std::cout << "Use keyboard to control. Press [q] to quit.\n\n";

    auto last_control = std::chrono::steady_clock::now();

    while (g_running) {
        // Check keyboard
        if (kbhit()) {
            char ch = getChar();

            switch (ch) {
                case 'w': case 'W':
                    v_cmd = config.max_linear_velocity;
                    omega_cmd = 0.0;
                    new_command = true;
                    break;
                case 's': case 'S':
                    v_cmd = -config.max_linear_velocity;
                    omega_cmd = 0.0;
                    new_command = true;
                    break;
                case 'a': case 'A':
                    v_cmd = 0.0;
                    omega_cmd = config.max_angular_velocity;
                    new_command = true;
                    break;
                case 'd': case 'D':
                    v_cmd = 0.0;
                    omega_cmd = -config.max_angular_velocity;
                    new_command = true;
                    break;
                case 'q': case 'Q':
                    g_running = false;
                    break;
                case '0':
                    kinematics.resetPose();
                    std::cout << "\n[Odometry reset]\n";
                    break;
                case 'c': case 'C':
                    config.print();
                    break;
                case 'r': case 'R':
                    if (readStatus(can, config.motor_node_id, status)) {
                        std::cout << "\n[Motor Status: 0x" << std::hex << status << std::dec << "]\n";
                        std::cout << "  Enabled: " << ((status & 0x0004) ? "YES" : "NO") << "\n";
                        std::cout << "  Fault: " << ((status & 0x0008) ? "YES" : "NO") << "\n";
                    }
                    break;
                case '1': speed_scale = 0.1; new_command = true; std::cout << "\n[Speed: 10%]\n"; break;
                case '2': speed_scale = 0.2; new_command = true; std::cout << "\n[Speed: 20%]\n"; break;
                case '3': speed_scale = 0.3; new_command = true; std::cout << "\n[Speed: 30%]\n"; break;
                case '4': speed_scale = 0.4; new_command = true; std::cout << "\n[Speed: 40%]\n"; break;
                case '5': speed_scale = 0.5; new_command = true; std::cout << "\n[Speed: 50%]\n"; break;
                case '6': speed_scale = 0.6; new_command = true; std::cout << "\n[Speed: 60%]\n"; break;
                case '7': speed_scale = 0.7; new_command = true; std::cout << "\n[Speed: 70%]\n"; break;
                case '8': speed_scale = 0.8; new_command = true; std::cout << "\n[Speed: 80%]\n"; break;
                case '9': speed_scale = 1.0; new_command = true; std::cout << "\n[Speed: 100%]\n"; break;
            }
        }

        // Control loop
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_control).count();

        if (dt >= 1.0 / config.control_rate_hz) {
            last_control = now;

            double v = v_cmd * speed_scale;
            double omega = omega_cmd * speed_scale;

            auto rpm = kinematics.velocityToRPM(v, omega);
            cmd_L = rpm.left;
            cmd_R = rpm.right;

            if (new_command) {
                setVelocity(can, config.motor_node_id, cmd_L, cmd_R);
                new_command = false;
            }

            readVelocity(can, config.motor_node_id, fb_L, fb_R);
            printStatusBar(v, omega, cmd_L, cmd_R, fb_L, fb_R, kinematics.pose());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Cleanup
    disableRawMode();
    std::cout << "\n\nStopping motors...\n";
    setVelocity(can, config.motor_node_id, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    disableMotor(can, config.motor_node_id);
    can.close();

    std::cout << "Final pose: x=" << std::fixed << std::setprecision(3) << kinematics.pose().x
              << " y=" << kinematics.pose().y
              << " θ=" << (kinematics.pose().theta * 180.0 / M_PI) << "°\n";
    std::cout << "Exited safely.\n";

    return 0;
}
