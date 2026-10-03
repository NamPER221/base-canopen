/**
 * @file mbdv_teleop_keyboard.cpp
 * @brief Teleop Keyboard for Moons' MBDV Dual-Axis (like ros2 teleop_twist_keyboard)
 *
 * Điều khiển robot differential drive bằng bàn phím qua CANopen.
 * Tương đương chức năng của `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.
 *
 * Phím điều khiển:
 *        u    i    o          Tiến-trái  Tiến  Tiến-phải
 *        j    k    l          Quay-trái  Dừng  Quay-phải
 *        m    ,    .          Lùi-trái   Lùi   Lùi-phải
 *
 *   q/z : tăng/giảm tốc độ tuyến tính 10%
 *   w/x : tăng/giảm tốc độ góc 10%
 *   e   : dừng khẩn cấp (quick stop)
 *   r   : reset odometry
 *   t   : hiển thị telemetry chi tiết
 *   SPACE: dừng mềm (v=0, w=0)
 *   ESC/Ctrl+C: thoát
 *
 * Usage:
 *   sudo ./mbdv_teleop_keyboard can0 -1 1 -2 2
 *   sudo ./mbdv_teleop_keyboard can0 -1 1 --single-axis
 *   sudo ./mbdv_teleop_keyboard can0 -1 1 -2 2 --max-v 0.5 --max-w 2.0
 */

#include <canopen/can/raw/socket_can_bus.hpp>
#include <canopen/drivers/mbdv/mbdv_driver.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// =============================================================================
// Terminal Raw Mode
// =============================================================================

namespace {

struct termios g_orig_termios;
bool g_raw_mode_enabled = false;
std::atomic<bool> g_shutdown{false};

void restore_terminal() {
    if (g_raw_mode_enabled) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &g_orig_termios);
        g_raw_mode_enabled = false;
    }
}

void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;    // non-blocking
    raw.c_cc[VTIME] = 1;   // 100ms timeout
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    g_raw_mode_enabled = true;
}

void signal_handler(int sig) {
    (void)sig;
    g_shutdown.store(true);
}

int read_key() {
    char c = 0;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n <= 0) return -1;

    // Handle escape sequences (arrow keys etc.)
    if (c == 27) {
        char seq[2];
        if (::read(STDIN_FILENO, &seq[0], 1) <= 0) return 27;  // bare ESC
        if (::read(STDIN_FILENO, &seq[1], 1) <= 0) return 27;
        if (seq[0] == '[') {
            switch (seq[1]) {
                case 'A': return 'i';  // Up    → forward
                case 'B': return ',';  // Down  → backward
                case 'C': return 'l';  // Right → turn right
                case 'D': return 'j';  // Left  → turn left
            }
        }
        return 27;
    }
    return c;
}

const char* cia402_str(canopen::CiA402State s) {
    switch (s) {
        case canopen::CiA402State::NOT_READY_TO_SWITCH_ON: return "NOT_READY";
        case canopen::CiA402State::SWITCH_ON_DISABLED:     return "SW_DISABLED";
        case canopen::CiA402State::READY_TO_SWITCH_ON:     return "READY_SW_ON";
        case canopen::CiA402State::SWITCHED_ON:            return "SWITCHED_ON";
        case canopen::CiA402State::OPERATION_ENABLED:      return "OPER_ENABLED";
        case canopen::CiA402State::QUICK_STOP_ACTIVE:      return "QUICK_STOP";
        case canopen::CiA402State::FAULT_REACTION_ACTIVE:  return "FAULT_REACT";
        case canopen::CiA402State::FAULT:                  return "FAULT";
        default:                                           return "UNKNOWN";
    }
}

} // anonymous namespace

// =============================================================================
// Main
// =============================================================================

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::atexit(restore_terminal);

    // --- Default Parameters ---
    std::string can_dev   = "can0";
    uint8_t axis1_id      = 1;
    uint8_t axis2_id      = 2;
    bool single_axis      = false;
    bool invert_right     = true;

    canopen::drivers::MbdvKinematicsConfig k_cfg;
    double speed_linear   = 0.2;    // current target v [m/s]
    double speed_angular  = 1.0;    // current target w [rad/s]
    double speed_step_v   = 0.1;    // step for linear  (q/z)
    double speed_step_w   = 0.2;    // step for angular (w/x)
    double max_v          = 2.0;    // hard ceiling [m/s]
    double max_w          = 4.0;    // hard ceiling [rad/s]
    double control_rate   = 200.0;  // loop rate [Hz] (default: 200 Hz)

    // --- Parse Arguments ---
    int arg_idx = 1;
    if (argc > 1 && argv[1][0] != '-') {
        can_dev = argv[1];
        arg_idx = 2;
    }

    for (int i = arg_idx; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-1" || arg == "--axis1") && i + 1 < argc) {
            axis1_id = static_cast<uint8_t>(std::stoi(argv[++i]));
        } else if ((arg == "-2" || arg == "--axis2") && i + 1 < argc) {
            axis2_id = static_cast<uint8_t>(std::stoi(argv[++i]));
        } else if (arg == "-s" || arg == "--single-axis") {
            single_axis = true;
        } else if ((arg == "-r" || arg == "--radius") && i + 1 < argc) {
            k_cfg.wheel_radius_m = std::stod(argv[++i]);
        } else if ((arg == "-l" || arg == "--track") && i + 1 < argc) {
            k_cfg.wheelbase_m = std::stod(argv[++i]);
        } else if ((arg == "-c" || arg == "--cpr") && i + 1 < argc) {
            k_cfg.encoder_cpr = std::stoi(argv[++i]);
        } else if (arg == "--invert-right") {
            invert_right = true;
        } else if (arg == "--no-invert-right") {
            invert_right = false;
        } else if (arg == "--max-v" && i + 1 < argc) {
            max_v = std::stod(argv[++i]);
        } else if (arg == "--max-w" && i + 1 < argc) {
            max_w = std::stod(argv[++i]);
        } else if ((arg == "-f" || arg == "--rate") && i + 1 < argc) {
            control_rate = std::stod(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            std::cout
                << "Usage: " << argv[0] << " [interface] [options]\n\n"
                << "Options:\n"
                << "  -1, --axis1 <id>       Node ID for Axis 1 / Left  (default: 1)\n"
                << "  -2, --axis2 <id>       Node ID for Axis 2 / Right (default: 2)\n"
                << "  -s, --single-axis      Single-axis mode (Axis 1 only)\n"
                << "  -f, --rate <hz>        Control loop frequency in Hz (default: 200)\n"
                << "  -r, --radius <m>       Wheel radius (default: 0.07333)\n"
                << "  -l, --track <m>        Wheel track base (default: 0.4544)\n"
                << "  -c, --cpr <counts>     Encoder CPR (default: 10000)\n"
                << "  --max-v <m/s>          Max linear velocity (default: 2.0)\n"
                << "  --max-w <rad/s>        Max angular velocity (default: 4.0)\n"
                << "  --invert-right         Invert right wheel (default)\n"
                << "  --no-invert-right      Don't invert right wheel\n"
                << "  -h, --help             Show this message\n";
            return 0;
        }
    }

    k_cfg.invert_right = invert_right;
    k_cfg.max_linear_velocity_m_s  = max_v;
    k_cfg.max_angular_velocity_rad_s = max_w;

    // Clamp initial speeds to max
    speed_linear  = std::min(speed_linear, max_v);
    speed_angular = std::min(speed_angular, max_w);

    // --- Banner ---
    std::cout
        << "\n"
        << "╔══════════════════════════════════════════════════════════════╗\n"
        << "║     MBDv Teleop Keyboard  (like teleop_twist_keyboard)     ║\n"
        << "╠══════════════════════════════════════════════════════════════╣\n"
        << "║                                                            ║\n"
        << "║   Moving around:         Arrow keys also work!             ║\n"
        << "║        u    i    o                                         ║\n"
        << "║        j    k    l                                         ║\n"
        << "║        m    ,    .                                         ║\n"
        << "║                                                            ║\n"
        << "║   i / ↑ : forward        u : forward + turn left           ║\n"
        << "║   , / ↓ : backward       o : forward + turn right          ║\n"
        << "║   j / ← : turn left      m : backward + turn left          ║\n"
        << "║   l / → : turn right     . : backward + turn right          ║\n"
        << "║   k / SPACE : stop                                         ║\n"
        << "║                                                            ║\n"
        << "║   q/z : increase/decrease linear speed  by 10%             ║\n"
        << "║   w/x : increase/decrease angular speed by 10%             ║\n"
        << "║   e   : EMERGENCY STOP (quick stop)                        ║\n"
        << "║   r   : reset odometry                                     ║\n"
        << "║   t   : print detailed telemetry                           ║\n"
        << "║   ESC / Ctrl+C : quit                                      ║\n"
        << "║                                                            ║\n"
        << "╚══════════════════════════════════════════════════════════════╝\n"
        << "\n"
        << "  CAN: " << can_dev
        << "  Axis1: " << static_cast<int>(axis1_id)
        << "  Axis2: " << (single_axis ? "DISABLED" : std::to_string(axis2_id))
        << "  Wheel: " << k_cfg.wheel_radius_m << "m"
        << "  Track: " << k_cfg.wheelbase_m << "m"
        << "  CPR: " << k_cfg.encoder_cpr
        << "\n\n";

    // --- Open CAN Bus ---
    canopen::SocketCanBus bus(can_dev);
    int err = bus.open();
    if (err < 0) {
        std::cerr << "ERROR: Cannot open '" << can_dev << "': " << strerror(-err) << "\n"
                  << "  Try: sudo ip link set " << can_dev << " up type can bitrate 500000\n";
        return 1;
    }

    // --- Initialize Driver ---
    canopen::drivers::MbdvDriver driver(axis1_id, axis2_id, &bus);
    driver.set_single_axis_mode(single_axis);
    driver.set_kinematics_config(k_cfg);

    std::cout << "Initializing MBDv driver..." << std::flush;
    if (!driver.init(3000)) {
        std::cerr << "\nWarning: Init incomplete (no nodes responded). Continuing...\n";
    } else {
        std::cout << " OK\n";
    }

    std::cout << "Enabling servo (CiA 402 Servo ON)..." << std::flush;
    if (!driver.enable(std::chrono::milliseconds(5000))) {
        std::cerr << "\nERROR: Failed to enable servo axes!\n";
        driver.disable();
        bus.close();
        return 1;
    }
    std::cout << " OK\n\n";

    // --- Teleop Loop ---
    enable_raw_mode();

    double v_cmd = 0.0;
    double w_cmd = 0.0;
    bool need_send = true;
    bool emergency_stopped = false;

    auto last_odom_time = std::chrono::steady_clock::now();
    auto last_print_time = last_odom_time;
    auto last_cmd_time = last_odom_time;
    uint32_t last_tpdo1_count = 0;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Ready! Use keys to drive. Currently:\n";
    std::cout << "  speed_v = " << speed_linear << " m/s, speed_w = " << speed_angular << " rad/s\n\n";

    while (!g_shutdown.load()) {
        int key = read_key();

        if (key >= 0) {
            bool recognized = true;

            switch (key) {
                // --- Motion ---
                case 'i':  // forward
                    v_cmd = speed_linear;
                    w_cmd = 0.0;
                    break;
                case ',':  // backward
                    v_cmd = -speed_linear;
                    w_cmd = 0.0;
                    break;
                case 'j':  // turn left
                    v_cmd = 0.0;
                    w_cmd = speed_angular;
                    break;
                case 'l':  // turn right
                    v_cmd = 0.0;
                    w_cmd = -speed_angular;
                    break;
                case 'u':  // forward + turn left
                    v_cmd = speed_linear;
                    w_cmd = speed_angular;
                    break;
                case 'o':  // forward + turn right
                    v_cmd = speed_linear;
                    w_cmd = -speed_angular;
                    break;
                case 'm':  // backward + turn left
                    v_cmd = -speed_linear;
                    w_cmd = speed_angular;
                    break;
                case '.':  // backward + turn right
                    v_cmd = -speed_linear;
                    w_cmd = -speed_angular;
                    break;

                // --- Stop ---
                case 'k':
                case ' ':
                    v_cmd = 0.0;
                    w_cmd = 0.0;
                    break;

                // --- Speed Adjust ---
                case 'q':  // increase linear speed
                    speed_linear = std::min(speed_linear + speed_step_v, max_v);
                    std::cout << "\rspeed_v = " << speed_linear
                              << " m/s | speed_w = " << speed_angular << " rad/s       \n";
                    recognized = false;  // don't change motion command
                    break;
                case 'z':  // decrease linear speed
                    speed_linear = std::max(speed_linear - speed_step_v, speed_step_v);
                    std::cout << "\rspeed_v = " << speed_linear
                              << " m/s | speed_w = " << speed_angular << " rad/s       \n";
                    recognized = false;
                    break;
                case 'w':  // increase angular speed
                    speed_angular = std::min(speed_angular + speed_step_w, max_w);
                    std::cout << "\rspeed_v = " << speed_linear
                              << " m/s | speed_w = " << speed_angular << " rad/s       \n";
                    recognized = false;
                    break;
                case 'x':  // decrease angular speed
                    speed_angular = std::max(speed_angular - speed_step_w, speed_step_w);
                    std::cout << "\rspeed_v = " << speed_linear
                              << " m/s | speed_w = " << speed_angular << " rad/s       \n";
                    recognized = false;
                    break;

                // --- Emergency Stop ---
                case 'e':
                case 'E':
                    std::cout << "\r*** EMERGENCY STOP ***                                     \n";
                    driver.quick_stop();
                    v_cmd = 0.0;
                    w_cmd = 0.0;
                    emergency_stopped = true;
                    recognized = false;
                    break;

                // --- Reset Odometry ---
                case 'r':
                case 'R':
                    driver.reset_odometry();
                    std::cout << "\r[Odometry reset to (0, 0, 0)]                              \n";
                    recognized = false;
                    break;

                // --- Telemetry ---
                case 't':
                case 'T': {
                    auto telem = driver.get_telemetry();
                    std::cout << "\n┌─── Telemetry ─────────────────────────────────────┐\n"
                              << "│ Axis 1 [Node " << static_cast<int>(telem.axis1.node_id) << "]\n"
                              << "│   State:    " << cia402_str(telem.axis1.state)
                              << "  SW: 0x" << std::hex << telem.axis1.statusword << std::dec << "\n"
                              << "│   Position: " << telem.axis1.actual_position_counts << " counts\n"
                              << "│   Velocity: " << telem.axis1.actual_velocity_counts_s << " cnt/s ("
                              << telem.axis1.actual_velocity_rpm << " RPM)\n"
                              << "│   Voltage:  " << telem.axis1.voltage_v << " V"
                              << "  Temp: " << telem.axis1.temperature_c << " °C\n"
                              << "│   Alarm:    0x" << std::hex << telem.axis1.dsp_alarm_code << std::dec
                              << "  TPDO#: " << telem.axis1.tpdo_count << "\n";
                    if (!single_axis) {
                        std::cout
                              << "│ Axis 2 [Node " << static_cast<int>(telem.axis2.node_id) << "]\n"
                              << "│   State:    " << cia402_str(telem.axis2.state)
                              << "  SW: 0x" << std::hex << telem.axis2.statusword << std::dec << "\n"
                              << "│   Position: " << telem.axis2.actual_position_counts << " counts\n"
                              << "│   Velocity: " << telem.axis2.actual_velocity_counts_s << " cnt/s ("
                              << telem.axis2.actual_velocity_rpm << " RPM)\n"
                              << "│   Voltage:  " << telem.axis2.voltage_v << " V"
                              << "  Temp: " << telem.axis2.temperature_c << " °C\n"
                              << "│   Alarm:    0x" << std::hex << telem.axis2.dsp_alarm_code << std::dec
                              << "  TPDO#: " << telem.axis2.tpdo_count << "\n";
                    }
                    std::cout << "│ Odometry: x=" << telem.pose.x << " y=" << telem.pose.y
                              << " θ=" << (telem.pose.theta * 180.0 / M_PI) << "°\n"
                              << "│ Twist:    v=" << telem.twist.linear_v
                              << " m/s  w=" << telem.twist.angular_w << " rad/s\n"
                              << "└───────────────────────────────────────────────────┘\n";
                    recognized = false;
                    break;
                }

                // --- Quit ---
                case 27:   // ESC
                    g_shutdown.store(true);
                    recognized = false;
                    break;

                default:
                    recognized = false;
                    break;
            }

            if (recognized) {
                need_send = true;
                // Re-enable after emergency stop
                if (emergency_stopped && (v_cmd != 0.0 || w_cmd != 0.0)) {
                    std::cout << "\rRe-enabling after emergency stop..." << std::flush;
                    driver.fault_reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    driver.enable(std::chrono::milliseconds(3000));
                    emergency_stopped = false;
                    std::cout << " OK\n";
                }
            }
        }

        // --- Send command cyclically at control_rate Hz ---
        const double cmd_period = 1.0 / control_rate;
        auto now = std::chrono::steady_clock::now();
        double dt_cmd = std::chrono::duration<double>(now - last_cmd_time).count();

        if (need_send || dt_cmd >= cmd_period) {
            if (!emergency_stopped) {
                driver.set_cmd_vel(v_cmd, w_cmd);
            }
            last_cmd_time = now;
            need_send = false;
        }

        // --- Update Odometry ---
        double dt_odom = std::chrono::duration<double>(now - last_odom_time).count();
        if (dt_odom >= cmd_period) {
            driver.update_odometry(dt_odom);
            last_odom_time = now;
        }

        // --- Print Status Bar at 5 Hz ---
        double dt_print = std::chrono::duration<double>(now - last_print_time).count();
        if (dt_print >= 0.2) {
            auto pose = driver.get_pose();
            auto telem = driver.get_telemetry();

            double actual_fb_hz = (dt_print > 0.0) ? (telem.axis1.tpdo_count - last_tpdo1_count) / dt_print : 0.0;
            last_tpdo1_count = telem.axis1.tpdo_count;

            std::cout << "\rv=" << std::setw(6) << v_cmd
                      << " w=" << std::setw(6) << w_cmd
                      << " | odom: x=" << std::setw(6) << pose.x
                      << " y=" << std::setw(6) << pose.y
                      << " θ=" << std::setw(6) << std::setprecision(1)
                      << (pose.theta * 180.0 / M_PI) << "°"
                      << std::setprecision(3)
                      << " | FB: " << std::setw(3) << static_cast<int>(std::round(actual_fb_hz)) << "Hz"
                      << " [CMD: " << static_cast<int>(control_rate) << "Hz]   " << std::flush;

            last_print_time = now;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    // --- Cleanup ---
    restore_terminal();
    std::cout << "\n\nStopping motors...\n";

    for (int i = 0; i < 5; ++i) {
        driver.set_cmd_vel(0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    driver.disable();

    auto final_pose = driver.get_pose();
    std::cout << "Final pose: x=" << final_pose.x
              << " y=" << final_pose.y
              << " θ=" << (final_pose.theta * 180.0 / M_PI) << "°\n"
              << "Exited safely.\n";

    bus.close();
    return 0;
}
