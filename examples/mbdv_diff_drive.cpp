/**
 * @file mbdv_diff_drive.cpp
 * @brief Differential Drive Control & Odometry for Moons' MBDV Dual-Axis Servo
 *
 * Example program integrating Moons' MBDV Dual-Axis Driver (MBDV-2X-520AC)
 * with Differential Drive Kinematics (cmd_vel -> Wheel Speeds & Odometry).
 *
 * Usage Examples:
 *   1. Monitor telemetry only:
 *      ./mbdv_diff_drive can0 -1 1 -2 2 --monitor
 *
 *   2. Forward motion test (v = 0.2 m/s, w = 0.0 rad/s, for 3 seconds):
 *      ./mbdv_diff_drive can0 -1 1 -2 2 -v 0.2 -w 0.0 -t 3.0
 *
 *   3. In-place rotation test (v = 0.0 m/s, w = 0.5 rad/s, for 3 seconds):
 *      ./mbdv_diff_drive can0 -1 1 -2 2 -v 0.0 -w 0.5 -t 3.0
 *
 *   4. Single-axis mode (Axis 1 only @ 500k):
 *      ./mbdv_diff_drive can0 -1 1 --single-axis -v 0.2 -t 3.0
 */

#include <canopen/can/raw/socket_can_bus.hpp>
#include <canopen/drivers/mbdv/mbdv_driver.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        g_shutdown_requested.store(true);
    }
}

void print_help(const char* prog) {
    std::cout << "Usage: " << prog << " [interface] [options]\n\n"
              << "Arguments:\n"
              << "  interface               CAN interface name (default: can0)\n\n"
              << "Options:\n"
              << "  -1, --axis1 <id>        Node ID for Axis 1 / Left (default: 1)\n"
              << "  -2, --axis2 <id>        Node ID for Axis 2 / Right (default: 2)\n"
              << "  -s, --single-axis       Single-axis mode (control Axis 1 only)\n"
              << "  -v, --linear <m/s>      Target linear velocity (default: 0.2 m/s)\n"
              << "  -w, --angular <rad/s>   Target angular velocity (default: 0.0 rad/s)\n"
              << "  -t, --time <sec>        Motion duration in seconds (default: 3.0)\n"
              << "  -r, --radius <m>        Wheel radius in meters (default: 0.07333)\n"
              << "  -l, --track <m>         Wheel track base in meters (default: 0.4544)\n"
              << "  -c, --cpr <counts>      Encoder counts per revolution (default: 10000)\n"
              << "  --invert-right          Invert right wheel direction (default: true for diff drive)\n"
              << "  -m, --monitor           Continuous telemetry monitoring mode (no motion)\n"
              << "  -h, --help              Display this help message\n"
              << std::endl;
}

const char* cia402_state_str(canopen::CiA402State s) {
    switch (s) {
        case canopen::CiA402State::NOT_READY_TO_SWITCH_ON: return "NOT_READY";
        case canopen::CiA402State::SWITCH_ON_DISABLED:     return "SWITCH_DISABLED";
        case canopen::CiA402State::READY_TO_SWITCH_ON:     return "READY_SWITCH_ON";
        case canopen::CiA402State::SWITCHED_ON:             return "SWITCHED_ON";
        case canopen::CiA402State::OPERATION_ENABLED:      return "OPER_ENABLED";
        case canopen::CiA402State::QUICK_STOP_ACTIVE:      return "QUICK_STOP";
        case canopen::CiA402State::FAULT_REACTION_ACTIVE:  return "FAULT_REACT";
        case canopen::CiA402State::FAULT:                  return "FAULT";
        default:                                           return "UNKNOWN";
    }
}
} // anonymous namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string can_dev = "can0";
    uint8_t axis1_id = 1;
    uint8_t axis2_id = 2;
    bool single_axis = false;
    double target_v = 0.2;
    double target_w = 0.0;
    double duration_s = 3.0;
    bool monitor_only = false;
    bool invert_right = true;

    canopen::drivers::MbdvKinematicsConfig k_cfg;

    // Parse arguments
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
        } else if ((arg == "-v" || arg == "--linear") && i + 1 < argc) {
            target_v = std::stod(argv[++i]);
        } else if ((arg == "-w" || arg == "--angular") && i + 1 < argc) {
            target_w = std::stod(argv[++i]);
        } else if ((arg == "-t" || arg == "--time") && i + 1 < argc) {
            duration_s = std::stod(argv[++i]);
        } else if ((arg == "-r" || arg == "--radius") && i + 1 < argc) {
            k_cfg.wheel_radius_m = std::stod(argv[++i]);
        } else if ((arg == "-l" || arg == "--track") && i + 1 < argc) {
            k_cfg.wheelbase_m = std::stod(argv[++i]);
        } else if ((arg == "-c" || arg == "--cpr") && i + 1 < argc) {
            k_cfg.encoder_cpr = std::stoi(argv[++i]);
        } else if (arg == "--invert-right") {
            invert_right = true;
        } else if (arg == "-m" || arg == "--monitor") {
            monitor_only = true;
        } else if (arg == "-h" || arg == "--help") {
            print_help(argv[0]);
            return 0;
        }
    }

    k_cfg.invert_right = invert_right;

    std::cout << "=========================================================\n"
              << "  Moons' MBDV Dual-Axis CANopen Master (Differential Drive)\n"
              << "=========================================================\n"
              << "CAN Interface: " << can_dev << "\n"
              << "Axis 1 Node:   " << static_cast<int>(axis1_id) << "\n"
              << "Axis 2 Node:   " << (single_axis ? "DISABLED" : std::to_string(axis2_id)) << "\n"
              << "Wheel Radius:  " << k_cfg.wheel_radius_m << " m\n"
              << "Wheel Base:    " << k_cfg.wheelbase_m << " m\n"
              << "Encoder CPR:   " << k_cfg.encoder_cpr << " counts/rev\n"
              << "Mode:          " << (monitor_only ? "MONITOR ONLY" : "MOTION TEST") << "\n"
              << "=========================================================\n"
              << std::endl;

    canopen::SocketCanBus bus(can_dev);
    int err = bus.open();
    if (err < 0) {
        std::cerr << "Failed to open SocketCAN interface '" << can_dev << "': "
                  << strerror(-err) << "\n"
                  << "Ensure the interface is up: 'sudo ip link set " << can_dev
                  << " up type can bitrate 500000'" << std::endl;
        return 1;
    }

    canopen::drivers::MbdvDriver driver(axis1_id, axis2_id, &bus);
    driver.set_single_axis_mode(single_axis);
    driver.set_kinematics_config(k_cfg);

    std::cout << "Initializing Moons' MBDV CANopen driver..." << std::endl;
    if (!driver.init(2000)) {
        std::cerr << "Warning: Driver initialization incomplete (no nodes responded on bootup).\n"
                  << "Continuing to check bus communications..." << std::endl;
    }

    if (!monitor_only) {
        std::cout << "Enabling Servo Operation (CiA 402 Servo ON)..." << std::endl;
        if (!driver.enable(std::chrono::milliseconds(3000))) {
            std::cerr << "Failed to enable both servo axes! Aborting motion test." << std::endl;
            driver.disable();
            bus.close();
            return 1;
        }

        std::cout << "\n>>> Phase 1: Commanding Motion: v = " << target_v
                  << " m/s, w = " << target_w << " rad/s for " << duration_s << " s..."
                  << std::endl;

        driver.set_cmd_vel(target_v, target_w);

        auto start_time = std::chrono::steady_clock::now();
        auto last_time = start_time;

        while (!g_shutdown_requested.load() &&
               std::chrono::steady_clock::now() - start_time <
                   std::chrono::duration<double>(duration_s)) {
            // Cyclically stream velocity command (RPDO3) to refresh motion and avoid watchdog trip
            driver.set_cmd_vel(target_v, target_w);

            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time).count();
            last_time = now;

            driver.update_odometry(dt);
            auto telem = driver.get_telemetry();

            std::cout << std::fixed << std::setprecision(3)
                      << "[ODOM] x: " << telem.pose.x << " m | y: " << telem.pose.y
                      << " m | th: " << telem.pose.theta << " rad | v: "
                      << telem.twist.linear_v << " m/s | "
                      << "A1[SW:0x" << std::hex << telem.axis1.statusword << std::dec
                      << " v:" << telem.axis1.actual_velocity_counts_s
                      << " p:" << telem.axis1.actual_position_counts
                      << " pdo:" << telem.axis1.tpdo_count << "] "
                      << "A2[SW:0x" << std::hex << telem.axis2.statusword << std::dec
                      << " v:" << telem.axis2.actual_velocity_counts_s
                      << " p:" << telem.axis2.actual_position_counts
                      << " pdo:" << telem.axis2.tpdo_count << "]\n"
                      << std::flush;

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::cout << "\n";

        // Stop motion
        std::cout << "\n>>> Phase 2: Stopping robot (v = 0, w = 0)..." << std::endl;
        for (int i = 0; i < 5; ++i) {
            driver.set_cmd_vel(0.0, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::cout << "Disabling servo drives..." << std::endl;
        driver.disable();
        std::cout << "Motion test completed successfully!" << std::endl;

    } else {
        std::cout << "Running in Telemetry Monitor mode (Press Ctrl+C to exit)..." << std::endl;
        auto last_time = std::chrono::steady_clock::now();

        while (!g_shutdown_requested.load()) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time).count();
            last_time = now;

            driver.update_odometry(dt);
            auto telem = driver.get_telemetry();

            std::cout << std::fixed << std::setprecision(3)
                      << "[TELEMETRY] "
                      << "Pose: (" << telem.pose.x << ", " << telem.pose.y << ", "
                      << telem.pose.theta << " rad) | "
                      << "A1: [" << cia402_state_str(telem.axis1.state)
                      << " | pos: " << telem.axis1.actual_position_counts
                      << " | vel: " << telem.axis1.actual_velocity_counts_s << " cnt/s] | "
                      << "A2: [" << cia402_state_str(telem.axis2.state)
                      << " | pos: " << telem.axis2.actual_position_counts
                      << " | vel: " << telem.axis2.actual_velocity_counts_s << " cnt/s]\n"
                      << std::flush;

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    if (!monitor_only) {
        driver.set_cmd_vel(0.0, 0.0);
        driver.disable();
    }
    bus.close();
    std::cout << "\nExited cleanly." << std::endl;
    return 0;
}
