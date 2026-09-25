/**
 * @file test_motor_control.cpp
 * @brief ZLAC8015D Motor Control Tests — dùng driver library
 *
 * Test suite for ZLAC8015D dual-axis servo driver. Chạy trên hardware
 * thật (can0) hoặc vcan0. Sử dụng ZLAC8015Driver + DifferentialDrive
 * Kinematics từ thư viện (canopen_zlac + canopen_kinematics).
 *
 * Usage:
 *   ./test_motor_control
 *   ./test_motor_control --config ../config/zlac8015_config.yaml
 *   ./test_motor_control can0 1
 */

#include <canopen/drivers/zlac8015/zlac8015_driver.hpp>
#include <canopen/kinematics/differential_drive.hpp>
#include <canopen/can/raw/socket_can_bus.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>
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

struct MotorConfig {
    std::string can_interface = "can0";
    uint8_t motor_node_id = 1;
    uint32_t sdo_timeout_ms = 500;

    double wheel_radius = 0.0865;     // meters
    double wheelbase = 0.400;         // meters
    int16_t max_rpm = 1000;
    int16_t min_rpm = -1000;

    int16_t test_rpm = 50;
    double rpm_tolerance = 0.15;     // 15% tolerance

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

            std::cout << "[Config] Loaded from: " << filename << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[Config] Error loading " << filename << ": " << e.what() << "\n";
        }
#else
        (void)filename;
        std::cout << "[Config] YAML support not available, using defaults\n";
#endif
    }

    void loadFromArgs(int argc, char* argv[]) {
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "can0" || arg == "can1" || arg == "vcan0") {
                can_interface = arg;
            } else if (isdigit(arg[0])) {
                motor_node_id = static_cast<uint8_t>(std::stoi(arg));
            }
        }
    }

    void print() const {
        std::cout << "\n";
        std::cout << "  CAN Interface: " << can_interface << "\n";
        std::cout << "  Node ID:       " << static_cast<int>(motor_node_id) << "\n";
        std::cout << "  SDO Timeout:   " << sdo_timeout_ms << " ms\n";
        std::cout << "  Wheel Radius:  " << wheel_radius << " m\n";
        std::cout << "  Wheelbase:     " << wheelbase << " m\n";
        std::cout << "  Max RPM:       " << max_rpm << "\n";
        std::cout << "  Test RPM:      " << test_rpm << " RPM\n";
        std::cout << "  Tolerance:     " << static_cast<int>(rpm_tolerance * 100) << " %\n";
        std::cout << "\n";
    }
} config;

// ============================================================================
// Test Framework
// ============================================================================

static int tests_passed = 0;
static int tests_failed = 0;

#define RUN_TEST(name, ...)                                                     \
    do {                                                                        \
        try {                                                                   \
            name(__VA_ARGS__);                                                  \
            std::cout << "  " #name " PASSED\n";                                \
            tests_passed++;                                                     \
        } catch (std::exception& e) {                                           \
            std::cout << "  " #name " FAILED: " << e.what() << "\n";            \
            tests_failed++;                                                     \
        }                                                                       \
    } while (0)

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) throw std::runtime_error("Assertion failed: " #cond); \
} while(0)

#define ASSERT_NEAR(a, b, tol) do { \
    if (std::abs((a) - (b)) > (tol)) \
        throw std::runtime_error("Expected " + std::to_string(a) + " ~ " + std::to_string(b)); \
} while(0)

// ============================================================================
// Test Cases (dùng ZLAC8015Driver từ thư viện)
// ============================================================================

void test_can_connection(ZLAC8015Driver& driver, SocketCanBus& bus) {
    std::cout << "(Opening CAN interface: " << bus.interface_name() << ")\n";
    ASSERT_TRUE(bus.is_up() || bus.socket().is_open());
    ASSERT_TRUE(driver.init());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void test_read_initial_status(ZLAC8015Driver& driver) {
    uint16_t sw = driver.read_status();
    std::cout << "(Initial status: 0x" << std::hex << sw << std::dec << ")\n";
    std::cout << "  Fault: " << ((sw & 0x0008) ? "YES" : "NO") << "\n";
    std::cout << "  Enabled: " << ((sw & 0x0004) ? "YES" : "NO") << "\n";
    ASSERT_TRUE((sw & 0x0040) || (sw & 0x0021) || (sw & 0x0023));
}

void test_enable_motor(ZLAC8015Driver& driver) {
    std::cout << "(Enabling motor)\n";
    ASSERT_TRUE(driver.enable());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint16_t sw = driver.read_status();
    std::cout << "(Status after enable: 0x" << std::hex << sw << std::dec << ")\n";
    ASSERT_TRUE(!(sw & 0x0008));  // no fault
}

void test_set_velocity_single(ZLAC8015Driver& driver) {
    std::cout << "(Setting velocity to " << config.test_rpm << " RPM)\n";
    ASSERT_TRUE(driver.set_velocity_rpm(config.test_rpm, config.test_rpm));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    const int16_t left = driver.get_velocity_left();
    const int16_t right = driver.get_velocity_right();
    std::cout << "(Feedback: L=" << left << " R=" << right << " RPM)\n";

    const double tol = std::abs(static_cast<double>(config.test_rpm) * config.rpm_tolerance);
    ASSERT_NEAR(left, config.test_rpm, std::max(tol, 5.0));
    ASSERT_NEAR(right, config.test_rpm, std::max(tol, 5.0));
}

void test_set_velocity_dual_different(ZLAC8015Driver& driver) {
    const int16_t left_rpm = 30, right_rpm = 60;
    std::cout << "(Setting velocity: L=" << left_rpm << " R=" << right_rpm << " RPM)\n";
    ASSERT_TRUE(driver.set_velocity_rpm(left_rpm, right_rpm));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    const int16_t actual_L = driver.get_velocity_left();
    const int16_t actual_R = driver.get_velocity_right();
    std::cout << "(Feedback: L=" << actual_L << " R=" << actual_R << " RPM)\n";

    ASSERT_NEAR(actual_L, left_rpm, 5.0);
    ASSERT_NEAR(actual_R, right_rpm, 5.0);
}

void test_stop_motor(ZLAC8015Driver& driver) {
    std::cout << "(Stopping motor)\n";
    ASSERT_TRUE(driver.set_velocity_rpm(0, 0));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const int16_t left = driver.get_velocity_left();
    const int16_t right = driver.get_velocity_right();
    std::cout << "(Feedback: L=" << left << " R=" << right << " RPM)\n";
    ASSERT_TRUE(std::abs(left) < 5 && std::abs(right) < 5);
}

void test_kinematics_forward(ZLAC8015Driver& driver, DifferentialDriveKinematics& kin) {
    std::cout << "(Testing forward motion: v=0.5 m/s)\n";
    auto rpm = kin.velocity_to_rpm(0.5, 0.0);
    std::cout << "(Calculated RPM: L=" << rpm.left << " R=" << rpm.right << ")\n";
    ASSERT_TRUE(rpm.left == rpm.right);
    ASSERT_TRUE(driver.set_velocity_rpm(rpm.left, rpm.right));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void test_kinematics_point_turn(ZLAC8015Driver& driver, DifferentialDriveKinematics& kin) {
    std::cout << "(Testing point turn: omega=1.57 rad/s)\n";
    auto rpm = kin.velocity_to_rpm(0.0, 1.57);
    std::cout << "(Calculated RPM: L=" << rpm.left << " R=" << rpm.right << ")\n";
    ASSERT_TRUE(rpm.left == -rpm.right);
    ASSERT_TRUE(driver.set_velocity_rpm(rpm.left, rpm.right));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(driver.set_velocity_rpm(0, 0));
}

void test_kinematics_arc(ZLAC8015Driver& driver, DifferentialDriveKinematics& kin) {
    std::cout << "(Testing arc: v=0.3 m/s, omega=0.5 rad/s)\n";
    auto rpm = kin.velocity_to_rpm(0.3, 0.5);
    std::cout << "(Calculated RPM: L=" << rpm.left << " R=" << rpm.right << ")\n";
    ASSERT_TRUE(rpm.right > rpm.left);
    ASSERT_TRUE(driver.set_velocity_rpm(rpm.left, rpm.right));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    ASSERT_TRUE(driver.set_velocity_rpm(0, 0));
}

void test_disable_motor(ZLAC8015Driver& driver) {
    std::cout << "(Disabling motor)\n";
    ASSERT_TRUE(driver.set_velocity_rpm(0, 0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(driver.disable());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint16_t sw = driver.read_status();
    std::cout << "(Status after disable: 0x" << std::hex << sw << std::dec << ")\n";
    ASSERT_TRUE(!(sw & 0x0004));  // operation_enabled = false
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "\n";
    std::cout << "=====================================================\n";
    std::cout << "        ZLAC8015D MOTOR CONTROL TEST SUITE          \n";
    std::cout << "=====================================================\n";
    std::cout << "  Uses ZLAC8015Driver + Kinematics from library\n";
    std::cout << "  Usage:\n";
    std::cout << "    ./test_motor_control --config config/zlac8015_config.yaml\n";
    std::cout << "    ./test_motor_control can0 1\n";
    std::cout << "=====================================================\n";

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
            std::cout << "  can0 [node_id]        Quick start\n";
            std::cout << "  --help, -h            Show this help\n";
            return 0;
        } else {
            config.loadFromArgs(argc, argv);
        }
    }

    if (!config_file.empty()) {
        config.loadFromFile(config_file);
    }
    config.print();

    // Create kinematics from library
    DifferentialDriveKinematics kinematics(config.wheel_radius, config.wheelbase,
                                           config.max_rpm);

    // Create CAN bus + driver from library
    SocketCanBus bus(config.can_interface);
    ZLAC8015Driver driver(config.motor_node_id, &bus);
    driver.set_sdo_timeout(config.sdo_timeout_ms);
    driver.set_wheel_parameters(config.wheel_radius, config.wheelbase);

    std::cout << "Starting Motor Control Tests...\n";
    std::cout << "=====================================================\n";

    RUN_TEST(test_can_connection, driver, bus);
    RUN_TEST(test_read_initial_status, driver);
    RUN_TEST(test_enable_motor, driver);
    RUN_TEST(test_set_velocity_single, driver);
    RUN_TEST(test_set_velocity_dual_different, driver);
    RUN_TEST(test_stop_motor, driver);
    RUN_TEST(test_kinematics_forward, driver, kinematics);
    RUN_TEST(test_kinematics_point_turn, driver, kinematics);
    RUN_TEST(test_kinematics_arc, driver, kinematics);
    RUN_TEST(test_disable_motor, driver);

    bus.close();

    std::cout << "\n=====================================================\n";
    std::cout << "Test Summary\n";
    std::cout << "=====================================================\n";
    std::cout << "Total: " << (tests_passed + tests_failed) << " | ";
    std::cout << "Passed: " << tests_passed << " | ";
    std::cout << "Failed: " << tests_failed << "\n";
    std::cout << "=====================================================\n";

    return tests_failed > 0 ? 1 : 0;
}
