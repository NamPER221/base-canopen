/**
 * @file test_mbdv_driver.cpp
 * @brief Unit tests for Moons' MBDV Dual-Axis Servo Driver over FakeBus
 */

#include <canopen/drivers/mbdv/mbdv_driver.hpp>
#include <canopen/mock/fake_bus.hpp>

#include <atomic>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

using namespace canopen;
using namespace canopen::drivers;
using namespace canopen::test;

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

namespace {

/**
 * @brief Simulated Moons' MBDV Single-Axis Node (CiA 402 responder over FakeBus)
 */
class VirtualMbdvAxis {
public:
    explicit VirtualMbdvAxis(uint8_t node_id) : node_id_(node_id) {}

    void attach(FakeBus& bus) {
        bus_ = &bus;
        // Listen to SDO requests (0x600 + node_id)
        route_sdo_ = bus.add_route(0x600u + node_id_, 0x7FF, [this](const CANFrame& f) {
            handle_sdo_request(f);
        });

        // Listen to RPDO3 (0x400 + node_id)
        route_rpdo3_ = bus.add_route(0x400u + node_id_, 0x7FF, [this](const CANFrame& f) {
            handle_rpdo3(f);
        });

        // Listen to NMT (0x000)
        route_nmt_ = bus.add_route(0x000, 0x7FF, [this](const CANFrame& f) {
            if (f.len() >= 2) {
                uint8_t cmd = f.get_u8(0);
                uint8_t target_node = f.get_u8(1);
                if (target_node == 0 || target_node == node_id_) {
                    if (cmd == 0x01) { // START
                        send_heartbeat(0x05); // Operational
                    }
                }
            }
        });
    }

    void detach() {
        if (bus_) {
            if (route_sdo_) bus_->remove_route(route_sdo_);
            if (route_rpdo3_) bus_->remove_route(route_rpdo3_);
            if (route_nmt_) bus_->remove_route(route_nmt_);
        }
        bus_ = nullptr;
    }

    void send_heartbeat(uint8_t nmt_state = 0x05) { // 0x05 = Operational
        if (!bus_) return;
        CANFrame hb;
        hb.set_id(0x700u + node_id_);
        hb.set_len(1);
        hb.set_u8(0, nmt_state);
        bus_->dispatch(hb);
    }

    void send_tpdo1(uint16_t statusword) {
        if (!bus_) return;
        statusword_ = statusword;
        CANFrame f;
        f.set_id(0x180u + node_id_);
        f.set_len(2);
        f.set_u16_le(0, statusword);
        bus_->dispatch(f);
    }

    void send_tpdo2(int32_t position) {
        if (!bus_) return;
        CANFrame f;
        f.set_id(0x280u + node_id_);
        f.set_len(4);
        f.set_u32_le(0, static_cast<uint32_t>(position));
        bus_->dispatch(f);
    }

    uint16_t last_controlword() const { return last_controlword_; }
    int32_t last_target_velocity() const { return last_target_velocity_; }
    int8_t last_mode() const { return last_mode_; }
    uint32_t rpdo3_count() const { return rpdo3_count_; }

private:
    void handle_rpdo3(const CANFrame& f) {
        if (f.len() >= 6) {
            last_controlword_ = f.get_u16_le(0);
            last_target_velocity_ = static_cast<int32_t>(f.get_u32_le(2));
            rpdo3_count_++;

            // Auto-update statusword from controlword
            if (last_controlword_ == 0x0006) {
                send_tpdo1(0x0021); // Ready to switch on
            } else if (last_controlword_ == 0x0007) {
                send_tpdo1(0x0023); // Switched on
            } else if (last_controlword_ == 0x000F || last_controlword_ == 0x010F) {
                send_tpdo1(0x0027); // Operation enabled
            }
        }
    }

    void handle_sdo_request(const CANFrame& f) {
        if (f.len() < 4) return;
        uint8_t cs = f.get_u8(0) >> 5;
        uint16_t index = f.get_u16_le(1);
        uint8_t subindex = f.get_u8(3);

        if (cs == 1) { // SDO Download (Write)
            if (index == 0x6060 && subindex == 0) {
                last_mode_ = static_cast<int8_t>(f.get_u8(4));
            } else if (index == 0x6040 && subindex == 0) {
                last_controlword_ = f.get_u16_le(4);
                if (last_controlword_ == 0x0006) send_tpdo1(0x0021);
                else if (last_controlword_ == 0x0007) send_tpdo1(0x0023);
                else if (last_controlword_ == 0x000F || last_controlword_ == 0x010F) send_tpdo1(0x0027);
            }

            // Send SDO Download response (ccs = 3 -> 0x60)
            CANFrame resp;
            resp.set_id(0x580u + node_id_);
            resp.set_len(8);
            resp.set_u8(0, 0x60);
            resp.set_u16_le(1, index);
            resp.set_u8(3, subindex);
            bus_->dispatch(resp);

        } else if (cs == 2) { // SDO Upload (Read)
            CANFrame resp;
            resp.set_id(0x580u + node_id_);
            resp.set_len(8);

            if (index == 0x6041) { // Statusword
                resp.set_u8(0, 0x4B); // 2 bytes
                resp.set_u16_le(1, index);
                resp.set_u8(3, subindex);
                resp.set_u16_le(4, statusword_);
            } else {
                resp.set_u8(0, 0x80); // Abort
                resp.set_u16_le(1, index);
                resp.set_u8(3, subindex);
            }
            bus_->dispatch(resp);
        }
    }

    uint8_t node_id_;
    FakeBus* bus_{nullptr};
    BusInterface::RouteHandle route_sdo_{0};
    BusInterface::RouteHandle route_rpdo3_{0};
    BusInterface::RouteHandle route_nmt_{0};

    uint16_t statusword_{0x0000};
    uint16_t last_controlword_{0x0000};
    int32_t last_target_velocity_{0};
    int8_t last_mode_{0};
    uint32_t rpdo3_count_{0};
};

} // anonymous namespace

// ============================================================================
// Test Suite
// ============================================================================

void test_mbdv_init_and_state_machine() {
    std::cout << "\n=== Running test_mbdv_init_and_state_machine ===" << std::endl;

    FakeBus bus;
    VirtualMbdvAxis v_axis1(1);
    VirtualMbdvAxis v_axis2(2);

    v_axis1.attach(bus);
    v_axis2.attach(bus);

    MbdvDriver driver(1, 2, &bus);

    // Provide heartbeats
    v_axis1.send_heartbeat(0x05);
    v_axis2.send_heartbeat(0x05);

    // Initial Statusword: Not ready
    v_axis1.send_tpdo1(0x0040); // Switch on disabled
    v_axis2.send_tpdo1(0x0040);

    bool init_ok = driver.init(100);
    CHECK(init_ok, "driver.init() succeeds with simulated online nodes");
    CHECK(v_axis1.last_mode() == 3, "Axis 1 configured to Profile Velocity mode (PV=3)");
    CHECK(v_axis2.last_mode() == 3, "Axis 2 configured to Profile Velocity mode (PV=3)");

    // Test CiA 402 Servo ON Sequence
    bool enable_ok = driver.enable(std::chrono::milliseconds(200));
    CHECK(enable_ok, "driver.enable() successfully enabled both axes");
    CHECK(driver.axis1().state() == CiA402State::OPERATION_ENABLED,
          "Axis 1 state is OPERATION_ENABLED (0x0027)");
    CHECK(driver.axis2().state() == CiA402State::OPERATION_ENABLED,
          "Axis 2 state is OPERATION_ENABLED (0x0027)");

    v_axis1.detach();
    v_axis2.detach();
}

void test_mbdv_inverse_kinematics() {
    std::cout << "\n=== Running test_mbdv_inverse_kinematics ===" << std::endl;

    FakeBus bus;
    VirtualMbdvAxis v_axis1(1);
    VirtualMbdvAxis v_axis2(2);

    v_axis1.attach(bus);
    v_axis2.attach(bus);

    MbdvDriver driver(1, 2, &bus);

    MbdvKinematicsConfig cfg;
    cfg.wheel_radius_m = 0.07333;
    cfg.wheelbase_m = 0.4544;
    cfg.encoder_cpr = 10000;
    cfg.gear_ratio = 1.0;
    cfg.invert_left = false;
    cfg.invert_right = false;
    driver.set_kinematics_config(cfg);

    // Test 1: Pure forward motion (v = 0.2 m/s, w = 0)
    // w = 0.2 / 0.07333 = 2.7274 rad/s
    // driver_vel = 2.7274 * 10000 / (2 * pi) = 4341 counts/s
    driver.set_cmd_vel(0.2, 0.0);

    CHECK(std::abs(v_axis1.last_target_velocity() - 4341) <= 2,
          "Axis 1 forward velocity target (~4341 counts/s)");
    CHECK(std::abs(v_axis2.last_target_velocity() - 4341) <= 2,
          "Axis 2 forward velocity target (~4341 counts/s)");

    // Test 2: Pure rotation in place (v = 0, w = 1.0 rad/s)
    // w_left  = -1.0 * (0.4544 / 2) / 0.07333 = -3.0983 rad/s -> -4931 counts/s
    // w_right = +1.0 * (0.4544 / 2) / 0.07333 = +3.0983 rad/s -> +4931 counts/s
    driver.set_cmd_vel(0.0, 1.0);

    CHECK(std::abs(v_axis1.last_target_velocity() - (-4931)) <= 2,
          "Axis 1 rotational velocity target (-4931 counts/s)");
    CHECK(std::abs(v_axis2.last_target_velocity() - (+4931)) <= 2,
          "Axis 2 rotational velocity target (+4931 counts/s)");

    v_axis1.detach();
    v_axis2.detach();
}

void test_mbdv_forward_kinematics_odometry() {
    std::cout << "\n=== Running test_mbdv_forward_kinematics_odometry ===" << std::endl;

    FakeBus bus;
    VirtualMbdvAxis v_axis1(1);
    VirtualMbdvAxis v_axis2(2);

    v_axis1.attach(bus);
    v_axis2.attach(bus);

    MbdvDriver driver(1, 2, &bus);

    MbdvKinematicsConfig cfg;
    cfg.wheel_radius_m = 0.07333;
    cfg.wheelbase_m = 0.4544;
    cfg.encoder_cpr = 10000;
    cfg.gear_ratio = 1.0;
    driver.set_kinematics_config(cfg);

    // Initial ticks at 0
    v_axis1.send_tpdo2(0);
    v_axis2.send_tpdo2(0);
    driver.update_odometry(0.1);

    auto p0 = driver.get_pose();
    CHECK(p0.x == 0.0 && p0.y == 0.0 && p0.theta == 0.0, "Initial pose is (0, 0, 0)");

    // Simulate 1 wheel revolution = 10000 counts on both wheels
    // Distance = 2 * pi * r = 2 * pi * 0.07333 = 0.46075 m
    v_axis1.send_tpdo2(10000);
    v_axis2.send_tpdo2(10000);
    driver.update_odometry(1.0);

    auto p1 = driver.get_pose();
    double expected_dist = 2.0 * M_PI * 0.07333;
    CHECK(std::abs(p1.x - expected_dist) < 0.005,
          "Odometry X position after 1 rev (~0.461 m)");
    CHECK(std::abs(p1.y) < 1e-4, "Odometry Y position is zero");
    CHECK(std::abs(p1.theta) < 1e-4, "Odometry Heading theta is zero");

    v_axis1.detach();
    v_axis2.detach();
}

void test_mbdv_quick_stop_and_health() {
    std::cout << "\n=== Running test_mbdv_quick_stop_and_health ===" << std::endl;

    FakeBus bus;
    VirtualMbdvAxis v_axis1(1);
    VirtualMbdvAxis v_axis2(2);

    v_axis1.attach(bus);
    v_axis2.attach(bus);

    MbdvDriver driver(1, 2, &bus);

    v_axis1.send_heartbeat(0x05);
    v_axis2.send_heartbeat(0x05);
    driver.init(100);

    CHECK(driver.check_health(), "Driver reports healthy when heartbeats active");

    driver.quick_stop();
    CHECK(v_axis1.last_controlword() == 0x0002, "Axis 1 received Quick Stop CW 0x0002");
    CHECK(v_axis2.last_controlword() == 0x0002, "Axis 2 received Quick Stop CW 0x0002");

    v_axis1.detach();
    v_axis2.detach();
}

int main() {
    std::cout << "Starting Moons' MBDV CANopen Driver Unit Tests..." << std::endl;

    test_mbdv_init_and_state_machine();
    test_mbdv_inverse_kinematics();
    test_mbdv_forward_kinematics_odometry();
    test_mbdv_quick_stop_and_health();

    std::cout << "\n=========================================" << std::endl;
    std::cout << "Test Summary: " << tests_passed << " / " << tests_total << " passed." << std::endl;
    std::cout << "=========================================" << std::endl;

    return (tests_passed == tests_total) ? 0 : 1;
}
