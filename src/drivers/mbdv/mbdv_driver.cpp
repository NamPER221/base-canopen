/**
 * @file mbdv_driver.cpp
 * @brief Moons' MBDV Dual-Axis CANopen Servo Driver Implementation
 */

#include <canopen/drivers/mbdv/mbdv_driver.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

namespace canopen {
namespace drivers {

namespace {

// CiA 402 Controlword commands
constexpr uint16_t CW_SHUTDOWN         = 0x0006;
constexpr uint16_t CW_SWITCH_ON        = 0x0007;
constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
constexpr uint16_t CW_DISABLE_VOLTAGE  = 0x0000;
constexpr uint16_t CW_QUICK_STOP       = 0x0002;
constexpr uint16_t CW_FAULT_RESET      = 0x0080;

// NMT commands
constexpr uint8_t NMT_START         = 0x01;
constexpr uint8_t NMT_STOP          = 0x02;
constexpr uint8_t NMT_ENTER_PREOP   = 0x80;
constexpr uint8_t NMT_RESET_NODE    = 0x81;
constexpr uint8_t NMT_RESET_COMM    = 0x82;

static void send_nmt(BusInterface* bus, uint8_t node, uint8_t cmd) {
    if (!bus) return;
    CANFrame frame;
    frame.set_id(0x000);
    frame.set_len(2);
    frame.set_u8(0, cmd);
    frame.set_u8(1, node);
    bus->send(frame);
}

static CiA402State decode_statusword(uint16_t sw) {
    if ((sw & 0x004F) == 0x0000) return CiA402State::NOT_READY_TO_SWITCH_ON;
    if ((sw & 0x004F) == 0x0040) return CiA402State::SWITCH_ON_DISABLED;
    if ((sw & 0x006F) == 0x0021) return CiA402State::READY_TO_SWITCH_ON;
    if ((sw & 0x006F) == 0x0023) return CiA402State::SWITCHED_ON;
    if ((sw & 0x006F) == 0x0027) return CiA402State::OPERATION_ENABLED;
    if ((sw & 0x006F) == 0x0007) return CiA402State::QUICK_STOP_ACTIVE;
    if ((sw & 0x004F) == 0x000F) return CiA402State::FAULT_REACTION_ACTIVE;
    if ((sw & 0x004F) == 0x0008) return CiA402State::FAULT;
    return CiA402State::NOT_READY_TO_SWITCH_ON;
}

} // anonymous namespace

// ============================================================================
// MbdvAxis Implementation
// ============================================================================

MbdvAxis::MbdvAxis(uint8_t node_id, std::string name)
    : node_id_(node_id), name_(std::move(name)) {
    drive_ = std::make_unique<CiA402Drive>(node_id, nullptr);
    drive_->sdo_encoding(SdoEncoding::Legacy);
}

MbdvAxis::~MbdvAxis() {
    remove_pdo_routes();
}

void MbdvAxis::set_bus(BusInterface* bus) {
    bus_ = bus;
    if (drive_) {
        drive_->set_bus(bus);
        drive_->sdo_encoding(SdoEncoding::Legacy);
    }
    if (bus_) {
        setup_pdo_routes(bus_);
    } else {
        remove_pdo_routes();
    }
}

void MbdvAxis::set_sdo_timeout(uint32_t ms) {
    if (drive_) drive_->set_sdo_timeout(ms);
}

void MbdvAxis::setup_pdo_routes(BusInterface* bus) {
    if (!bus) return;
    remove_pdo_routes();
    bus_ = bus;
    if (drive_) {
        drive_->set_bus(bus);
        drive_->sdo_encoding(SdoEncoding::Legacy);
    }

    // 1. TPDO1: Statusword (0x180 + node_id)
    auto r1 = bus_->add_route(0x180u + node_id_, 0x7FF, [this](const CANFrame& f) {
        handle_tpdo1(f);
    });
    if (r1) route_handles_.push_back(r1);

    // 2. TPDO2: Position actual value (0x280 + node_id)
    auto r2 = bus_->add_route(0x280u + node_id_, 0x7FF, [this](const CANFrame& f) {
        handle_tpdo2(f);
    });
    if (r2) route_handles_.push_back(r2);

    // 3. TPDO3: Velocity actual value (0x380 + node_id)
    auto r3 = bus_->add_route(0x380u + node_id_, 0x7FF, [this](const CANFrame& f) {
        handle_tpdo3(f);
    });
    if (r3) route_handles_.push_back(r3);

    // 4. TPDO4: Position + Velocity combined (0x480 + node_id)
    auto r4 = bus_->add_route(0x480u + node_id_, 0x7FF, [this](const CANFrame& f) {
        handle_tpdo4(f);
    });
    if (r4) route_handles_.push_back(r4);

    // 5. Heartbeat / Bootup (0x700 + node_id)
    auto r5 = bus_->add_route(0x700u + node_id_, 0x7FF, [this](const CANFrame& f) {
        handle_heartbeat(f);
    });
    if (r5) route_handles_.push_back(r5);

    // 6. Emergency (0x080 + node_id)
    auto r6 = bus_->add_route(0x080u + node_id_, 0x7FF, [this](const CANFrame& f) {
        handle_emcy(f);
    });
    if (r6) route_handles_.push_back(r6);
}

void MbdvAxis::remove_pdo_routes() {
    if (!bus_) return;
    for (auto h : route_handles_) {
        bus_->remove_route(h);
    }
    route_handles_.clear();
}

void MbdvAxis::update_from_drive_poll() {
    if (!drive_) return;
    if (drive_->poll_status() == SDOError::OK) {
        uint16_t sw = drive_->get_statusword();
        statusword_.store(sw);
        auto new_state = decode_statusword(sw);
        auto old_state = state_.exchange(new_state);
        if (old_state != new_state && on_state_change) {
            on_state_change(old_state, new_state);
        }
    }
}

bool MbdvAxis::poll_feedback_sdo() {
    if (!drive_) return false;
    int32_t pos = 0;
    int32_t vel = 0;
    bool ok1 = drive_->sdo_read_i32(mbdv_od::POSITION_ACTUAL, 0x00, pos);
    bool ok2 = drive_->sdo_read_i32(mbdv_od::VELOCITY_ACTUAL, 0x00, vel);
    if (ok1) actual_position_.store(pos);
    if (ok2) actual_velocity_.store(vel);
    return ok1 && ok2;
}

bool MbdvAxis::init(uint32_t timeout_ms) {
    if (!bus_ || !bus_->is_up()) {
        std::cerr << "[" << name_ << "] Error: CAN bus is not up!" << std::endl;
        return false;
    }

    setup_pdo_routes(bus_);

    // Ensure node is in PRE-OPERATIONAL for PDO/SDO configuration (CiA 301)
    send_nmt(bus_, node_id_, NMT_ENTER_PREOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Poll status via SDO immediately
    update_from_drive_poll();

    // Clear any existing fault on initialization (Alarm 40/11 from previous run)
    if (state_.load() == CiA402State::FAULT || (statusword_.load() & 0x0008) != 0) {
        std::cout << "[" << name_ << "] Drive in FAULT at init, resetting..." << std::endl;
        fault_reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // All PDO/SDO configuration must happen in PRE-OPERATIONAL
    if (drive_) {
        // Disable communication watchdog (P1-39 enable flag, object 0x2060:01 = 0)
        drive_->sdo_write_u16(0x2060, 0x01, 0);

        // Enable RPDO3: clear bit 31 of COB-ID (0x1402:01 = 0x400 + node_id)
        // Must be done in PRE-OP; drives may reject COB-ID changes in OPERATIONAL
        drive_->sdo_write_u32(0x1402, 0x01, 0x400u + node_id_);
        // Ensure RPDO3 transmission type is 0xFF (asynchronous/event-driven)
        drive_->sdo_write_u8(0x1402, 0x02, 0xFF);

        // TPDO1: Statusword (0x1800) @ 10ms (100 Hz)
        drive_->sdo_write_u16(mbdv_od::TPDO1_COMM_PARAM, 0x05, 10);
        // TPDO2: Position + Velocity combined (0x1801) @ 10ms (100 Hz)
        drive_->sdo_write_u16(mbdv_od::TPDO2_COMM_PARAM, 0x05, 10);
        // TPDO3: Error code + DSP Alarm (0x1802) @ 100ms
        drive_->sdo_write_u16(mbdv_od::TPDO3_COMM_PARAM, 0x05, 100);
    }

    // Configure Profile Velocity mode (PV = 3)
    set_operation_mode(OperationMode::PROFILED_VELOCITY);

    // Set default acceleration and deceleration (counts/s^2)
    set_profile(50000, 100000);

    // Transition to OPERATIONAL: enables PDO communication
    send_nmt(bus_, node_id_, NMT_START);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Wait for operational state or valid statusword
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
        if (operational_.load() || statusword_.load() != 0) {
            online_.store(true);
            return true;
        }
        update_from_drive_poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (statusword_.load() != 0) {
        online_.store(true);
        return true;
    }

    std::cerr << "[" << name_ << "] Init warning: No heartbeat/PDO received within "
              << timeout_ms << "ms (Node " << static_cast<int>(node_id_) << ")" << std::endl;
    return false;
}

bool MbdvAxis::enable(std::chrono::milliseconds timeout) {
    if (!bus_ || !bus_->is_up()) return false;

    // Check if drive is currently in Fault
    update_from_drive_poll();
    if (state_.load() == CiA402State::FAULT || (statusword_.load() & 0x0008) != 0) {
        std::cout << "[" << name_ << "] Drive in FAULT, triggering fault reset..." << std::endl;
        fault_reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        update_from_drive_poll();
    }

    auto start_time = std::chrono::steady_clock::now();

    // Step 1: Send Shutdown (0x0006) -> Wait for READY_TO_SWITCH_ON
    if (drive_) drive_->set_controlword(CW_SHUTDOWN);
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        if (state_.load() == CiA402State::READY_TO_SWITCH_ON ||
            state_.load() == CiA402State::SWITCHED_ON ||
            state_.load() == CiA402State::OPERATION_ENABLED) {
            break;
        }
        update_from_drive_poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Step 2: Send Switch On (0x0007) -> Wait for SWITCHED_ON
    if (drive_) drive_->set_controlword(CW_SWITCH_ON);
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        if (state_.load() == CiA402State::SWITCHED_ON ||
            state_.load() == CiA402State::OPERATION_ENABLED) {
            break;
        }
        update_from_drive_poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Re-assert Profile Velocity mode before enabling operation
    set_operation_mode(OperationMode::PROFILED_VELOCITY);

    // Step 3: Send Enable Operation (0x000F)
    auto step3_start = std::chrono::steady_clock::now();
    if (drive_) drive_->set_controlword(CW_ENABLE_OPERATION);
    send_rpdo3_velocity(0, CW_ENABLE_OPERATION);

    while (std::chrono::steady_clock::now() - step3_start < timeout) {
        if (state_.load() == CiA402State::OPERATION_ENABLED) {
            std::cout << "[" << name_ << "] Servo ON successful (Operation Enabled)!" << std::endl;
            return true;
        }
        update_from_drive_poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cerr << "[" << name_ << "] Enable timeout! State: "
              << static_cast<int>(state_.load())
              << " SW: 0x" << std::hex << statusword_.load() << std::dec << std::endl;
    return false;
}

bool MbdvAxis::disable() {
    if (drive_) {
        drive_->set_controlword(CW_SWITCH_ON); // Disable operation -> Switched on
    }
    return true;
}

void MbdvAxis::quick_stop() {
    if (drive_) {
        drive_->quick_stop();
    }
    // Also send immediate RPDO with Quick Stop CW (0x0002), DLC = 6
    if (bus_) {
        CANFrame frame;
        frame.set_id(0x400u + node_id_);
        frame.set_len(6);
        frame.set_u16_le(0, CW_QUICK_STOP);
        frame.set_u32_le(2, 0);
        bus_->send(frame);
    }
}

bool MbdvAxis::fault_reset() {
    if (drive_) {
        // Moons-specific: clear DSP-level alarm (0x2006 = 1)
        drive_->sdo_write_u8(0x2006, 0x00, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        // CiA 402: Fault Reset via controlword bit 7
        drive_->set_controlword(CW_FAULT_RESET);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        drive_->set_controlword(CW_DISABLE_VOLTAGE);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        update_from_drive_poll();
    }
    return true;
}

bool MbdvAxis::set_operation_mode(OperationMode mode) {
    if (drive_) {
        // Moons' MBDV vendor requirement: P1-00 (0x2A30) must match CiA 402 mode:
        // PP (1) -> P1-00 = 21, PV (3) -> P1-00 = 15, PT (4) -> P1-00 = 1
        uint32_t p1_00 = 21;
        if (mode == OperationMode::PROFILED_VELOCITY) {
            p1_00 = 15;
        } else if (mode == OperationMode::PROFILED_TORQUE) {
            p1_00 = 1;
        }
        uint32_t current_p1_00 = 0;
        if (!drive_->sdo_read_u32(0x2A30, 0x00, current_p1_00) || current_p1_00 != p1_00) {
            for (int r = 0; r < 3; ++r) {
                drive_->sdo_write_u32(0x2A30, 0x00, p1_00);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                if (drive_->sdo_read_u32(0x2A30, 0x00, current_p1_00) && current_p1_00 == p1_00) {
                    break;
                }
            }
        }
        std::cout << "[" << name_ << "] P1-00 (0x2A30) is: " << current_p1_00 << std::endl;

        bool ok = drive_->set_operation_mode(mode);
        if (!ok) return false;

        // Verify via 0x6061 with retry (CiA 402 requirement)
        for (int i = 0; i < 10; ++i) {
            uint8_t disp_mode = 0;
            if (drive_->sdo_read_u8(mbdv_od::MODES_OF_OPERATION_DISP, 0x00, disp_mode)) {
                if (disp_mode == static_cast<uint8_t>(mode)) {
                    std::cout << "[" << name_ << "] Mode of operation 0x6061 confirmed: "
                              << static_cast<int>(disp_mode) << std::endl;
                    return true;
                }
            } else {
                break; // SDO read failed (mock or communication lost)
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
        return ok;
    }
    return false;
}

bool MbdvAxis::set_profile(uint32_t accel_counts_s2, uint32_t decel_counts_s2) {
    if (drive_) {
        bool ok1 = drive_->sdo_write_u32(mbdv_od::PROFILE_ACCELERATION, 0x00, accel_counts_s2);
        bool ok2 = drive_->sdo_write_u32(mbdv_od::PROFILE_DECELERATION, 0x00, decel_counts_s2);
        return ok1 && ok2;
    }
    return false;
}

void MbdvAxis::set_target_velocity_counts(int32_t counts_per_sec) {
    // In CiA 402 Profile Velocity Mode, use CW_ENABLE_OPERATION (0x000F).
    // Velocity setpoint 0 naturally stops the drive at deceleration rate.
    send_rpdo3_velocity(counts_per_sec, CW_ENABLE_OPERATION);
    if (drive_ && counts_per_sec != last_target_velocity_) {
        last_target_velocity_ = counts_per_sec;
        set_target_velocity_sdo(counts_per_sec);
    }
}

bool MbdvAxis::set_target_velocity_sdo(int32_t counts_per_sec) {
    if (drive_) {
        return drive_->sdo_write_i32(mbdv_od::TARGET_VELOCITY, 0x00, counts_per_sec);
    }
    return false;
}

void MbdvAxis::set_target_position_counts(int32_t counts, bool new_setpoint,
                                          bool immediate, bool relative) {
    uint16_t cw = CW_ENABLE_OPERATION;
    if (new_setpoint) cw |= 0x0010;  // Bit 4: New setpoint
    if (immediate)    cw |= 0x0020;  // Bit 5: Change set immediately
    if (relative)     cw |= 0x0040;  // Bit 6: Relative move

    send_rpdo2_position(counts, cw);

    if (new_setpoint) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        send_rpdo2_position(counts, CW_ENABLE_OPERATION);
    }
}

bool MbdvAxis::send_rpdo3_velocity(int32_t counts_per_sec, uint16_t controlword) {
    if (!bus_ || !bus_->is_up()) return false;

    // RPDO3: 0x400 + node_id
    // Bytes 0-1: Controlword (16-bit LE)
    // Bytes 2-5: Target velocity (int32_t LE)
    // Total DLC: 6 bytes (EDS 0x1602 mapping: 0x60400010 + 0x60FF0020 = 48 bits)
    CANFrame frame;
    frame.set_id(0x400u + node_id_);
    frame.set_len(6);
    frame.set_u16_le(0, controlword);
    frame.set_u32_le(2, static_cast<uint32_t>(counts_per_sec));

    return bus_->send(frame);
}

bool MbdvAxis::send_rpdo2_position(int32_t counts, uint16_t controlword) {
    if (!bus_ || !bus_->is_up()) return false;

    // RPDO2: 0x300 + node_id
    // Bytes 0-1: Controlword (16-bit LE)
    // Bytes 2-5: Target position (int32_t LE)
    // Total DLC: 6 bytes (Moons MBDV EDS object 0x1601 mapping length: 16-bit CW + 32-bit Pos)
    CANFrame frame;
    frame.set_id(0x300u + node_id_);
    frame.set_len(6);
    frame.set_u16_le(0, controlword);
    frame.set_u32_le(2, static_cast<uint32_t>(counts));

    return bus_->send(frame);
}

void MbdvAxis::handle_tpdo1(const CANFrame& frame) {
    if (frame.len() >= 2) {
        uint16_t sw = frame.get_u16_le(0);
        statusword_.store(sw);
        auto new_state = decode_statusword(sw);
        auto old_state = state_.exchange(new_state);
        if (old_state != new_state && on_state_change) {
            on_state_change(old_state, new_state);
        }
        tpdo_count_++;
    }
}

void MbdvAxis::handle_tpdo2(const CANFrame& frame) {
    // Moons' MBDV TPDO2: 0x280 + node contains Position actual (0x6064) + Velocity actual (0x606C)
    if (frame.len() >= 4) {
        int32_t pos = static_cast<int32_t>(frame.get_u32_le(0));
        actual_position_.store(pos);
    }
    if (frame.len() >= 8) {
        int32_t vel = static_cast<int32_t>(frame.get_u32_le(4));
        actual_velocity_.store(vel);
    }
    tpdo_count_++;
}

void MbdvAxis::handle_tpdo3(const CANFrame& frame) {
    // Moons' MBDV TPDO3: 0x380 + node contains Error code (0x603F) + DSP alarm code (0x200F)
    if (frame.len() >= 6) {
        uint32_t alarm = frame.get_u32_le(2);
        dsp_alarm_.store(static_cast<uint16_t>(alarm & 0xFFFF));
    }
    tpdo_count_++;
}

void MbdvAxis::handle_tpdo4(const CANFrame& frame) {
    if (frame.len() >= 8) {
        int32_t pos = static_cast<int32_t>(frame.get_u32_le(0));
        int32_t vel = static_cast<int32_t>(frame.get_u32_le(4));
        actual_position_.store(pos);
        actual_velocity_.store(vel);
        tpdo_count_++;
    }
}

void MbdvAxis::handle_heartbeat(const CANFrame& frame) {
    online_.store(true);
    auto now_rep = std::chrono::steady_clock::now().time_since_epoch().count();
    last_heartbeat_time_.store(now_rep);

    if (frame.len() >= 1) {
        uint8_t nmt_st = frame.get_u8(0);
        operational_.store(nmt_st == 0x05); // 0x05 = Operational
    }
}

void MbdvAxis::handle_emcy(const CANFrame& frame) {
    if (frame.len() >= 2) {
        uint16_t err = frame.get_u16_le(0);
        std::cerr << "[" << name_ << "] CANopen EMCY Frame! ErrorCode: 0x"
                  << std::hex << err << std::dec << std::endl;
        if (on_emcy) on_emcy(err);
    }
}

bool MbdvAxis::poll_diagnostics_sdo() {
    if (!drive_) return false;

    // Read DC Bus Voltage (0x2030, unit: 0.1V)
    uint16_t v_raw = 0;
    if (drive_->sdo_read_u16(mbdv_od::DC_BUS_VOLTAGE, 0x00, v_raw)) {
        voltage_.store(v_raw * 0.1);
    }

    // Read Temperature (0x2019:01, unit: °C)
    int16_t temp_raw = 0;
    if (drive_->sdo_read_i16(mbdv_od::DEVICE_TEMPERATURE, 0x01, temp_raw)) {
        temperature_.store(temp_raw);
    }

    // Read DSP Alarm Code (0x200F)
    uint16_t alarm = 0;
    if (drive_->sdo_read_u16(mbdv_od::DSP_ALARM_CODE, 0x00, alarm)) {
        dsp_alarm_.store(alarm);
    }

    return true;
}

bool MbdvAxis::check_health(uint32_t heartbeat_timeout_ms) {
    auto last_rep = last_heartbeat_time_.load();
    if (last_rep == 0) return online_.load();

    auto last_tp = std::chrono::steady_clock::time_point(
        std::chrono::steady_clock::duration(last_rep));
    auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_tp).count();

    if (diff_ms > static_cast<int64_t>(heartbeat_timeout_ms)) {
        online_.store(false);
        operational_.store(false);
        return false;
    }
    return true;
}

MbdvAxisTelemetry MbdvAxis::get_telemetry() const {
    MbdvAxisTelemetry t;
    t.node_id = node_id_;
    t.online = online_.load();
    t.operational = operational_.load();
    t.state = state_.load();
    t.statusword = statusword_.load();
    t.actual_position_counts = actual_position_.load();
    t.actual_velocity_counts_s = actual_velocity_.load();
    // Default 10000 CPR -> RPM = counts_s * 60 / 10000
    t.actual_velocity_rpm = (static_cast<double>(t.actual_velocity_counts_s) * 60.0) / 10000.0;
    t.voltage_v = voltage_.load();
    t.temperature_c = temperature_.load();
    t.dsp_alarm_code = dsp_alarm_.load();
    t.tpdo_count = tpdo_count_.load();

    auto last_rep = last_heartbeat_time_.load();
    if (last_rep > 0) {
        auto last_tp = std::chrono::steady_clock::time_point(
            std::chrono::steady_clock::duration(last_rep));
        t.ms_since_heartbeat = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_tp).count();
    } else {
        t.ms_since_heartbeat = -1;
    }
    return t;
}

// ============================================================================
// MbdvDriver Implementation (Dual-Axis with Kinematics)
// ============================================================================

MbdvDriver::MbdvDriver(uint8_t node_id_1, uint8_t node_id_2, BusInterface* bus)
    : node_id_1_(node_id_1),
      node_id_2_(node_id_2),
      bus_(bus),
      axis1_(node_id_1, "Axis 1 (Left)"),
      axis2_(node_id_2, "Axis 2 (Right)") {
    if (bus_) set_bus(bus_);
}

MbdvDriver::~MbdvDriver() {
    if (bus_ && sync_enabled_) {
        sync_service_.stop_producer();
        sync_service_.detach(*bus_);
    }
}

void MbdvDriver::set_bus(BusInterface* bus) {
    if (bus_ && sync_enabled_) {
        sync_service_.stop_producer();
        sync_service_.detach(*bus_);
    }
    bus_ = bus;
    axis1_.set_bus(bus);
    axis2_.set_bus(bus);
    if (bus_ && sync_enabled_) {
        sync_service_.attach(*bus_);
    }
}

void MbdvDriver::set_sdo_timeout(uint32_t ms) {
    axis1_.set_sdo_timeout(ms);
    axis2_.set_sdo_timeout(ms);
}

void MbdvDriver::set_kinematics_config(const MbdvKinematicsConfig& config) {
    config_ = config;
}

bool MbdvDriver::init(uint32_t timeout_ms) {
    if (bus_ && sync_enabled_) {
        sync_service_.attach(*bus_);
        sync_service_.start_producer(10000); // 10 ms (100 Hz SYNC generator)
    }
    bool ok1 = axis1_.init(timeout_ms);
    bool ok2 = true;
    if (!single_axis_mode_) {
        ok2 = axis2_.init(timeout_ms);
    }
    return ok1 && ok2;
}

bool MbdvDriver::enable(std::chrono::milliseconds timeout) {
    bool ok1 = axis1_.enable(timeout);
    bool ok2 = true;
    if (!single_axis_mode_) {
        ok2 = axis2_.enable(timeout);
    }
    if (ok1 && ok2) {
        // Wait for electromagnetic brake release delay (P5-24 is 200 ms) before allowing motion commands
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    return ok1 && ok2;
}

bool MbdvDriver::disable() {
    bool ok1 = axis1_.disable();
    bool ok2 = true;
    if (!single_axis_mode_) {
        ok2 = axis2_.disable();
    }
    if (bus_ && sync_enabled_) {
        sync_service_.stop_producer();
    }
    return ok1 && ok2;
}

void MbdvDriver::quick_stop() {
    axis1_.quick_stop();
    if (!single_axis_mode_) {
        axis2_.quick_stop();
    }
}

bool MbdvDriver::fault_reset() {
    bool ok1 = axis1_.fault_reset();
    bool ok2 = true;
    if (!single_axis_mode_) {
        ok2 = axis2_.fault_reset();
    }
    return ok1 && ok2;
}

void MbdvDriver::set_cmd_vel(double linear_v, double angular_w) {
    // 1. Velocity Clamping
    double v_clamped = std::max(-config_.max_linear_velocity_m_s,
                                std::min(config_.max_linear_velocity_m_s, linear_v));
    double w_clamped = std::max(-config_.max_angular_velocity_rad_s,
                                std::min(config_.max_angular_velocity_rad_s, angular_w));

    // 2. Inverse Kinematics for Differential Drive:
    // w_left  = (v - w * L / 2) / r
    // w_right = (v + w * L / 2) / r
    double half_base = config_.wheelbase_m / 2.0;
    double w_left_rad = (v_clamped - (w_clamped * half_base)) / config_.wheel_radius_m;
    double w_right_rad = (v_clamped + (w_clamped * half_base)) / config_.wheel_radius_m;

    // 3. Convert angular velocity (rad/s) to driver pulse frequency (counts/s):
    // driver_vel = (omega * CPR * N) / (2 * pi)
    double counts_per_rad =
        (static_cast<double>(config_.encoder_cpr) * config_.gear_ratio) / (2.0 * M_PI);

    int32_t left_driver_vel = static_cast<int32_t>(std::round(w_left_rad * counts_per_rad));
    int32_t right_driver_vel = static_cast<int32_t>(std::round(w_right_rad * counts_per_rad));

    set_wheel_velocities(left_driver_vel, right_driver_vel);
}

void MbdvDriver::set_wheel_velocities(int32_t left_counts_s, int32_t right_counts_s) {
    if (config_.invert_left)  left_counts_s = -left_counts_s;
    if (config_.invert_right) right_counts_s = -right_counts_s;

    axis1_.set_target_velocity_counts(left_counts_s);
    if (!single_axis_mode_) {
        axis2_.set_target_velocity_counts(right_counts_s);
    }
}

void MbdvDriver::update_odometry(double dt_sec) {
    std::lock_guard<std::mutex> lock(odom_mutex_);

    // SDO fallback if TPDO has not produced position feedback yet
    if (axis1_.get_telemetry().tpdo_count == 0) {
        axis1_.poll_feedback_sdo();
    }
    if (!single_axis_mode_ && axis2_.get_telemetry().tpdo_count == 0) {
        axis2_.poll_feedback_sdo();
    }

    int32_t raw_left_ticks = axis1_.actual_position();
    int32_t raw_right_ticks = single_axis_mode_ ? 0 : axis2_.actual_position();

    int32_t left_ticks = config_.invert_left ? -raw_left_ticks : raw_left_ticks;
    int32_t right_ticks = config_.invert_right ? -raw_right_ticks : raw_right_ticks;

    if (first_odom_update_) {
        prev_ticks_left_ = left_ticks;
        prev_ticks_right_ = right_ticks;
        first_odom_update_ = false;
        return;
    }

    int32_t delta_left = left_ticks - prev_ticks_left_;
    int32_t delta_right = right_ticks - prev_ticks_right_;

    prev_ticks_left_ = left_ticks;
    prev_ticks_right_ = right_ticks;

    // Displacement per tick
    double meters_per_count = (2.0 * M_PI * config_.wheel_radius_m) /
                              (static_cast<double>(config_.encoder_cpr) * config_.gear_ratio);

    double d_left = delta_left * meters_per_count;
    double d_right = delta_right * meters_per_count;

    // Forward Kinematics
    double d_s = (d_right + d_left) / 2.0;
    double d_theta = (d_right - d_left) / config_.wheelbase_m;

    // Runge-Kutta 2nd order (midpoint) pose integration
    double mid_theta = pose_.theta + (d_theta / 2.0);
    pose_.x += d_s * std::cos(mid_theta);
    pose_.y += d_s * std::sin(mid_theta);
    pose_.theta = normalize_angle(pose_.theta + d_theta);

    if (dt_sec > 1e-4) {
        current_twist_.linear_v = d_s / dt_sec;
        current_twist_.angular_w = d_theta / dt_sec;
    }
}

void MbdvDriver::reset_odometry(double x, double y, double theta) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    pose_.x = x;
    pose_.y = y;
    pose_.theta = normalize_angle(theta);
    first_odom_update_ = true;
}

double MbdvDriver::normalize_angle(double angle) const {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

MbdvPose MbdvDriver::get_pose() const {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return pose_;
}

MbdvTwist MbdvDriver::get_twist() const {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return current_twist_;
}

MbdvTelemetry MbdvDriver::get_telemetry() const {
    MbdvTelemetry t;
    t.axis1 = axis1_.get_telemetry();
    t.axis2 = axis2_.get_telemetry();
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        t.pose = pose_;
        t.twist = current_twist_;
    }
    t.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return t;
}

bool MbdvDriver::check_health() {
    bool ok1 = axis1_.check_health();
    bool ok2 = single_axis_mode_ ? true : axis2_.check_health();
    return ok1 && ok2;
}

} // namespace drivers
} // namespace canopen
