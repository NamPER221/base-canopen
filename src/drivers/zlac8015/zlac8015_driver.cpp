/**
 * @file zlac8015_driver.cpp
 * @brief ZLAC8015D Dual-Axis Servo Driver implementation
 */

#include <canopen/drivers/zlac8015/zlac8015_driver.hpp>
#include <cmath>

namespace canopen {
namespace drivers {

namespace {
    constexpr uint16_t CW_SHUTDOWN        = 0x0006;
    constexpr uint16_t CW_SWITCH_ON       = 0x0007;
    constexpr uint16_t CW_ENABLE_OPER     = 0x000F;
    constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;
    constexpr uint16_t CW_UNLOCK_RAMP     = 0x007F;  // unlock ramp generator

    // Statusword bits
    constexpr uint16_t SW_OPERATION_ENABLED = 0x0004;
    constexpr uint16_t SW_FAULT             = 0x0008;
    constexpr uint16_t SW_SWITCH_ON_DISABLED = 0x0040;

    // NMT commands
    constexpr uint8_t NMT_START         = 0x01;
    constexpr uint8_t NMT_STOP          = 0x02;
    constexpr uint8_t NMT_PRE_OPER      = 0x80;
    constexpr uint8_t NMT_RESET_COMM    = 0x82;
} // anonymous namespace

// ============================================================================
// NMT helper
// ============================================================================

static void nmt_command(BusInterface* bus, uint8_t node, uint8_t cmd) {
    if (!bus) return;
    CANFrame frame;
    frame.set_id(0x000);
    frame.set_len(2);
    frame.set_u8(0, cmd);
    frame.set_u8(1, node);
    bus->send(frame);
}

// Log helpers
static std::string hex4(uint16_t v) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%04X", v);
    return buf;
}

static std::string sdo_error_str(SDOError e) {
    switch (e) {
        case SDOError::OK: return "OK";
        case SDOError::TIMEOUT: return "TIMEOUT";
        case SDOError::ABORT: return "ABORT";
        case SDOError::BUS_ERROR: return "BUS_ERROR";
        case SDOError::INVALID_RESPONSE: return "INVALID_RESPONSE";
        case SDOError::NO_BUS: return "NO_BUS";
        default: return "UNKNOWN";
    }
}

ZLAC8015Driver::ZLAC8015Driver(uint8_t node_id, BusInterface* bus)
    : node_id_(node_id), bus_(bus) {
    drive_ = std::make_unique<CiA402Drive>(node_id, bus);
    drive_->on_state_change = [this](CiA402State old_s, CiA402State new_s) {
        if (on_state_change) on_state_change(old_s, new_s);
    };
    drive_->on_motion_timeout = [this]() {
        if (on_motion_timeout) on_motion_timeout();
    };
}

ZLAC8015Driver::~ZLAC8015Driver() = default;

void ZLAC8015Driver::set_bus(BusInterface* bus) {
    bus_ = bus;
    drive_->set_bus(bus);
}

void ZLAC8015Driver::set_sdo_timeout(uint32_t ms) {
    drive_->set_sdo_timeout(ms);
}

void ZLAC8015Driver::set_wheel_parameters(double wheel_radius_m, double wheelbase_m) {
    wheel_radius_ = wheel_radius_m;
    wheelbase_ = wheelbase_m;
}

bool ZLAC8015Driver::init(uint32_t timeout_ms) {
    if (!bus_) return false;

    log("init: SDO timeout = 100ms");

    drive_->set_sdo_timeout(100);  // giống LELY_CO_NMT_TIMEOUT = 100ms
    drive_->set_sdo_verbose(true); // trace mọi SDO request/response

    // ---- Phase 1: đưa node về Operational (bắt buộc với ZLAC firmware) ----
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 3000);

    log("init: NMT Reset Communication (0x82)");
    nmt_command(bus_, node_id_, NMT_RESET_COMM);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    log("init: NMT Start (0x01)");
    nmt_command(bus_, node_id_, NMT_START);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Thử đọc statusword tối đa 5 lần (retry như lely)
    SDOError poll_err = SDOError::TIMEOUT;
    for (int attempt = 0; attempt < 5; ++attempt) {
        poll_err = drive_->poll_status();
        if (poll_err == SDOError::OK) {
            log("init: poll_status OK, statusword = 0x" +
                hex4(drive_->get_statusword()) +
                " (attempt " + std::to_string(attempt + 1) + ")");
            break;
        }
        log("init: poll_status " + sdo_error_str(poll_err) +
            ", gửi lại NMT Start (attempt " + std::to_string(attempt + 1) + ")");
        nmt_command(bus_, node_id_, NMT_START);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (std::chrono::steady_clock::now() > deadline) break;
    }

    if (poll_err != SDOError::OK) {
        log("init: FAIL - SDO không phản hồi sau 5 lần thử");
        return false;
    }

    // ---- Phase 2: CiA 402 enable sequence ----
    drive_->init();
    const bool ok = enable();
    log(std::string("init: ") + (ok ? "SUCCESS" : "FAIL ở enable()") +
        ", statusword = 0x" + hex4(drive_->get_statusword()));
    return ok;
}

bool ZLAC8015Driver::enable() {
    if (!bus_) return false;

    // CiA 402 enable sequence (giống bản lely-core hoạt động):
    //   Shutdown(0x0006) -> Switch On(0x0007) -> Enable Operation(0x000F)
    //   -> Unlock Ramp Generator(0x007F)
    const uint16_t steps[] = {CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OPER, CW_UNLOCK_RAMP};
    const char* names[] = {"Shutdown(0x0006)", "SwitchOn(0x0007)",
                           "EnableOper(0x000F)", "UnlockRamp(0x007F)"};

    for (int i = 0; i < 4; ++i) {
        drive_->set_controlword(steps[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const SDOError err = drive_->poll_status();
        log("enable: " + std::string(names[i]) + " -> SDO " +
            sdo_error_str(err) + ", statusword = 0x" + hex4(drive_->get_statusword()));

        if (err != SDOError::OK) {
            // Retry: gửi lại controlword (lely cũng retry khi SDO fail)
            nmt_command(bus_, node_id_, NMT_START);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            drive_->set_controlword(steps[i]);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            drive_->poll_status();
        }
    }

    // Success = không fault và đã rời SWITCH_ON_DISABLED
    const uint16_t sw = drive_->get_statusword();
    return !(sw & SW_FAULT) && !(sw & SW_SWITCH_ON_DISABLED);
}

bool ZLAC8015Driver::disable() {
    if (!bus_) return false;

    drive_->set_controlword(CW_DISABLE_VOLTAGE);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    drive_->poll_status();
    return true;
}

void ZLAC8015Driver::quick_stop() {
    drive_->quick_stop();
}

bool ZLAC8015Driver::fault_reset() {
    if (!bus_) return false;

    drive_->fault_reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    drive_->poll_status();

    // Re-enable after fault reset
    if (!drive_->has_fault()) {
        return enable();
    }
    return true;
}

bool ZLAC8015Driver::set_profile(uint32_t profile_velocity, uint32_t accel,
                                 uint32_t decel) {
    if (!bus_) return false;

    bool ok = drive_->write_profile_velocity_axis(1, profile_velocity);
    ok = drive_->write_profile_velocity_axis(2, profile_velocity) && ok;

    ok = drive_->write_profile_acceleration_axis(1, accel) && ok;
    ok = drive_->write_profile_acceleration_axis(2, accel) && ok;

    ok = drive_->write_profile_deceleration_axis(1, decel) && ok;
    ok = drive_->write_profile_deceleration_axis(2, decel) && ok;

    return ok;
}

bool ZLAC8015Driver::set_operation_mode(int8_t mode) {
    // Retry 5 lần (giống logic lely: write 0x6060 tối đa 5 lần, fail → gửi lại NMT START)
    for (int attempt = 0; attempt < 5; ++attempt) {
        if (drive_->set_operation_mode(static_cast<OperationMode>(mode))) {
            return true;
        }
        nmt_command(bus_, node_id_, NMT_START);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

bool ZLAC8015Driver::set_velocity_rpm(int16_t left_rpm, int16_t right_rpm) {
    if (!bus_) return false;

    // Invert right motor because it's mounted in the opposite direction
    right_rpm = -right_rpm;

    // Per-axis 16-bit writes (0x60FF:01, 0x60FF:02)
    bool ok = drive_->write_velocity_axis(1, left_rpm);
    ok = drive_->write_velocity_axis(2, right_rpm) && ok;

    // Combined 32-bit write (0x60FF:03) — ZLAC8015D specific.
    // Đây là frame chính mà bản lely-core dùng:
    //   0x601#23FF03 LL LL RR RR
    const uint32_t combined =
        (static_cast<uint32_t>(static_cast<uint16_t>(left_rpm)) & 0xFFFF) |
        (static_cast<uint32_t>(static_cast<uint16_t>(right_rpm)) << 16);
    ok = drive_->write_velocity_combined(combined) && ok;

    return ok;
}

uint16_t ZLAC8015Driver::read_status() {
    drive_->poll_status();
    return drive_->get_statusword();
}

int16_t ZLAC8015Driver::get_velocity_left() {
    int16_t v = 0;
    drive_->read_velocity_axis(1, v);
    return v;
}

int16_t ZLAC8015Driver::get_velocity_right() {
    int16_t v = 0;
    drive_->read_velocity_axis(2, v);
    return -v;
}

int32_t ZLAC8015Driver::get_position_left() {
    int32_t p = 0;
    drive_->read_position_axis(1, p);
    return p;
}

int32_t ZLAC8015Driver::get_position_right() {
    int32_t p = 0;
    drive_->read_position_axis(2, p);
    return -p;
}

void ZLAC8015Driver::update() {
    drive_->poll_status();
}

} // namespace drivers
} // namespace canopen
