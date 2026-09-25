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

static std::string hex8(uint32_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%08X", v);
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

// ==================== Encoder / Odometry ====================

bool ZLAC8015Driver::read_encoder_lines(uint16_t& left_lines, uint16_t& right_lines) {
    if (!bus_) return false;

    // 0x200E:01 = Left Motor Encoder Line, 0x200E:02 = Right
    uint16_t l = 0, r = 0;
    const bool ok_l = drive_->read_encoder_line(1, l);
    const bool ok_r = drive_->read_encoder_line(2, r);

    if (ok_l) encoder_lines_left_ = l;
    if (ok_r) encoder_lines_right_ = r;

    left_lines = encoder_lines_left_;
    right_lines = encoder_lines_right_;

    log("encoder_line: left=" + std::to_string(encoder_lines_left_) +
        " right=" + std::to_string(encoder_lines_right_));
    return ok_l || ok_r;
}

uint16_t ZLAC8015Driver::encoder_lines_left() {
    if (encoder_lines_left_ == 0) {
        uint16_t l = 0, r = 0;
        read_encoder_lines(l, r);
    }
    return encoder_lines_left_ ? encoder_lines_left_ : 1024;  // fallback default
}

double ZLAC8015Driver::counts_to_rad(int32_t counts) const {
    const uint16_t lines = encoder_lines_left_ ? encoder_lines_left_ : 1024;
    return counts * (2.0 * M_PI) / lines;
}

bool ZLAC8015Driver::set_encoder_lines(uint16_t lines) {
    if (!bus_ || lines == 0) return false;

    const bool ok1 = drive_->write_encoder_line(1, lines);
    const bool ok2 = drive_->write_encoder_line(2, lines);
    if (ok1 && ok2) {
        encoder_lines_left_ = lines;
        encoder_lines_right_ = lines;
    }
    return ok1 && ok2;
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

    // ---- Phase 3: cấu hình PDO (SDO, 1 lần) để điều khiển thời gian thực ----
    if (ok) {
        setup_pdo();
    }
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

// ==================== PDO setup (SDO, chỉ 1 lần lúc khởi tạo) ====================

bool ZLAC8015Driver::setup_pdo() {
    if (!bus_ || !drive_) return false;

    rpdo_cobid_ = 0x200u + node_id_;
    tpdo_cobid_ = 0x180u + node_id_;

    // ---- QUAN TRỌNG: 0x200F = 0 (Asynchronous control) ----
    // Nếu 0x200F = 1 (Synchronization), drive CHỈ nhận RPDO khi có SYNC
    // frame trên bus (theo PDF dòng 2396-2402) → velocity không được áp dụng.
    drive_->sdo_write_u8(0x200F, 0x00, 0);
    log("setup_pdo: 0x200F=0 (asynchronous control)");

    // ==================== Cấu hình RPDO1 theo ĐÚNG THỨ TỰ của tài liệu ZLAC ====================
    // Thứ tự bắt buộc (theo ví dụ TPDO trong can_help.pdf):
    //   1. 0x1600:00 = 0            Clear mapping
    //   2. 0x1600:01 = <entry>      Ghi mapping entry
    //   3. 0x1400:01 = COB-ID       Đặt COB-ID
    //   4. 0x1400:02 = trans type   Transmission type
    //   5. 0x1400:03 = inhibit      Inhibit time
    //   6. 0x1600:00 = 1            ★ START MAPPING — PHẢI ĐẶT CUỐI ★
    //   7. 0x2010:00 = 2            Lưu EEPROM
    //
    // Nếu đặt số lượng mapping TRƯỚC, drive sẽ "kích hoạt" mapping rỗng
    // rồi BỎ QUA frame RPDO đến (thực nghiệm 2026-09).

    auto configure_rpdo1 = [&](uint8_t entry_count, const uint32_t* entries) {
        drive_->sdo_write_u8(0x1600, 0x00, 0);                     // 1. clear
        for (uint8_t i = 0; i < entry_count; ++i) {
            drive_->sdo_write_u32(0x1600, static_cast<uint8_t>(i + 1), entries[i]);
        }
        drive_->sdo_write_u32(0x1400, 0x01, rpdo_cobid_);          // 3. COB-ID
        drive_->sdo_write_u8(0x1400, 0x02, 255);                    // 4. async
        drive_->sdo_write_u16(0x1400, 0x03, 0);                     // 5. inhibit
        drive_->sdo_write_u8(0x1600, 0x00, entry_count);            // 6. START
    };

    // Thử 2 kiểu mapping, verify, dùng cái nào drive thực sự nhận
    bool ok = false;
    for (int attempt = 0; attempt < 2; ++attempt) {
        if (attempt == 0) {
            const uint32_t e[] = {0x60FF0320u};  // 1 entry 32-bit
            configure_rpdo1(1, e);
        } else {
            const uint32_t e[] = {0x60FF0110u, 0x60FF0220u};  // 2 entry 16-bit
            configure_rpdo1(2, e);
        }

        uint8_t n = 0;
        uint32_t m0 = 0, m1 = 0, cob = 0;
        const bool rd_ok =
            drive_->sdo_read_u8(0x1600, 0x00, n) &&
            drive_->sdo_read_u32(0x1600, 0x01, m0) &&
            drive_->sdo_read_u32(0x1400, 0x01, cob);
        if (attempt == 1) drive_->sdo_read_u32(0x1600, 0x02, m1);

        const bool enabled = (cob & 0x80000000u) == 0;
        const bool mapped =
            rd_ok && ((attempt == 0 && n == 1 && m0 == 0x60FF0320u) ||
                      (attempt == 1 && n == 2 && m0 == 0x60FF0110u &&
                       m1 == 0x60FF0220u));

        log("setup_pdo RPDO1: " +
            std::string(attempt == 0 ? "A) 1x32bit" : "B) 2x16bit") +
            " -> n=" + std::to_string(n) +
            " m0=0x" + hex8(m0) + (attempt == 1 ? " m1=0x" + hex8(m1) : "") +
            " cobid=0x" + hex8(cob) +
            (enabled ? " [enabled]" : " [DISABLED!]") +
            (mapped ? " [MAPPING OK]" : " [mapping rejected]"));

        if (enabled && mapped) {
            rpdo_mode_ = (attempt == 0) ? RpdoMode::Combined32 : RpdoMode::TwoAxes16;
            ok = true;
            break;
        }
    }

    log(std::string("setup_pdo: RPDO1 ") +
        (ok ? "cấu hình OK" : "thất bại cả 2 kiểu") +
        " — mapping start đặt CUỐI theo thứ tự tài liệu ZLAC");

    // Lưu cấu hình vào EEPROM (0x2010:00 = 2 = save all) để drive giữ
    // cấu hình PDO sau khi mất nguồn
    drive_->sdo_write_u8(0x2010, 0x00, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ==================== TPDO1: tốc độ thực tế ====================
    // Cùng thứ tự: clear → entry → COB-ID → type → inhibit → START
    drive_->sdo_write_u8(0x1A00, 0x00, 0);
    drive_->sdo_write_u32(0x1A00, 0x01, 0x606C0120u);
    drive_->sdo_write_u32(0x1A00, 0x02, 0x606C0220u);
    drive_->sdo_write_u32(0x1800, 0x01, tpdo_cobid_);
    drive_->sdo_write_u8(0x1800, 0x02, 255);
    drive_->sdo_write_u16(0x1800, 0x03, 0);
    drive_->sdo_write_u16(0x1800, 0x05, 100);   // event timer 100ms
    drive_->sdo_write_u8(0x1A00, 0x00, 2);      // ★ START mapping ★

    uint32_t t0 = 0, t1 = 0;
    if (drive_->sdo_read_u32(0x1A00, 0x01, t0) &&
        drive_->sdo_read_u32(0x1A00, 0x02, t1) &&
        t0 == 0x606C0120u && t1 == 0x606C0220u) {
        if (route_tpdo_ == 0) {
            route_tpdo_ = bus_->add_route(tpdo_cobid_, 0x7FF,
                [this](const CANFrame& f) { on_tpdo_frame(f); });
        }
        pdo_ready_ = true;
    } else {
        log("setup_pdo: TPDO1 mapping rejected (0x1A00:01=0x" + hex8(t0) +
            ") — phản hồi sẽ dùng SDO");
    }

    log(std::string("setup_pdo: ") +
        (pdo_ready_ ? "OK (RPDO1 " +
            std::string(rpdo_mode_ == RpdoMode::Combined32 ? "1x32bit" : "2x16bit") +
            ", TPDO1 OK)" : "PARTIAL — velocity qua SDO"));
    return pdo_ready_;
}

void ZLAC8015Driver::on_tpdo_frame(const CANFrame& frame) {
    if (frame.len() >= 8) {
        // 0x606C:01 (4 byte) + 0x606C:02 (4 byte)
        tpdo_vel_left_.store(frame.get_u32_le(0));
        tpdo_vel_right_.store(frame.get_u32_le(4));
    } else if (frame.len() >= 4) {
        tpdo_vel_left_.store(frame.get_u32_le(0));
    }
    tpdo_count_.fetch_add(1);
}

bool ZLAC8015Driver::set_velocity_rpm(int16_t left_rpm, int16_t right_rpm) {
    if (!bus_) return false;

    // Invert right motor because it's mounted in the opposite direction
    right_rpm = -right_rpm;

    // Bỏ qua nếu tốc độ không đổi — tránh spam bus
    if (velocity_sent_ && left_rpm == last_left_rpm_ &&
        right_rpm == last_right_rpm_) {
        return true;
    }

    const uint32_t combined =
        (static_cast<uint32_t>(static_cast<uint16_t>(left_rpm)) & 0xFFFF) |
        (static_cast<uint32_t>(static_cast<uint16_t>(right_rpm)) << 16);

    // RPDO (nhanh, 1 frame không chờ response) — mapping được cấu hình theo
    // ĐÚNG thứ tự tài liệu ZLAC (clear → entry → COB-ID → type → START).
    // Nếu drive vẫn bỏ qua, dùng use_pdo(false) để chuyển sang SDO.
    bool ok = true;
    if (pdo_enabled_ && pdo_ready_) {
        // Chỉ dùng khi người dùng ép buộc bật (mặc định TẮT)
        CANFrame frame;
        frame.set_id(rpdo_cobid_);
        frame.set_len(4);
        frame.set_u32_le(0, combined);
        ok = bus_->send(frame);
    } else {
        ok = drive_->write_velocity_combined(combined);
    }

    last_left_rpm_ = left_rpm;
    last_right_rpm_ = right_rpm;
    velocity_sent_ = true;
    return ok;
}

uint16_t ZLAC8015Driver::read_status() {
    drive_->poll_status();
    return drive_->get_statusword();
}

int16_t ZLAC8015Driver::get_velocity_left() {
    // Ưu tiên dữ liệu TPDO (không chặn, cập nhật liên tục)
    if (pdo_ready_ && tpdo_count_ > 0) {
        return static_cast<int16_t>(tpdo_vel_left_.load());
    }
    int16_t v = 0;
    drive_->read_velocity_axis(1, v);
    return v;
}

int16_t ZLAC8015Driver::get_velocity_right() {
    // Ưu tiên TPDO; đảo dấu vì motor phải ngược chiều
    if (pdo_ready_ && tpdo_count_ > 0) {
        return static_cast<int16_t>(-tpdo_vel_right_.load());
    }
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
