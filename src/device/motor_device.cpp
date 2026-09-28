/**
 * @file motor_device.cpp
 * @brief Triển khai facade cấp cao cho thiết bị động cơ CiA 402
 */

#include <canopen/device/motor_device.hpp>

#include <canopen/can/msg/message_factory.hpp>

#include <chrono>
#include <cstdio>
#include <thread>

namespace canopen {

namespace {

// NMT commands (CiA 301)
constexpr uint8_t NMT_START = 0x01;
constexpr uint8_t NMT_STOP = 0x02;
constexpr uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80;
constexpr uint8_t NMT_RESET_COMM = 0x82;

std::string hex4(uint16_t v) {
    char b[8];
    std::snprintf(b, sizeof(b), "%04X", v);
    return std::string(b);
}

void nmt_cmd(BusInterface& bus, uint8_t node, uint8_t cmd) {
    CANFrame f;
    f.set_id(0x000);
    f.set_len(2);
    f.set_u8(0, cmd);
    f.set_u8(1, node);
    bus.send(f);
}

/** Trạng thái NMT của node, đọc qua object 0x1019. false = không đọc được */
bool read_nmt_state(SDOClient& sdo, NMTState& out) {
    uint8_t v = 0;
    size_t len = 1;
    if (sdo.upload_sync(0x1019, 0x00, &v, len) != SDOError::OK || len != 1) {
        return false;
    }
    out = static_cast<NMTState>(v);
    return true;
}

/**
 * @brief Chuyển statusword thô thành state CiA 402
 *
 * Theo CiA 402, state được lấy từ statusword & 0x4F, sau đó đối chiếu với
 * mã trạng thái. Không thể dùng statusword trực tiếp làm state: giá trị thô
 * có nhiều bit khác (ví dụ ZLAC trả 0x1421 cho "ready to switch on").
 */
CiA402State decode_state(uint16_t statusword) {
    switch (statusword & 0x4F) {
        case 0x00: return CiA402State::NOT_READY_TO_SWITCH_ON;
        case 0x40: return CiA402State::SWITCH_ON_DISABLED;
        case 0x01: return CiA402State::READY_TO_SWITCH_ON;
        case 0x03: return CiA402State::SWITCHED_ON;
        case 0x07: return CiA402State::OPERATION_ENABLED;
        case 0x05: return CiA402State::QUICK_STOP_ACTIVE;
        case 0x0F: return CiA402State::FAULT_REACTION_ACTIVE;
        case 0x08: return CiA402State::FAULT;
        default:   return CiA402State::NOT_READY_TO_SWITCH_ON;
    }
}

const char* cia402_state_name(CiA402State s) {
    switch (s) {
        case CiA402State::NOT_READY_TO_SWITCH_ON: return "Not ready to switch on";
        case CiA402State::SWITCH_ON_DISABLED:     return "Switch on disabled";
        case CiA402State::READY_TO_SWITCH_ON:     return "Ready to switch on";
        case CiA402State::SWITCHED_ON:            return "Switched on";
        case CiA402State::OPERATION_ENABLED:      return "Operation enabled";
        case CiA402State::QUICK_STOP_ACTIVE:      return "Quick stop active";
        case CiA402State::FAULT_REACTION_ACTIVE:  return "Fault reaction active";
        case CiA402State::FAULT:                  return "Fault";
    }
    return "?";
}

} // namespace

// ==================== Construction ====================

MotorDevice::MotorDevice(BusInterface& bus, DeviceProfile profile)
    : bus_(&bus), profile_(std::move(profile)) {
    sdo_ = std::make_unique<SDOClient>(&bus, profile_.node_id());
    sdo_->set_timeout(500);
    // Constructor của SDOClient KHÔNG tự đăng ký route — phải attach thủ công,
    // nếu không sẽ không nhận được response nào từ thiết bị.
    sdo_->attach(bus);
}

MotorDevice::~MotorDevice() {
    disconnect();
}

// ==================== Read / write theo vai trò ====================

bool MotorDevice::read_value(uint16_t index, uint8_t subindex, double& out) {
    if (!sdo_) return false;
    uint8_t buf[8] = {0};
    size_t len = sizeof(buf);
    if (sdo_->upload_sync(index, subindex, buf, len) != SDOError::OK) return false;
    if (len == 0) return false;

    // Dùng kiểu dữ liệu từ EDS nếu có, nếu không thì đoán theo độ dài
    DataType dt = DataType::UNSIGNED32;
    size_t size = len;
    if (const ObjectEntry* e = profile_.dictionary().get_object(index, subindex)) {
        dt = e->data_type;
        if (e->size) size = e->size;
    }
    ResolvedObject probe;
    probe.data_type = dt;
    probe.size = size;
    return probe.to_double(buf, len, out);
}

bool MotorDevice::write_value(uint16_t index, uint8_t subindex, double value) {
    if (!sdo_) return false;
    uint8_t buf[8] = {0};
    size_t len = sizeof(buf);

    DataType dt = DataType::UNSIGNED32;
    size_t size = 0;
    if (const ObjectEntry* e = profile_.dictionary().get_object(index, subindex)) {
        dt = e->data_type;
        size = e->size;
    }
    ResolvedObject probe;
    probe.data_type = dt;
    probe.size = size;
    if (!probe.from_double(value, buf, len)) return false;

    return sdo_->download_sync(index, subindex, buf, len) == SDOError::OK;
}

bool MotorDevice::read_role(ObjectRole role, double& out) {
    const ResolvedObject& o = profile_.resolve(role);
    if (!o.valid) return false;
    if (!sdo_) return false;
    sdo_gap();

    uint8_t buf[8] = {0};
    size_t len = sizeof(buf);
    if (sdo_->upload_sync(o.index, o.subindex, buf, len) != SDOError::OK) {
        return false;
    }
    return o.to_double(buf, len, out);
}

bool MotorDevice::write_role(ObjectRole role, double value) {
    const ResolvedObject& o = profile_.resolve(role);
    if (!o.valid) return false;
    if (!sdo_) return false;
    sdo_gap();

    uint8_t buf[8] = {0};
    size_t len = sizeof(buf);
    if (!o.from_double(value, buf, len)) return false;
    return sdo_->download_sync(o.index, o.subindex, buf, len) == SDOError::OK;
}

// ==================== Kết nối ====================

bool MotorDevice::transition(uint16_t controlword, CiA402State expect,
                             uint32_t timeout_ms) {
    const ResolvedObject& cw = profile_.resolve(ObjectRole::Controlword);
    const ResolvedObject& sw = profile_.resolve(ObjectRole::Statusword);
    if (!cw.valid || !sw.valid) return false;

    uint8_t wbuf[8] = {0};
    size_t wlen = sizeof(wbuf);
    if (!cw.from_double(controlword, wbuf, wlen)) return false;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    // Ghi lại nhiều lần: drive thật có thể rơi lệnh SDO, và nếu chỉ gửi một
    // lần mà lệnh rơi thì transition sẽ thất bại oan.
    while (std::chrono::steady_clock::now() < deadline) {
        sdo_gap();
        sdo_->download_sync(cw.index, cw.subindex, wbuf, wlen);

        // Chờ một nhịp ngắn rồi đọc lại, thay vì đọc tức thì — drive cần
        // thời gian áp dụng controlword trước khi statusword phản ánh.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        uint8_t sbuf[8] = {0};
        size_t slen = sizeof(sbuf);
        sdo_gap();
        if (sdo_->upload_sync(sw.index, sw.subindex, sbuf, slen) == SDOError::OK) {
            double raw = 0;
            ResolvedObject probe;
            probe.data_type = sw.data_type;
            probe.size = slen;
            probe.byte_order = sw.byte_order;
            if (probe.to_double(sbuf, slen, raw) &&
                decode_state(static_cast<uint16_t>(raw)) == expect) {
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Chờ thiết bị phản hồi SDO sau khi gửi NMT Start
 *
 * Một số drive còn đang xử lý NMT Start khi ta đã ghi SDO, nên lệnh ghi đầu
 * tiên sẽ rơi lặng lẽ. Chờ cho tới khi đọc được statusword là cách xác nhận
 * thiết bị thật sự sẵn sàng nhận lệnh.
 */
/**
 * @brief Chờ khoảng cách tối thiểu giữa hai giao dịch SDO
 *
 * Một số drive (ZLAC8015D) bỏ qua yêu cầu SDO tới ngay sau khi vừa trả lời
 * yêu cầu trước đó — quan sát được rõ: đọc statusword lần đầu cho kết quả
 * đúng, lần ngay sau đó trả về 0. Nghỉ tối thiểu vài mili giây để tránh.
 */
void MotorDevice::sdo_gap() {
    if (sdo_gap_ms_ == 0) return;
    const auto gap = std::chrono::milliseconds(sdo_gap_ms_);
    const int64_t last = last_sdo_.load();
    if (last != 0) {
        const auto since =
            std::chrono::steady_clock::now().time_since_epoch().count() - last;
        if (since < gap.count() * 1000000) {
            std::this_thread::sleep_for(
                std::chrono::nanoseconds(gap.count() * 1000000 - since));
        }
    }
    last_sdo_.store(std::chrono::steady_clock::now().time_since_epoch().count());
}

/**
 * @brief Chờ statusword ổn định trước khi ghi controlword
 *
 * Lúc vừa cấp điện, drive còn đang tự kiểm tra và statusword đổi liên tục.
 * Nếu ta ghi controlword quá sớm, drive sẽ ghi đè lệnh của ta khi hoàn tất
 * khởi động.
 *
 * Tiêu chí là TRẠNG THÁI ỔN ĐỊNH (hai lần đọc liên tiếp cho cùng giá trị),
 * không phải một giá trị cụ thể: drive chuẩn dừng ở Switch on disabled
 * (0x1440) còn ZLAC dùng 0x1400 làm trạng thái nghỉ — cả hai đều hợp lệ.
 */
bool MotorDevice::wait_boot_complete() {
    const ResolvedObject& sw = profile_.resolve(ObjectRole::Statusword);
    if (!sw.valid) return true;

    auto read_sw = [this, &sw](uint16_t& out) -> bool {
        sdo_gap();
        uint8_t buf[8] = {0};
        size_t len = sizeof(buf);
        if (sdo_->upload_sync(sw.index, sw.subindex, buf, len) != SDOError::OK) {
            return false;
        }
        double raw = 0;
        ResolvedObject probe;
        probe.data_type = sw.data_type;
        probe.size = len;
        probe.byte_order = sw.byte_order;
        if (!probe.to_double(buf, len, raw)) return false;
        out = static_cast<uint16_t>(raw);
        return true;
    };

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(boot_timeout_ms_);
    uint16_t prev = 0;
    bool have_prev = false;
    int stable = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        uint16_t sw_val = 0;
        if (read_sw(sw_val)) {
            if (have_prev && sw_val == prev) {
                ++stable;
                if (stable >= 2) {
                    log("connect: drive đã ổn định — " +
                        std::string(cia402_state_name(decode_state(sw_val))) +
                        " (statusword 0x" + hex4(sw_val) + ")");
                    return true;
                }
            } else {
                stable = 0;
            }
            prev = sw_val;
            have_prev = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Không bao giờ ổn định trong thời gian cho phép — vẫn tiếp tục, vì có
    // drive báo trạng thái nhấp nháy (ví dụ do bộ nguồn yếu) nhưng vẫn nhận
    // lệnh. Chặn ở đây chỉ khiến connect() thất bại oan.
    log("connect: statusword chưa ổn định sau " +
        std::to_string(boot_timeout_ms_) + "ms — vẫn thử enable");
    return true;
}

bool MotorDevice::wait_responsive() {
    const ResolvedObject& sw = profile_.resolve(ObjectRole::Statusword);
    if (!sw.valid) return true;

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(sdo_timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t buf[8] = {0};
        size_t len = sizeof(buf);
        sdo_gap();
        if (sdo_->upload_sync(sw.index, sw.subindex, buf, len) == SDOError::OK) {
            log("connect: thiết bị đã sẵn sàng nhận lệnh");
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

bool MotorDevice::connect(uint32_t timeout_ms) {
    if (!bus_ || !sdo_) return false;
    if (!profile_.is_usable()) {
        log("connect: hồ sơ thiết bị thiếu object bắt buộc — không thể kết nối");
        log(profile_.describe());
        return false;
    }

    const uint8_t node = profile_.node_id();
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    // Bước 1: NMT Start — đưa node ra khỏi Pre-operational (PDO chỉ chạy ở
    // Operational).
    //
    // Xác nhận Operational bằng object 0x1019 CHỈ KHI thiết bị có object đó.
    // CiA 301 bắt buộc phải có, nhưng thực tế ZLAC8015D không khai báo và
    // trả lỗi abort — nếu phụ thuộc cứng vào 0x1019 thì sẽ không kết nối
    // được với chính thiết bị này. Khi thiếu, bước enable bên dưới (đọc
    // statusword) mới là bằng chứng xác nhận thiết bị đã sẵn sàng.
    // Bước 0: NMT Reset Communication (0x82) đưa thiết bị về trạng thái sạch
    // — đây là trình tự mà master thực sự cần làm sau khi thiết bị khởi
    // động. Thiết bị mới vừa bật có thể chưa sẵn sàng và statusword trả về 0
    // cho tới khi nhận lệnh này.
    if (nmt_reset_on_connect_) {
        log("connect: NMT Reset Communication (0x82)");
        nmt_cmd(*bus_, node, NMT_RESET_COMM);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const bool expect_1019 = profile_.eds_has(0x1019, 0x00);
    bool operational = false;
    NMTState nmt_state = NMTState::INITIALISING;

    while (std::chrono::steady_clock::now() < deadline && !operational) {
        nmt_cmd(*bus_, node, NMT_START);

        if (expect_1019) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (read_nmt_state(*sdo_, nmt_state)) {
                operational = (nmt_state == NMTState::OPERATIONAL);
            }
        } else {
            // Không đọc được NMT state (vd ZLAC không có 0x1019). Thay vì
            // ngủ cứng một khoảng thời gian, chờ tới khi thiết bị THỰC SỰ
            // phản hồi SDO: đọc statusword cho tới khi thành công. Lệnh
            // NMT Start vừa gửi có thể còn đang được xử lý, ghi SDO quá sớm
            // sẽ bị bỏ qua và enable thất bại.
            operational = wait_responsive();
        }
    }
    if (!operational) {
        log("connect: node " + std::to_string(node) +
            " không vào được OPERATIONAL (0x1019 = " +
            std::to_string(static_cast<int>(nmt_state)) + ")");
        return false;
    }
    if (!expect_1019) {
        log("connect: thiết bị không có 0x1019 — chờ phản hồi SDO thay vì "
            "kiểm tra NMT state");
    }

    // Bước 1b: chờ thiết bị hoàn tất tự kiểm tra lúc khởi động.
    //
    // CiA 402: lúc vừa bật, statusword báo Not ready to switch on. Thiết bị
    // tự chuyển sang Switch on disabled sau khi tự kiểm tra xong. Nếu ta ghi
    // controlword quá sớm, drive sẽ GHI ĐÈ trạng thái của ta khi hoàn tất
    // khởi động — biểu hiện: ghi 0x0006 xong, statusword không đổi.
    if (!wait_boot_complete()) {
        log("connect: thiết bị không thoát khỏi trạng thái khởi động");
        return false;
    }

    // Bước 2: chuỗi enable CiA 402. Đọc statusword để biết đang ở bước nào
    // thay vì giả định thứ tự cố định — một số drive cần bỏ qua bước.
    const uint16_t kShutdown = 0x0006;
    const uint16_t kSwitchOn = 0x0007;
    const uint16_t kEnableOp = 0x000F;

    const ResolvedObject& sw = profile_.resolve(ObjectRole::Statusword);
    auto read_state = [&]() -> CiA402State {
        uint8_t b[8] = {0};
        size_t l = sizeof(b);
        sdo_gap();
        if (sdo_->upload_sync(sw.index, sw.subindex, b, l) != SDOError::OK) {
            return CiA402State::NOT_READY_TO_SWITCH_ON;
        }
        ResolvedObject p;
        p.data_type = sw.data_type;
        p.size = l;
        double v = 0;
        if (!p.to_double(b, l, v)) return CiA402State::NOT_READY_TO_SWITCH_ON;
        return decode_state(static_cast<uint16_t>(v));
    };
    // Drive có thể đang ở Fault — báo rõ thay vì im lặng
    CiA402State st = read_state();
    if (st == CiA402State::FAULT) {
        log("connect: thiết bị đang ở trạng thái FAULT — cần reset lỗi trước");
        return false;
    }

    log("connect: state hiện tại = " + std::string(cia402_state_name(st)) +
        " (statusword 0x" + hex4(statusword()) + ")");

    // Tiến lần lượt; mỗi bước chỉ cần khi thiết bị chưa ở trạng thái đó
    for (int i = 0; i < 2; ++i) {
        st = read_state();
        if (st != CiA402State::READY_TO_SWITCH_ON &&
            st != CiA402State::SWITCHED_ON &&
            st != CiA402State::OPERATION_ENABLED) {
            if (!transition(kShutdown, CiA402State::READY_TO_SWITCH_ON, 1500)) {
                log("connect: không vào được READY_TO_SWITCH_ON (statusword 0x" +
                    hex4(statusword()) + ")");
                return false;
            }
        }
        st = read_state();
        if (st != CiA402State::SWITCHED_ON &&
            st != CiA402State::OPERATION_ENABLED) {
            if (!transition(kSwitchOn, CiA402State::SWITCHED_ON, 1500)) {
                log("connect: không vào được SWITCHED_ON (statusword 0x" +
                    hex4(statusword()) + ")");
                return false;
            }
        }
        st = read_state();
        if (st != CiA402State::OPERATION_ENABLED) {
            if (!transition(kEnableOp, CiA402State::OPERATION_ENABLED, 2000)) {
                log("connect: không vào được OPERATION_ENABLED (state = " +
                    std::string(cia402_state_name(read_state())) + ")");
                return false;
            }
        }
        break;
    }

    connected_.store(true);
    log("connect: OK — " + std::string(cia402_state_name(cia402_state())));
    return true;
}

void MotorDevice::disconnect() {
    if (!connected_.exchange(false)) return;
    // Dừng tốc độ rồi tắt nguồn điều khiển — không để motor chạy tiếp
    stop();
    const ResolvedObject& cw = profile_.resolve(ObjectRole::Controlword);
    if (cw.valid && sdo_) {
        uint8_t buf[8] = {0};
        size_t len = sizeof(buf);
        if (cw.from_double(0x0006, buf, len)) {
            sdo_->download_sync(cw.index, cw.subindex, buf, len);
        }
    }
    if (bus_) nmt_cmd(*bus_, profile_.node_id(), NMT_STOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool MotorDevice::reconnect(uint32_t timeout_ms) {
    log("reconnect: thử kết nối lại node " + std::to_string(profile_.node_id()));
    connected_.store(false);
    if (bus_) nmt_cmd(*bus_, profile_.node_id(), NMT_ENTER_PRE_OPERATIONAL);
    return connect(timeout_ms);
}

// ==================== Chế độ & profile ====================

bool MotorDevice::set_operation_mode(OperationMode mode, int attempts) {
    if (!profile_.has(ObjectRole::ModesOfOperation)) return false;
    for (int i = 0; i < attempts; ++i) {
        if (write_role(ObjectRole::ModesOfOperation, static_cast<int>(mode))) {
            // Chờ thiết bị báo lại chế độ đã nhận (0x6061)
            for (int k = 0; k < 10; ++k) {
                double v = 0;
                if (read_role(ObjectRole::ModesOfOperationDisplay, v) &&
                    static_cast<int>(v) == static_cast<int>(mode)) {
                    mode_ = mode;
                    return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            mode_ = mode;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

bool MotorDevice::set_profile_velocity(double v) { return write_role(ObjectRole::ProfileVelocity, v); }
bool MotorDevice::set_profile_acceleration(double a) { return write_role(ObjectRole::ProfileAcceleration, a); }
bool MotorDevice::set_profile_deceleration(double d) { return write_role(ObjectRole::ProfileDeceleration, d); }

// ==================== Điều khiển tốc độ ====================

bool MotorDevice::set_velocity(double velocity) {
    if (!write_role(ObjectRole::TargetVelocity, velocity)) return false;
    command_count_.fetch_add(1);
    return true;
}

double MotorDevice::velocity() {
    double v = 0;
    if (!read_role(ObjectRole::ActualVelocity, v)) return 0;
    return v;
}

bool MotorDevice::stop() {
    return write_role(ObjectRole::TargetVelocity, 0);
}

// ==================== Trạng thái ====================

CiA402State MotorDevice::cia402_state() {
    double v = 0;
    if (!read_role(ObjectRole::Statusword, v)) {
        return CiA402State::NOT_READY_TO_SWITCH_ON;
    }
    return decode_state(static_cast<uint16_t>(v));
}

bool MotorDevice::write_controlword(uint16_t value) {
    return write_role(ObjectRole::Controlword, static_cast<double>(value));
}

uint16_t MotorDevice::statusword() {
    double v = 0;
    read_role(ObjectRole::Statusword, v);
    return static_cast<uint16_t>(v);
}

uint32_t MotorDevice::error_code() {
    double v = 0;
    if (!read_role(ObjectRole::ErrorCode, v)) return 0;
    return static_cast<uint32_t>(v);
}

} // namespace canopen
