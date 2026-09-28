/**
 * @file motor_device.cpp
 * @brief Triển khai facade cấp cao cho thiết bị động cơ CiA 402
 */

#include <canopen/device/motor_device.hpp>

#include <canopen/can/msg/message_factory.hpp>

#include <chrono>
#include <thread>

namespace canopen {

namespace {

// NMT commands (CiA 301)
constexpr uint8_t NMT_START = 0x01;
constexpr uint8_t NMT_STOP = 0x02;
constexpr uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80;
constexpr uint8_t NMT_RESET_COMM = 0x82;

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

    uint8_t buf[8] = {0};
    size_t len = sizeof(buf);
    if (!cw.from_double(controlword, buf, len)) return false;
    if (sdo_->download_sync(cw.index, cw.subindex, buf, len) != SDOError::OK) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t sbuf[8] = {0};
        size_t slen = sizeof(sbuf);
        if (sdo_->upload_sync(sw.index, sw.subindex, sbuf, slen) == SDOError::OK) {
            double raw = 0;
            ResolvedObject probe;
            probe.data_type = sw.data_type;
            probe.size = slen;
            if (probe.to_double(sbuf, slen, raw)) {
                const auto state = static_cast<CiA402State>(static_cast<uint16_t>(raw));
                if (state == expect) return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
    // Operational). Nếu node chưa sẵn sàng thì Start sẽ bị bỏ qua, nên thử
    // lại trong khung thời gian.
    bool operational = false;
    while (std::chrono::steady_clock::now() < deadline && !operational) {
        nmt_cmd(*bus_, node, NMT_START);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        NMTState st_nmt = NMTState::INITIALISING;
        operational = read_nmt_state(*sdo_, st_nmt) && st_nmt == NMTState::OPERATIONAL;
    }
    if (!operational) {
        log("connect: node " + std::to_string(node) +
            " không vào được OPERATIONAL");
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
        if (sdo_->upload_sync(sw.index, sw.subindex, b, l) != SDOError::OK) {
            return CiA402State::NOT_READY_TO_SWITCH_ON;
        }
        ResolvedObject p;
        p.data_type = sw.data_type;
        p.size = l;
        double v = 0;
        if (!p.to_double(b, l, v)) return CiA402State::NOT_READY_TO_SWITCH_ON;
        return static_cast<CiA402State>(static_cast<uint16_t>(v));
    };
    // Drive có thể đang ở Fault — báo rõ thay vì im lặng
    CiA402State st = read_state();
    if (st == CiA402State::FAULT) {
        log("connect: thiết bị đang ở trạng thái FAULT — cần reset lỗi trước");
        return false;
    }

    log("connect: state hiện tại = " + std::string(cia402_state_name(st)));

    // Tiến lần lượt; mỗi bước chỉ cần khi thiết bị chưa ở trạng thái đó
    for (int i = 0; i < 2; ++i) {
        st = read_state();
        if (st != CiA402State::READY_TO_SWITCH_ON &&
            st != CiA402State::SWITCHED_ON &&
            st != CiA402State::OPERATION_ENABLED) {
            if (!transition(kShutdown, CiA402State::READY_TO_SWITCH_ON, 1500)) {
                log("connect: không vào được READY_TO_SWITCH_ON");
                return false;
            }
        }
        st = read_state();
        if (st != CiA402State::SWITCHED_ON &&
            st != CiA402State::OPERATION_ENABLED) {
            if (!transition(kSwitchOn, CiA402State::SWITCHED_ON, 1500)) {
                log("connect: không vào được SWITCHED_ON");
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
    return static_cast<CiA402State>(static_cast<uint16_t>(v));
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
