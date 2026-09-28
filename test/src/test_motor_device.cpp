/**
 * @file test_motor_device.cpp
 * @brief Kiểm tra MotorDevice trên một drive CiA 402 ảo (không cần phần cứng)
 *
 * FakeBus được nối với một "drive ảo" trả lời SDO đúng chuẩn: đọc statusword,
 * nhận controlword, nhận target velocity, báo NMT state. Nhờ đó kiểm được
 * chuỗi enable, chuyển chế độ, và việc đóng gói giá trị đúng kiểu dữ liệu —
 * tức là đúng những thứ mà DeviceProfile tự dò giúp.
 */

#include <canopen/can/msg/message_factory.hpp>
#include <canopen/device/motor_device.hpp>
#include <canopen/mock/fake_bus.hpp>

#include <atomic>
#include <iostream>
#include <thread>

using namespace canopen;

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
 * @brief Drive CiA 402 ảo, trả lời SDO theo object dictionary nội bộ
 */
class VirtualCiA402Drive {
public:
    explicit VirtualCiA402Drive(uint8_t node) : node_(node) {}

    /** Gắn vào bus, xử lý frame SDO 0x600+node */
    void attach(BusInterface& bus) {
        bus_ = &bus;
        // Echo bật: request SDO gửi đi sẽ được dispatch lại vào route,
        // nhờ đó drive ảo "nhận" được lệnh và trả lời. Route chỉ khớp
        // 0x600+node nên response 0x580+node không bị xử lý lại.
        route_ = bus.add_route(0x600u + node_, 0x7FF, [this](const CANFrame& f) {
            handle_sdo(f);
        });
    }

    void detach(BusInterface& bus) {
        if (route_) bus.remove_route(route_);
        route_ = 0;
    }

    // ==================== Trạng thái quan sát được ====================
    uint16_t controlword() const { return controlword_; }
    uint16_t statusword() const { return statusword_; }
    int32_t target_velocity() const { return target_velocity_; }
    int8_t mode() const { return mode_; }
    uint32_t sdo_writes() const { return sdo_writes_; }
    bool nmt_operational() const { return nmt_operational_; }

    void set_nmt_operational(bool v) { nmt_operational_ = v; }
    /** 0x1019 là bắt buộc theo CiA 301 nhưng ZLAC8015D không có */
    void set_supports_1019(bool v) { supports_1019_ = v; }
    /** ZLAC trả statusword dạng 0x1421/0x1C27 thay vì 0x0021/0x0027 */
    void set_zlac_style_statusword(bool v) { zlac_style_ = v; }
    void set_faulted(bool f) { faulted_ = f; }

    /**
     * @brief Mô phỏng máy trạng thái CiA 402
     *
     * 0x0000 NotReady, 0x0001 SwitchOnDisabled, 0x0002 ReadyToSwitchOn,
     * 0x0003 SwitchedOn, 0x0004 OperationEnabled, 0x0007 Fault
     */
    void update_state_from_controlword() {
        if (faulted_) { statusword_ = 0x0007; return; }
        const uint16_t cw = controlword_;
        // Statusword THỰC TẾ theo CiA 402, không phải mã state.
        // bit5 = quick stop không active; giá trị chuẩn: 0x21/0x23/0x27.
        if (zlac_style_) {
            if (cw == 0x0006) statusword_.store(0x1421);
            else if (cw == 0x0007) statusword_.store(0x1423);
            else if (cw == 0x000F) statusword_.store(0x1C27);
            else if (cw == 0x0000) statusword_.store(0x1400);
        } else {
            if (cw == 0x0006) statusword_.store(0x0021);      // Ready to switch on
            else if (cw == 0x0007) statusword_.store(0x0023); // Switched on
            else if (cw == 0x000F) statusword_.store(0x0027); // Operation enabled
            else if (cw == 0x0000) statusword_.store(0x0040); // Switch on disabled
        }
        // controlword khác: giữ nguyên state
    }

private:
    void reply_upload(uint16_t index, uint8_t subindex) {
        uint8_t data[4] = {0};
        size_t n = 0;
        bool found = true;
        switch (index) {
            case 0x1019:
                if (!supports_1019_) { found = false; break; }
                data[0] = nmt_operational_ ? 0x05 : 0x7F;
                n = 1;
                break;
            case 0x6041: {
                const uint16_t cur = statusword_.load();
                const uint16_t sw = faulted_ ? uint16_t(0x0007)
                                             : (cur ? cur : uint16_t(0x0001));
                data[0] = sw & 0xFF;
                data[1] = (sw >> 8) & 0xFF;
                n = 2;
                break;
            }
            case 0x6060:
            case 0x6061: data[0] = static_cast<uint8_t>(mode_); n = 1; break;
            case 0x606C:
                data[0] = actual_velocity_ & 0xFF;
                data[1] = (actual_velocity_ >> 8) & 0xFF;
                n = 2;
                break;
            case 0x60FF: {
                const uint32_t v = static_cast<uint32_t>(target_velocity_);
                data[0] = v & 0xFF; data[1] = (v >> 8) & 0xFF;
                data[2] = (v >> 16) & 0xFF; data[3] = (v >> 24) & 0xFF;
                n = 4;
                break;
            }
            default: found = false;
        }
        if (!found) {
            const uint8_t abort[8] = {0x80, (uint8_t)(index & 0xFF),
                                      (uint8_t)(index >> 8), subindex,
                                      0x00, 0x00, 0x06, 0x00};
            send(abort, 8);
            return;
        }
        // Expedited upload response (ccs=0b010, e=1, s=0b01, n = số byte dư):
        //   4 byte -> 0x41   2 byte -> 0x49   1 byte -> 0x4F
        const uint8_t scs = (n == 4) ? 0x4C : (n == 2 ? 0x4E : 0x4F);
        uint8_t frame[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        frame[0] = scs;
        frame[1] = index & 0xFF;
        frame[2] = (index >> 8) & 0xFF;
        frame[3] = subindex;
        for (size_t i = 0; i < n; ++i) frame[4 + i] = data[i];
        send(frame, 8);
    }

    void handle_sdo(const CANFrame& f) {
        const uint8_t cmd = f.get_u8(0);
        const uint16_t index = f.get_u16_le(1);
        const uint8_t subindex = f.get_u8(3);
        sdo_writes_++;

        if (cmd == 0x40) {  // upload request
            reply_upload(index, subindex);
            return;
        }
        if ((cmd & 0xE0) != 0x20) return;

        // download: expedited nếu có bit 1
        if (cmd & 0x02) {
            const size_t n = 4 - (cmd & 0x03);   // CiA 301: n ở bit1-0
            uint8_t data[4] = {f.get_u8(4), f.get_u8(5), f.get_u8(6), f.get_u8(7)};
            switch (index) {
                case 0x6040:
                    controlword_.store(static_cast<uint16_t>(data[0] | (data[1] << 8)));
                    update_state_from_controlword();
                    break;
                case 0x6060:
                    mode_ = static_cast<int8_t>(data[0]);
                    break;
                case 0x6061:
                    break;  // read-only
                case 0x60FF: {
                    // 0x60FF trong EDS là INTEGER16 → phải sign-extend khi
                    // expedited gửi 2 byte, nếu không giá trị âm sẽ thành dương
                    int32_t v = 0;
                    if (n == 2) {
                        const int16_t s = static_cast<int16_t>(data[0] | (data[1] << 8));
                        v = s;
                    } else {
                        uint32_t u = 0;
                        for (size_t i = 0; i < n && i < 4; ++i) {
                            u |= static_cast<uint32_t>(data[i]) << (8 * i);
                        }
                        v = static_cast<int32_t>(u);
                    }
                    target_velocity_.store(v);
                    actual_velocity_.store(v);
                    break;
                }
                default:
                    break;
            }
        }
        // Download response phải ECHO lại index/subindex, nếu không client
        // sẽ không nhận ra response của mình
        const uint8_t ok[8] = {0x60, static_cast<uint8_t>(index & 0xFF),
                               static_cast<uint8_t>((index >> 8) & 0xFF),
                               subindex, 0, 0, 0, 0};
        send(ok, 8);
    }

    void send(const uint8_t* data, size_t len) {
        CANFrame r;
        r.set_id(0x580u + node_);
        r.set_len(static_cast<uint8_t>(len));
        for (size_t i = 0; i < len && i < 8; ++i) r.set_u8(static_cast<uint8_t>(i), data[i]);
        bus_->dispatch(r);
    }

    uint8_t node_;
    BusInterface::RouteHandle route_{0};
    BusInterface* bus_{nullptr};
    std::atomic<uint16_t> controlword_{0};
    std::atomic<uint16_t> statusword_{0};
    std::atomic<int32_t> target_velocity_{0};
    std::atomic<int32_t> actual_velocity_{0};
    std::atomic<int8_t> mode_{0};
    std::atomic<uint32_t> sdo_writes_{0};
    bool nmt_operational_{false};
    bool supports_1019_{true};
    bool zlac_style_{false};
    bool faulted_{false};
};

} // namespace

int main() {
    std::cout << "=== MotorDevice Tests (drive CiA 402 ảo) ===\n\n";

    test::FakeBus bus;
    VirtualCiA402Drive drive(1);

    // Dựng hồ sơ tay — mô phỏng trường hợp không có file EDS
    DeviceProfile profile;
    {
        ObjectDictionary od(1);
        auto add = [&od](uint16_t idx, DataType dt, AccessType acc,
                         const char* name) {
            od.add_object(ObjectEntryBuilder()
                              .set_index(idx).set_subindex(0)
                              .set_data_type(dt).set_access(acc)
                              .set_name(name).build());
        };
        add(0x6040, DataType::UNSIGNED16, AccessType::RW, "controlword");
        add(0x6041, DataType::UNSIGNED16, AccessType::RO, "statusword");
        add(0x6060, DataType::INTEGER8,  AccessType::RW, "modes_of_operation");
        add(0x6061, DataType::INTEGER8,  AccessType::RO, "modes_of_operation_display");
        add(0x606C, DataType::INTEGER16, AccessType::RO, "velocity_actual_value");
        add(0x60FF, DataType::INTEGER16, AccessType::RW, "target_velocity");
        profile = DeviceProfile::from_dictionary(od, 1);
    }

    CHECK(profile.is_usable(), "hồ sơ dựng tay nhận diện đủ 4 object bắt buộc");
    CHECK(profile.resolve(ObjectRole::TargetVelocity).subindex == 0,
          "target_velocity ở subindex 0 (drive 1 trục)");

    MotorDevice dev(bus, profile);
    dev.sdo().set_timeout(200);
    dev.logger = [](const std::string& m) { std::cout << "    [log] " << m << "\n"; };
    bus.set_echo(true);
    drive.attach(bus);

    // =====================================================================
    std::cout << "\n--- connect(): NMT + enable theo CiA 402 ---\n";
    // =====================================================================
    {
        // Drive chưa operational → connect phải thử NMT Start
        CHECK(!dev.is_connected(), "chưa connected trước khi gọi connect()");

        // Làm drive phản ứng với NMT Start: coi mọi frame 0x000 là Start
        bus.add_route(0x000, 0x7FF, [&](const CANFrame&) {
            if (drive.nmt_operational()) return;
            drive.set_nmt_operational(true);
        });

        const bool ok = dev.connect(3000);
        CHECK(ok, "connect() thành công");
        CHECK(dev.is_connected(), "is_connected() = true");
        CHECK(dev.is_operational(), "is_operational() = true");
        CHECK(drive.statusword() == 0x0027,
              "drive đạt OPERATION_ENABLED (statusword thô 0x0027)");
        CHECK(drive.controlword() == 0x000F, "controlword cuối = 0x000F (Enable Operation)");
    }

    // =====================================================================
    std::cout << "\n--- set_operation_mode() ---\n";
    // =====================================================================
    {
        CHECK(dev.set_operation_mode(OperationMode::PROFILED_VELOCITY),
              "chuyển sang PROFILE_VELOCITY thành công");
        CHECK(drive.mode() == 3, "drive nhận 0x6060 = 3");
        CHECK(static_cast<int>(dev.operation_mode()) == 3,
              "operation_mode() phản ánh 3");
    }

    // =====================================================================
    std::cout << "\n--- set_velocity() / velocity() ---\n";
    // =====================================================================
    {
        CHECK(dev.set_velocity(500), "set_velocity(500)");
        CHECK(drive.target_velocity() == 500, "drive nhận target = 500");
        CHECK(dev.velocity() == 500, "đọc lại actual = 500");

        CHECK(dev.set_velocity(-250), "set_velocity(-250)");
        CHECK(drive.target_velocity() == -250, "giá trị âm đóng gói đúng (không bị coi là uint)");

        CHECK(dev.set_velocity(0), "set_velocity(0)");
        CHECK(drive.target_velocity() == 0, "dừng được");
        CHECK(dev.command_count() == 3, "đếm đủ 3 lệnh");
    }

    // =====================================================================
    std::cout << "\n--- stop() ---\n";
    // =====================================================================
    {
        dev.set_velocity(400);
        CHECK(dev.stop(), "stop() thành công");
        CHECK(drive.target_velocity() == 0, "stop() đặt target = 0");
    }

    // =====================================================================
    std::cout << "\n--- Hồ sơ thiếu object: connect() phải từ chối ---\n";
    // =====================================================================
    {
        ObjectDictionary empty_od(1);
        DeviceProfile bad = DeviceProfile::from_dictionary(empty_od, 1);
        MotorDevice bad_dev(bus, bad);
        bad_dev.sdo().set_timeout(100);
        CHECK(!bad_dev.is_usable(), "hồ sơ rỗng → is_usable() = false");
        CHECK(!bad_dev.connect(500), "connect() từ chối hồ sơ thiếu object");
    }

    // =====================================================================
    std::cout << "\n--- Thiết bị KHÔNG có 0x1019 (như ZLAC8015D) ---\n";
    // =====================================================================
    // CiA 301 bắt buộc object 0x1019, nhưng ZLAC8015D không khai báo và trả
    // abort. connect() phải vẫn chạy được thay vì phụ thuộc cứng vào 0x1019.
    {
        test::FakeBus bus2;
        VirtualCiA402Drive drive2(1);
        drive2.set_supports_1019(false);
        bus2.set_echo(true);
        drive2.attach(bus2);
        bus2.add_route(0x000, 0x7FF, [&](const CANFrame&) {
            drive2.set_nmt_operational(true);
        });

        DeviceProfile p2;
        {
            ObjectDictionary od2(1);
            auto add2 = [&od2](uint16_t idx, DataType dt, AccessType acc,
                               const char* name) {
                od2.add_object(ObjectEntryBuilder()
                                   .set_index(idx).set_subindex(0)
                                   .set_data_type(dt).set_access(acc)
                                   .set_name(name).build());
            };
            add2(0x6040, DataType::UNSIGNED16, AccessType::RW, "controlword");
            add2(0x6041, DataType::UNSIGNED16, AccessType::RO, "statusword");
            add2(0x6060, DataType::INTEGER8, AccessType::RW, "modes_of_operation");
            add2(0x60FF, DataType::INTEGER16, AccessType::RW, "target_velocity");
            p2 = DeviceProfile::from_dictionary(od2, 1);
        }
        CHECK(!p2.eds_has(0x1019, 0x00), "EDS không khai 0x1019 (mô phỏng ZLAC)");

        MotorDevice dev2(bus2, p2);
        dev2.sdo().set_timeout(200);
        const bool ok2 = dev2.connect(3000);
        CHECK(ok2, "connect() vẫn thành công khi thiếu 0x1019");
        CHECK(drive2.statusword() == 0x0027,
              "vẫn đạt OPERATION_ENABLED (thiếu 0x1019 không ảnh hưởng)");
        CHECK(dev2.set_velocity(300) && drive2.target_velocity() == 300,
              "điều khiển tốc độ vẫn chạy");
        dev2.disconnect();
        drive2.detach(bus2);
    }

    // =====================================================================
    std::cout << "\n--- Statusword kiểu ZLAC (0x1421/0x1C27) phải giải mã đúng ---\n";
    // =====================================================================
    // CiA 402 lấy state từ statusword & 0x4F. ZLAC báo 0x1421 cho "ready to
    // switch on" và 0x1C27 cho "operation enabled" — dùng statusword thô làm
    // state sẽ sai hoàn toàn.
    {
        test::FakeBus bus3;
        VirtualCiA402Drive drive3(1);
        drive3.set_zlac_style_statusword(true);
        bus3.set_echo(true);
        drive3.attach(bus3);
        bus3.add_route(0x000, 0x7FF,
                       [&](const CANFrame&) { drive3.set_nmt_operational(true); });

        DeviceProfile p3;
        {
            ObjectDictionary od3(1);
            auto add3 = [&od3](uint16_t idx, DataType dt, AccessType acc,
                               const char* name) {
                od3.add_object(ObjectEntryBuilder()
                                   .set_index(idx).set_subindex(0)
                                   .set_data_type(dt).set_access(acc)
                                   .set_name(name).build());
            };
            add3(0x6040, DataType::UNSIGNED16, AccessType::RW, "controlword");
            add3(0x6041, DataType::UNSIGNED16, AccessType::RO, "statusword");
            add3(0x6060, DataType::INTEGER8, AccessType::RW, "modes_of_operation");
            add3(0x60FF, DataType::INTEGER16, AccessType::RW, "target_velocity");
            p3 = DeviceProfile::from_dictionary(od3, 1);
        }

        MotorDevice dev3(bus3, p3);
        dev3.sdo().set_timeout(200);
        CHECK(dev3.connect(3000), "connect() thành công với statusword kiểu ZLAC");
        CHECK(dev3.cia402_state() == CiA402State::OPERATION_ENABLED,
              "0x1C27 & 0x4F = 0x07 → OPERATION_ENABLED");
        CHECK(dev3.statusword() == 0x1C27, "đọc được statusword thô 0x1C27");
        dev3.disconnect();
        drive3.detach(bus3);
    }

    // =====================================================================
    std::cout << "\n--- Thông tin hồ sơ ---\n";
    // =====================================================================
    std::cout << profile.describe();

    drive.detach(bus);
    std::cout << "\n=== Results: " << tests_passed << "/" << tests_total
              << " passed ===\n";
    return tests_passed == tests_total ? 0 : 1;
}
