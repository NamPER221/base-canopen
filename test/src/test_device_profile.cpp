/**
 * @file test_device_profile.cpp
 * @brief Kiểm tra DeviceProfile tự động dò object CiA 402 từ EDS
 *
 * Dùng 2 tập dữ liệu:
 *   1. ZLAC8015D.eds thật — object kiểu MẢNG nhiều trục (0x60FF:00 là header,
 *      giá trị nằm ở :01/:02/:03) — trường hợp khó nhất
 *   2. EDS tổng hợp cho drive CiA 402 chuẩn một trục (0x60FF:00 = giá trị)
 */

#include <canopen/device/device_profile.hpp>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

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

/// EDS tối giản mô tả drive CiA 402 chuẩn MỘT TRỤC:
/// 0x60FF:00 và 0x606C:00 chứa giá trị trực tiếp (không phải header mảng)
const char* kStandardEds = R"(FileInfo)
FileName=standard.eds
FileVersion=1
DeviceInfo
VendorName=Generic
ProductName=StandardServo
DeviceName=StandardCiA402Drive
[1000]
ParameterName=Device type
ObjectType=0x7
DataType=0x0007
AccessType=ro
DefaultValue=0x00000000
PDOMapping=1
[6040]
ParameterName=Controlword
ObjectType=0x7
DataType=0x0006
AccessType=rw
DefaultValue=0x00000000
PDOMapping=1
[6041]
ParameterName=Statusword
ObjectType=0x7
DataType=0x0006
AccessType=ro
DefaultValue=0x00000000
PDOMapping=1
[6060]
ParameterName=Modes of operation
ObjectType=0x7
DataType=0x0002
AccessType=rw
DefaultValue=0
PDOMapping=1
[6061]
ParameterName=Modes of operation display
ObjectType=0x7
DataType=0x0002
AccessType=ro
DefaultValue=0
PDOMapping=1
[606C]
ParameterName=Velocity actual value
ObjectType=0x7
DataType=0x0003
AccessType=ro
DefaultValue=0x0000
PDOMapping=1
[60FF]
ParameterName=Target velocity
ObjectType=0x7
DataType=0x0003
AccessType=rw
DefaultValue=0x0000
PDOMapping=1
[607A]
ParameterName=Target position
ObjectType=0x7
DataType=0x0004
AccessType=rw
DefaultValue=0x00000000
PDOMapping=1
[6081]
ParameterName=Profile velocity
ObjectType=0x7
DataType=0x0007
AccessType=rw
DefaultValue=0x00000000
[6083]
ParameterName=Profile acceleration
ObjectType=0x7
DataType=0x0007
AccessType=rw
DefaultValue=0x00000000
[6084]
ParameterName=Profile deceleration
ObjectType=0x7
DataType=0x0007
AccessType=rw
DefaultValue=0x00000000
)";

bool write_temp(const char* path, const char* content) {
    std::ofstream f(path);
    if (!f) return false;
    f << content;
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    const std::string eds_dir = (argc > 1) ? argv[1] : ".";
    const std::string zlac_eds = eds_dir + "/ZLAC8015D.eds";
    const std::string std_eds = "/tmp/opencode_standard_device.eds";

    std::cout << "=== DeviceProfile Tests ===\n\n";

    // =====================================================================
    std::cout << "--- Drive CiA 402 chuẩn 1 trục (0x60FF:00 = giá trị) ---\n";
    // =====================================================================
    if (!write_temp(std_eds.c_str(), kStandardEds)) {
        std::cerr << "không ghi được " << std_eds << "\n";
        return 1;
    }
    {
        DeviceProfile p = DeviceProfile::from_eds(std_eds, 1);

        CHECK(p.is_usable(), "hồ sơ dùng được (đủ 4 object bắt buộc)");
        CHECK(p.missing_required().empty(), "không thiếu object bắt buộc");

        const ResolvedObject& cw = p.resolve(ObjectRole::Controlword);
        CHECK(cw.valid && cw.index == 0x6040 && cw.subindex == 0,
              "controlword = 0x6040:00");
        CHECK(cw.data_type == DataType::UNSIGNED16, "controlword UNSIGNED16");

        const ResolvedObject& tv = p.resolve(ObjectRole::TargetVelocity);
        CHECK(tv.valid && tv.index == 0x60FF && tv.subindex == 0,
              "target_velocity = 0x60FF:00 (subindex 0 là GIÁ TRỊ, không phải header)");
        CHECK(tv.data_type == DataType::INTEGER16, "target_velocity INTEGER16");
        CHECK(tv.is_signed, "target_velocity có dấu");
        CHECK(tv.size == 2, "target_velocity 2 byte");
        CHECK(tv.access == AccessType::RW, "target_velocity ghi được");

        const ResolvedObject& av = p.resolve(ObjectRole::ActualVelocity);
        CHECK(av.valid && av.index == 0x606C && av.subindex == 0,
              "actual_velocity = 0x606C:00");
        CHECK(av.access == AccessType::RO, "actual_velocity chỉ đọc");

        // Quy đổi giá trị: -1500 phải đóng gói đúng 2 byte little-endian
        uint8_t buf[4] = {0};
        size_t len = 0;
        const bool packed = tv.from_double(-1500.0, buf, len);
        CHECK(packed && len == 2, "pack -1500 → 2 byte");
        CHECK(len == 2 && buf[0] == 0x24 && buf[1] == 0xFA,
              "pack -1500 = 24 FA (0xFA24 little-endian)");
        double back = 0;
        CHECK(tv.to_double(buf, 2, back) && back == -1500.0,
              "unpack -1500 → đúng giá trị");

        // Profile velocity 32-bit
        const ResolvedObject& pv = p.resolve(ObjectRole::ProfileVelocity);
        CHECK(pv.valid && pv.size == 4, "profile_velocity 4 byte");
    }

    // =====================================================================
    std::cout << "\n--- ZLAC8015D thật (0x60FF:00 = HighestSubIndex header) ---\n";
    // =====================================================================
    if (std::ifstream(zlac_eds).good()) {
        DeviceProfile p = DeviceProfile::from_eds(zlac_eds, 1);

        CHECK(p.is_usable(), "ZLAC: hồ sơ dùng được");

        const ResolvedObject& tv = p.resolve(ObjectRole::TargetVelocity);
        CHECK(tv.valid && tv.index == 0x60FF,
              "ZLAC: target_velocity ở index 0x60FF");
        // Nếu logic nhầm coi :00 là giá trị thì sẽ ra subindex=0
        CHECK(tv.subindex != 0,
              "ZLAC: BỎ QUA header subindex 0 (HighestSubIndex)");
        CHECK(tv.subindex == 1, "ZLAC: target_velocity lấy subindex 1 (trục trái)");
        CHECK(tv.data_type == DataType::INTEGER32,
              "ZLAC: target_velocity INTEGER32 (4 byte, không phải 16 bit)");
        CHECK(tv.size == 4, "ZLAC: target_velocity 4 byte");

        const ResolvedObject& av = p.resolve(ObjectRole::ActualVelocity);
        CHECK(av.valid && av.subindex == 1, "ZLAC: actual_velocity subindex 1");
        // Lưu ý: EDS của ZLAC khai báo 0x606C:01 là 0x0007 (UNSIGNED32) dù
        // tốc độ có thể âm — lỗi của hãng. Thư viện đọc đúng theo EDS, nên
        // test khẳng định theo EDS chứ không "sửa" hộ.
        CHECK(av.data_type == DataType::UNSIGNED32,
              "ZLAC: actual_velocity theo EDS = UNSIGNED32 (EDS của hãng khai sai)");

        const ResolvedObject& m = p.resolve(ObjectRole::ModesOfOperation);
        CHECK(m.valid && m.index == 0x6060, "ZLAC: modes_of_operation 0x6060");
        CHECK(m.subindex == 0,
              "ZLAC: 0x6060 subindex 0 là giá trị (loại INTEGER8, không phải header)");

        // Đóng gói giá trị 32-bit
        uint8_t buf[4] = {0};
        size_t len = 0;
        CHECK(tv.from_double(-120, buf, len) && len == 4, "pack -120 → 4 byte");
        CHECK(len == 4 && buf[0] == 0x88 && buf[1] == 0xFF,
              "pack -120 = 88 FF FF FF");

        // Override: chỉ định lại sang subindex 3 (giá trị 32-bit kết hợp 2 trục)
        DeviceProfile q = DeviceProfile::from_eds(zlac_eds, 1);
        q.set_override(ObjectRole::TargetVelocity, 0x60FF, 3);
        const ResolvedObject& o = q.resolve(ObjectRole::TargetVelocity);
        CHECK(o.subindex == 3, "override target_velocity → subindex 3");
        CHECK(o.from_override, "đánh dấu là override");
        CHECK(o.data_type == DataType::UNSIGNED32,
              "override giữ data type của object (UNSIGNED32)");

        std::cout << "\n" << q.describe();
    } else {
        std::cout << "  (bỏ qua — không tìm thấy " << zlac_eds << ")\n";
        std::cout << "  chạy từ thư mục gốc repo: ./test/test_device_profile .\n";
    }

    // =====================================================================
    std::cout << "\n--- Byte order: little-endian vs big-endian ---\n";
    // =====================================================================
    {
        DeviceProfile le = DeviceProfile::from_eds(std_eds, 1);
        CHECK(le.byte_order() == ByteOrder::LittleEndian,
              "mặc định little-endian (chuẩn CiA 301)");

        // 16 bit: giá trị 0x1234
        const ResolvedObject& tv = le.resolve(ObjectRole::TargetVelocity);
        uint8_t buf[4] = {0};
        size_t len = 0;
        tv.from_double(0x1234, buf, len);
        CHECK(len == 2 && buf[0] == 0x34 && buf[1] == 0x12,
              "LE: 0x1234 → 34 12");
        double back = 0;
        tv.to_double(buf, 2, back);
        CHECK(back == 0x1234, "LE: đọc lại 0x1234 đúng");

        // Cùng giá trị, big-endian
        DeviceProfile be = DeviceProfile::from_eds(std_eds, 1);
        be.set_byte_order(ByteOrder::BigEndian);
        CHECK(std::string(be.byte_order_name()) == "big", "byte_order_name() = big");

        const ResolvedObject& tvbe = be.resolve(ObjectRole::TargetVelocity);
        CHECK(tvbe.byte_order == ByteOrder::BigEndian,
              "object nhận byte order từ profile");
        uint8_t bbuf[4] = {0};
        size_t blen = 0;
        tvbe.from_double(0x1234, bbuf, blen);
        CHECK(blen == 2 && bbuf[0] == 0x12 && bbuf[1] == 0x34,
              "BE: 0x1234 → 12 34 (MSB trước)");
        double bback = 0;
        tvbe.to_double(bbuf, 2, bback);
        CHECK(bback == 0x1234, "BE: đọc lại 0x1234 đúng");

        // 32 bit big-endian trên ZLAC (nếu có EDS)
        if (std::ifstream(zlac_eds).good()) {
            DeviceProfile z = DeviceProfile::from_eds(zlac_eds, 1);
            z.set_byte_order(ByteOrder::BigEndian);
            const ResolvedObject& zt = z.resolve(ObjectRole::TargetVelocity);
            uint8_t zbuf[4] = {0};
            size_t zlen = 0;
            zt.from_double(0x00010002, zbuf, zlen);
            CHECK(zlen == 4 && zbuf[0] == 0x00 && zbuf[1] == 0x01 &&
                  zbuf[2] == 0x00 && zbuf[3] == 0x02,
                  "ZLAC BE 32-bit: 0x00010002 → 00 01 00 02");
        }

        // Giá trị âm 32 bit: phải sign-extend đúng ở cả hai kiểu
        DeviceProfile n32 = DeviceProfile::from_eds(std_eds, 1);
        const ResolvedObject& pv = n32.resolve(ObjectRole::TargetPosition);
        CHECK(pv.valid && pv.size == 4 && pv.is_signed,
              "target_position 0x607A INTEGER32 (có dấu) để test sign-extend");
        uint8_t nb[4] = {0};
        size_t nl = 0;
        pv.from_double(-100000, nb, nl);
        double nback = 0;
        pv.to_double(nb, 4, nback);
        CHECK(nback == -100000, "LE 32-bit: -100000 qua lại đúng");

        DeviceProfile n32be = DeviceProfile::from_eds(std_eds, 1);
        n32be.set_byte_order(ByteOrder::BigEndian);
        const ResolvedObject& pvbe = n32be.resolve(ObjectRole::TargetPosition);
        uint8_t nbb[4] = {0};
        size_t nbl = 0;
        pvbe.from_double(-100000, nbb, nbl);
        CHECK(nbb[0] == 0xFF && nbb[1] == 0xFE && nbb[2] == 0x79 && nbb[3] == 0x60,
              "BE 32-bit: -100000 → FF FE 79 60");
        double nbb2 = 0;
        pvbe.to_double(nbb, 4, nbb2);
        CHECK(nbb2 == -100000, "BE 32-bit: -100000 qua lại đúng");
    }

    // =====================================================================
    std::cout << "\n--- EDS khai sai kích thước: phải dùng số byte thực tế ---\n";
    // =====================================================================
    {
        // ZLAC khai statusword là UNSIGNED32 (4 byte) nhưng một số drive
        // thực tế chỉ có 16 bit. Khi thiết bị trả về 2 byte, thư viện phải
        // đọc được thay vì báo lỗi.
        ResolvedObject o;
        o.valid = true;
        o.index = 0x6041;
        o.subindex = 0;
        o.data_type = DataType::UNSIGNED32;
        o.size = 4;
        o.is_signed = false;
        o.byte_order = ByteOrder::LittleEndian;

        uint8_t two[2] = {0x00, 0x14};   // 0x1400
        double v = 0;
        CHECK(o.to_double(two, 2, v), "đọc được khi EDS khai 4 byte, thiết bị trả 2");
        CHECK(v == 0x1400, "giá trị 0x1400 đúng (0x0014 little-endian)");

        uint8_t four[4] = {0x00, 0x14, 0x00, 0x00};
        CHECK(o.to_double(four, 4, v) && v == 0x1400, "đọc được khi thiết bị trả đủ 4 byte");

        uint8_t one[1] = {0x27};
        CHECK(o.to_double(one, 1, v) && v == 0x27, "đọc được cả khi chỉ 1 byte");

        uint8_t zero[1] = {0};
        CHECK(!o.to_double(zero, 0, v), "trả false khi không có byte nào");
    }

    // =====================================================================
    std::cout << "\n--- Thiếu object bắt buộc ---\n";
    // =====================================================================
    {
        const char* poor_eds = R"(FileInfo
FileName=poor.eds
[606C]
ParameterName=Velocity actual value
ObjectType=0x7
DataType=0x0003
AccessType=ro
PDOMapping=1
)";
        const std::string path = "/tmp/opencode_poor_device.eds";
        write_temp(path.c_str(), poor_eds);
        DeviceProfile p = DeviceProfile::from_eds(path, 1);

        CHECK(!p.is_usable(), "EDS thiếu object → không dùng được");
        const auto missing = p.missing_required();
        CHECK(missing.size() == 4, "báo thiếu cả 4 object bắt buộc");
        CHECK(!p.has(ObjectRole::TargetVelocity), "không tìm thấy target_velocity");
        CHECK(p.has(ObjectRole::ActualVelocity),
              "vẫn tìm thấy actual_velocity dù thiếu bắt buộc");
        std::cout << p.describe();
    }

    std::remove(std_eds.c_str());
    std::remove("/tmp/opencode_poor_device.eds");

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_total
              << " passed ===\n";
    return tests_passed == tests_total ? 0 : 1;
}
