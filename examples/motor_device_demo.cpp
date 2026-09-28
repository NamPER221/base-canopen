/**
 * @file motor_device_demo.cpp
 * @brief Demo lớp cấp cao MotorDevice — chứng minh "gọi hàm là chạy"
 *
 * Chỉ cần trỏ tới file EDS, không cần biết thiết bị dùng object nào, bao
 * nhiêu byte, có dấu hay không, và enable theo thứ tự nào.
 *
 * @code
 *   ./motor_device_demo can0 1 ZLAC8015D.eds
 * @endcode
 *
 * Với ZLAC8015D, target velocity nằm ở 0x60FF:03 (không phải :00 như chuẩn
 * CiA 402) nên cần override — đây là phần "khác biệt" duy nhất.
 */

#include <canopen/can/raw/socket_can_bus.hpp>
#include <canopen/device/motor_device.hpp>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace canopen;
using namespace std::chrono_literals;

namespace {

void wait_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void nmt_start(SocketCanBus& bus, uint8_t node) {
    CANFrame f;
    f.set_id(0x000);
    f.set_len(2);
    f.set_u8(0, 0x01);
    f.set_u8(1, node);
    bus.send(f);
}

bool file_exists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

/**
 * @brief Tìm file EDS khi người dùng chỉ đưa tên file
 *
 * Chạy từ thư mục build/ thì "ZLAC8015D.eds" không tồn tại — file nằm ở gốc
 * repo. Thử lần lượt các vị trí thường gặp rồi mới báo lỗi.
 */
std::string resolve_eds(const std::string& given) {
    if (file_exists(given)) return given;

    const char* base_name = given.c_str();
    const size_t slash = given.find_last_of('/');
    if (slash != std::string::npos) base_name = given.c_str() + slash + 1;

    const std::vector<std::string> candidates = {
        std::string("../") + base_name,
        std::string("../../") + base_name,
        std::string("ZLAC8015D.eds"),
        std::string("../ZLAC8015D.eds"),
#ifdef CANOPEN_DEFAULT_EDS
        std::string(CANOPEN_DEFAULT_EDS),
#endif
    };
    for (const auto& c : candidates) {
        if (file_exists(c)) return c;
    }
    return given;  // không tìm thấy — báo lỗi bên dưới
}

} // namespace

int main(int argc, char* argv[]) {
    std::string iface = "can0";
    uint8_t node = 1;
    std::string eds_path = "ZLAC8015D.eds";
    int rpm = 200;          // giá trị thử nghiệm
    int hold_ms = 1500;     // thời gian giữ tốc độ

    bool verbose_sdo = false;
    bool probe_cw = false;
    int pos = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-v" || a == "--verbose") { verbose_sdo = true; continue; }
        if (a == "--probe-cw") { probe_cw = true; continue; }
        if (a == "-h" || a == "--help") {
            std::cout << "Usage: " << argv[0] << " [interface] [node] [eds] [rpm] [hold_ms] [-v]\n"
                      << "  vd: can0 1 ZLAC8015D.eds 200 1500\n"
                      << "  -v         : trace từng trao đổi SDO\n"
                      << "  --probe-cw : thử controlword 2 byte vs 4 byte\n";
            return 0;
        }
        if (!a.empty() && a[0] == '-') continue;
        switch (pos++) {
            case 0: iface = a; break;
            case 1: node = static_cast<uint8_t>(std::stoi(a)); break;
            case 2: eds_path = a; break;
            case 3: rpm = std::stoi(a); break;
            case 4: hold_ms = std::stoi(a); break;
        }
    }

    std::cout << "=== MotorDevice Demo ===\n"
              << "lib: " << canopen_version() << " build: " << canopen_build_stamp()
              << "\n"
              << "bus=" << iface << "  node=" << static_cast<int>(node)
              << "  eds=" << eds_path << "  rpm=" << rpm << "\n\n";

    // ============ Bước 1: nạp EDS, tự động dò object ============
    const std::string eds_full = resolve_eds(eds_path);
    if (eds_full != eds_path) {
        std::cout << "  (EDS: \"" << eds_path << "\" không có ở thư mục hiện tại"
                  << " → dùng \"" << eds_full << "\")\n";
    }
    std::cout << "--- 1. Nạp EDS và tự động dò vai trò object ---\n";
    DeviceProfile profile = DeviceProfile::from_eds(eds_full, node);
    if (!profile.is_usable()) {
        std::cerr << "ERROR: EDS thiếu object bắt buộc\n"
                  << profile.describe() << "\n";
        return 1;
    }
    std::cout << profile.describe();

    // ZLAC8015D đặt target velocity ở subindex 3 (giá trị 32-bit gộp 2 trục)
    // thay vì subindex 0 như CiA 402 quy định → override 1 dòng.
    profile.set_override(ObjectRole::TargetVelocity, 0x60FF, 0x03);
    profile.set_override(ObjectRole::ActualVelocity, 0x606C, 0x01);
    std::cout << "\n  (override: target_velocity → 0x60FF:03 cho ZLAC)\n";

    // ============ Bước 2: mở bus + tạo MotorDevice ============
    std::cout << "\n--- 2. Mở bus và tạo MotorDevice ---\n";
    SocketCanBus bus(iface);
    if (bus.open() < 0 || !bus.is_up()) {
        std::cerr << "ERROR: không mở được " << iface
                  << " (cần: sudo ip link set " << iface
                  << " up type can bitrate 500000)\n";
        return 1;
    }

    MotorDevice dev(bus, profile);
    dev.set_sdo_timeout(200);
    dev.sdo().set_verbose(verbose_sdo);
    dev.logger = [](const std::string& m) { std::cout << "  [driver] " << m << "\n"; };

    // ============ Bước 2b: thử kích thước controlword nào drive chấp nhận ======
    // Quan sát: ghi 4 byte → không abort, nhưng lần đọc NGAY SAU đó bị
    // abort 0x08000021. Ghi 2 byte → drive abort 0x06070010 (length mismatch)
    // nhưng lần đọc sau lại bình thường. Thử cả hai để xác định cái nào
    // thực sự làm drive chuyển trạng thái.
    if (probe_cw) {
        std::cout << "\n--- 2b. Thử kích thước controlword ---\n";
        for (size_t cw_size : {size_t(2), size_t(4)}) {
            DeviceProfile p2 = DeviceProfile::from_eds(eds_full, node);
            p2.set_override(ObjectRole::TargetVelocity, 0x60FF, 0x03);
            if (cw_size != 2) p2.set_override(ObjectRole::Controlword, 0x6040, 0, cw_size);

            MotorDevice m2(bus, p2);
            m2.set_sdo_timeout(200);

            // NMT Start rồi chờ drive ổn định
            nmt_start(bus, node);
            std::this_thread::sleep_for(std::chrono::milliseconds(400));

            const uint16_t before = static_cast<uint16_t>(m2.statusword());
            m2.write_controlword(0x0006);
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
            const uint16_t after1 = static_cast<uint16_t>(m2.statusword());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            const uint16_t after2 = static_cast<uint16_t>(m2.statusword());

            char a1[8], a2[8];
            std::snprintf(a1, sizeof(a1), "%04X", before);
            std::snprintf(a2, sizeof(a2), "%04X", after1);
            std::cout << "  " << cw_size << " byte: 0x" << a1 << " → 0x" << a2
                      << " → 0x" << after2
                      << (after1 != before ? "   [ĐÃ CHUYỂN]" : "   [không đổi]")
                      << "\n";
            m2.disconnect();
        }
        std::cout << "\n  (kết quả trên cho biết kích thước nào drive thực sự "
                     "tiếp nhận)\n";
    }

    // ============ Bước 3: connect() — NMT + enable CiA 402 tự động ============
    std::cout << "\n--- 3. connect(): NMT Start + chuỗi enable CiA 402 ---\n";
    if (!dev.connect(4000)) {
        std::cerr << "ERROR: connect() thất bại\n";
        bus.close();
        return 1;
    }
    std::cout << "  state = 0x" << std::hex << dev.statusword() << std::dec
              << "  operational = " << (dev.is_operational() ? "YES" : "NO") << "\n";

    // ============ Bước 4: chế độ + profile ============
    std::cout << "\n--- 4. set_operation_mode + profile ---\n";
    const bool mode_ok = dev.set_operation_mode(OperationMode::PROFILED_VELOCITY);
    std::cout << "  set_operation_mode(PROFILED_VELOCITY) = "
              << (mode_ok ? "OK" : "FAIL") << "\n";

    dev.set_profile_velocity(1000);
    dev.set_profile_acceleration(800);
    dev.set_profile_deceleration(5000);
    std::cout << "  profile: velocity=1000 accel=800 decel=5000\n";

    // ============ Bước 5: chạy thử ============
    std::cout << "\n--- 5. set_velocity(" << rpm << ") — không biết index/kiểu dữ liệu ---\n";
    if (!dev.set_velocity(rpm)) {
        std::cerr << "ERROR: set_velocity thất bại\n";
        dev.disconnect();
        bus.close();
        return 1;
    }
    std::cout << "  đã gửi lệnh " << rpm << " RPM\n";

    // Theo dõi trong lúc chạy
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(hold_ms);
    while (std::chrono::steady_clock::now() < end) {
        wait_ms(250);
        std::cout << "  statusword=0x" << std::hex << dev.statusword() << std::dec
                  << "  actual=" << static_cast<int>(dev.velocity())
                  << "  lệnh đã gửi=" << dev.command_count() << "\n";
    }

    // ============ Bước 6: dừng an toàn ============
    std::cout << "\n--- 6. stop() + disconnect() ---\n";
    dev.stop();
    wait_ms(500);
    std::cout << "  sau stop: actual=" << static_cast<int>(dev.velocity()) << "\n";

    const uint32_t err = dev.error_code();
    if (err != 0) std::cout << "  CẢNH BÁO: mã lỗi drive = 0x" << std::hex << err << std::dec << "\n";

    dev.disconnect();
    bus.close();

    std::cout << "\n=== Hoàn tất ===\n";
    return 0;
}
