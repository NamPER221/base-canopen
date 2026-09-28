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
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

using namespace canopen;
using namespace std::chrono_literals;

namespace {

void wait_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

} // namespace

int main(int argc, char* argv[]) {
    std::string iface = "can0";
    uint8_t node = 1;
    std::string eds_path = "ZLAC8015D.eds";
    int rpm = 200;          // giá trị thử nghiệm
    int hold_ms = 1500;     // thời gian giữ tốc độ

    int pos = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            std::cout << "Usage: " << argv[0] << " [interface] [node] [eds] [rpm] [hold_ms]\n"
                      << "  vd: can0 1 ZLAC8015D.eds 200 1500\n";
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
              << "bus=" << iface << "  node=" << static_cast<int>(node)
              << "  eds=" << eds_path << "  rpm=" << rpm << "\n\n";

    // ============ Bước 1: nạp EDS, tự động dò object ============
    std::cout << "--- 1. Nạp EDS và tự động dò vai trò object ---\n";
    DeviceProfile profile = DeviceProfile::from_eds(eds_path, node);
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
    dev.sdo().set_timeout(200);
    dev.logger = [](const std::string& m) { std::cout << "  [driver] " << m << "\n"; };

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
