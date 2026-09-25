/**
 * @file pdo_test.cpp
 * @brief Quyết định nhanh: RPDO velocity có được drive áp dụng không
 *
 * Chiến lược: gửi lệnh, rồi ĐỌC NGƯỢC 0x60FF:03 qua SDO.
 *   - Giá trị thay đổi  → drive ĐÃ nhận lệnh (vấn đề ở chỗ khác)
 *   - Giá trị không đổi → drive BỎ QUA frame đó
 *
 * Test tuần tự:
 *   A) SDO  0x60FF:03 = 50        (đường tham chiếu — chắc chắn chạy)
 *   B) RPDO 0x201 với mapping 1x32bit
 *   C) RPDO + SYNC frame 0x080 đi kèm (trường hợp drive ở chế độ sync)
 *   D) RPDO với mapping 2x16bit (0x60FF:01 + 0x60FF:02)
 *   E) RPDO sau khi lưu cấu hình vào EEPROM (0x2010:01 = 1)
 *
 * Usage: ./pdo_test [can_interface] [node_id]
 */

#include <canopen/drivers/zlac8015/zlac8015_driver.hpp>
#include <canopen/can/raw/socket_can_bus.hpp>
#include <canopen/can/msg/message_factory.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cstring>

using namespace canopen;
using namespace canopen::drivers;

static SocketCanBus* g_bus = nullptr;
static uint8_t g_node = 1;

static uint32_t read_target_via_sdo(CiA402Drive& drive) {
    uint32_t v = 0;
    drive.sdo_read_u32(0x60FF, 0x03, v);
    return v;
}

static void wait_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static void send_rpdo(uint32_t cobid, uint16_t left, uint16_t right, bool with_sync) {
    CANFrame f;
    f.set_id(cobid);
    f.set_len(4);
    const uint32_t combined = (static_cast<uint32_t>(left) & 0xFFFF) |
                              (static_cast<uint32_t>(right) << 16);
    f.set_u32_le(0, combined);

    if (with_sync) {
        // SYNC rồi RPDO ngay sau — drive ở chế độ sync cần SYNC trước
        CANFrame sync;
        sync.set_id(0x080);
        sync.set_len(0);
        g_bus->send(sync);
        wait_ms(5);
    }
    g_bus->send(f);
}

static void report(const char* name, uint32_t before, uint32_t after,
                   int16_t actual_l, int16_t actual_r) {
    const bool accepted = (after != before);
    const bool moving = (actual_l != 0 || actual_r != 0);
    std::cout << "  " << std::left << std::setw(34) << name
              << " 0x60FF:03 " << std::hex << before << "->" << after << std::dec
              << (accepted ? "  [DRIVE NHẬN]" : "  [BỎ QUA]    ")
              << " actual=" << actual_l << "/" << actual_r
              << (moving ? "  [MOTOR CHẠY]" : "  [đứng yên]") << "\n";
}

int main(int argc, char* argv[]) {
    std::string iface = "can0";
    uint8_t node = 1;
    int positional = 0;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a.size() > 0 && a[0] != '-') {
            if (positional == 0) iface = a;
            else if (positional == 1) node = static_cast<uint8_t>(std::stoi(a));
            positional++;
        }
    }
    g_node = node;

    std::cout << "=== PDO Velocity Test (" << iface << ", node " << static_cast<int>(node)
              << ") ===\n\n";

    SocketCanBus bus(iface);
    if (bus.open() < 0 || !bus.is_up()) {
        std::cerr << "ERROR: cannot open " << iface << " (errno "
                  << bus.socket().get_error_num() << ")\n";
        return 1;
    }
    g_bus = &bus;

    ZLAC8015Driver driver(node, &bus);
    driver.logger = [](const std::string& m) { std::cout << "  " << m << "\n"; };

    if (!driver.init()) {
        std::cerr << "ERROR: motor init failed\n";
        return 1;
    }

    // ---- Chuẩn bị: mode + profile velocity (KHÔNG được bỏ qua) ----
    // Trong Profile Velocity mode, nếu 0x6081 = 0 thì target bị clamp về 0 → đứng yên.
    driver.set_operation_mode(3);
    driver.set_profile(1000, 800, 800);

    std::cout << "\n--- Chuẩn bị xong, bắt đầu test ---\n\n";

    const uint16_t L = 50, R = 50;   // RPM
    uint32_t before = 0, after = 0;

    // ============ A) SDO — đường tham chiếu ============
    {
        CANFrame req = MessageFactory::create_sdo_download_request(
            node, 0x60FF, 0x03, nullptr, 0);
        req.set_len(8);
        req.set_u8(0, 0x23);
        req.set_u16_le(1, 0x60FF);
        req.set_u8(3, 0x03);
        req.set_u16_le(4, L);
        req.set_u16_le(6, R);
        bus.send(req);
        wait_ms(600);

        before = read_target_via_sdo(driver.drive());
        // đọc lại sau khi đã gán
        after = before;
        std::cout << "  " << std::left << std::setw(34) << "A) SDO 0x60FF:03 = 50"
                  << " 0x60FF:03 = 0x" << std::hex << after << std::dec
                  << "  actual=" << driver.velocity_actual_left() << "/"
                  << driver.velocity_actual_right()
                  << (driver.velocity_actual_left() != 0 ? "  [MOTOR CHẠY]" : "  [đứng yên]")
                  << "\n\n";
        driver.stop();
        wait_ms(500);
    }

    // ============ B) RPDO mapping 1x32bit ============
    {
        // reset về 0
        CANFrame z;
        z.set_id(0x201); z.set_len(4); z.set_u32_le(0, 0);
        bus.send(z);
        wait_ms(400);

        before = read_target_via_sdo(driver.drive());
        send_rpdo(0x201u + node, L, R, false);
        wait_ms(600);
        after = read_target_via_sdo(driver.drive());
        report("B) RPDO 0x201 (mapping 1x32bit)", before, after,
               driver.velocity_actual_left(), driver.velocity_actual_right());
        driver.stop();
        wait_ms(400);
    }

    // ============ C) RPDO + SYNC ============
    {
        CANFrame z;
        z.set_id(0x201); z.set_len(4); z.set_u32_le(0, 0);
        bus.send(z);
        wait_ms(400);

        before = read_target_via_sdo(driver.drive());
        for (int i = 0; i < 5; ++i) {          // gửi lặp để chắc chắn
            send_rpdo(0x201u + node, L, R, true);
            wait_ms(100);
        }
        wait_ms(300);
        after = read_target_via_sdo(driver.drive());
        report("C) RPDO 0x201 + SYNC 0x080", before, after,
               driver.velocity_actual_left(), driver.velocity_actual_right());
        driver.stop();
        wait_ms(400);
    }

    // ============ D) RPDO mapping 2x16bit ============
    {
        // Đổi mapping sang 2 entry 16-bit
        driver.drive().sdo_write_u32(0x1400, 0x01, (0x200u + node) | 0x80000000u);
        driver.drive().sdo_write_u8(0x1600, 0x00, 2);
        driver.drive().sdo_write_u32(0x1600, 0x01, 0x60FF0110u);
        driver.drive().sdo_write_u32(0x1600, 0x02, 0x60FF0220u);
        driver.drive().sdo_write_u8(0x1400, 0x02, 255);
        driver.drive().sdo_write_u16(0x1400, 0x03, 0);
        driver.drive().sdo_write_u32(0x1400, 0x01, 0x200u + node);

        uint32_t m0 = 0, m1 = 0;
        driver.drive().sdo_read_u32(0x1600, 0x01, m0);
        driver.drive().sdo_read_u32(0x1600, 0x02, m1);
        std::cout << "  (2x16bit mapping: 0x" << std::hex << m0 << " / 0x" << m1
                  << std::dec << ")\n";

        // reset
        CANFrame z;
        z.set_id(0x201); z.set_len(4); z.set_u32_le(0, 0);
        bus.send(z);
        wait_ms(400);

        before = read_target_via_sdo(driver.drive());
        // left ở bytes 0-1, right ở bytes 2-3
        send_rpdo(0x201u + node, L, R, false);
        wait_ms(600);
        after = read_target_via_sdo(driver.drive());
        report("D) RPDO 0x201 (mapping 2x16bit)", before, after,
               driver.velocity_actual_left(), driver.velocity_actual_right());
        driver.stop();
        wait_ms(400);
    }

    // ============ E) Lưu EEPROM rồi thử lại ============
    {
        std::cout << "  (ghi cấu hình vào EEPROM: 0x2010:01 = 1 ...)\n";
        driver.drive().sdo_write_u8(0x2010, 0x01, 1);
        wait_ms(800);

        before = read_target_via_sdo(driver.drive());
        send_rpdo(0x201u + node, L, R, false);
        wait_ms(600);
        after = read_target_via_sdo(driver.drive());
        report("E) RPDO sau khi lưu EEPROM", before, after,
               driver.velocity_actual_left(), driver.velocity_actual_right());
        driver.stop();
    }

    std::cout << "\n=== Kết thúc ===\n";
    return 0;
}
