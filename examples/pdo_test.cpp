/**
 * @file pdo_test.cpp
 * @brief Kiểm tra RPDO của ZLAC8015D — phép thử sạch, đọc actual qua SDO.
 *
 * Kết luận từ các phiên thử trước:
 *   - RPDO mapping 1x32bit (0x1600:01 = 0x60FF0320), COB-ID 0x200+node
 *   - DLC phải KHỚP CHÍNH XÁC 4 byte, không đệm (DLC=8 bị bỏ qua)
 *   - Frame RPDO ghi được 0x60FF:03 (read-back đúng) nhưng chưa chứng minh
 *     motor chạy → cần đo actual velocity qua SDO để phân biệt
 *     "RPDO bị bỏ qua" với "RPDO nhận nhưng motor không đuổi kịp".
 *
 * TPDO có thể đã ngừng phát sau nhiều lần NMT Reset Comm, nên actual được
 * đọc bằng SDO 0x606C:01/:02 (đáng tin hơn TPDO trong phép thử này).
 */

#include <canopen/can/raw/socket_can_bus.hpp>
#include <canopen/co/cia402/cia402_drive.hpp>
#include <canopen/drivers/zlac8015/zlac8015_driver.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

using namespace canopen;
using namespace std::chrono_literals;

namespace {

SocketCanBus* g_bus = nullptr;
std::atomic<uint8_t> g_nmt_state{0xFF};
std::atomic<int> g_tpdo_count{0};

constexpr uint16_t TARGET_VELOCITY = 0x60FF;
constexpr uint16_t ACTUAL_VELOCITY = 0x606C;
constexpr uint16_t CONTROLWORD = 0x6040;

void wait_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

/** Gửi RPDO1 = 1x32bit, DLC đúng bằng mapping (4 byte, không đệm). */
void send_rpdo_velocity(uint32_t cobid, int16_t left, int16_t right, int frames = 10) {
    const uint32_t combined =
        static_cast<uint32_t>(static_cast<uint16_t>(left)) |
        (static_cast<uint32_t>(static_cast<uint16_t>(right)) << 16);
    for (int i = 0; i < frames; ++i) {
        CANFrame f;
        f.set_id(cobid);
        f.set_len(4);
        f.set_u32_le(0, combined);
        g_bus->send(f);
        wait_ms(30);
    }
}

uint32_t read_target(CiA402Drive& d) {
    uint32_t v = 0;
    d.sdo_read_u32(TARGET_VELOCITY, 0x03, v);
    return v;
}

int16_t read_actual_left(CiA402Drive& d) {
    int16_t v = 0;
    d.sdo_read_i16(ACTUAL_VELOCITY, 0x01, v);
    return v;
}

int16_t read_actual_right(CiA402Drive& d) {
    int16_t v = 0;
    d.sdo_read_i16(ACTUAL_VELOCITY, 0x02, v);
    return v;
}

uint16_t read_statusword(CiA402Drive& d) {
    uint16_t sw = 0;
    d.sdo_read_u16(0x6041, 0x00, sw);
    return sw;
}

/** Cấu hình lại RPDO1: 1x32bit, DLC=4, COB-ID = 0x200+node. */
bool setup_rpdo(CiA402Drive& d, uint8_t node, uint32_t& cobid_out) {
    const uint32_t cobid = 0x200u + node;
    bool ok = true;
    ok = d.sdo_write_u8(0x1600, 0x00, 0) && ok;                    // clear
    ok = d.sdo_write_u32(0x1600, 0x01, 0x60FF0320u) && ok;         // 0x60FF:03 32bit
    ok = d.sdo_write_u32(0x1400, 0x01, cobid) && ok;              // COB-ID
    ok = d.sdo_write_u8(0x1400, 0x02, 255) && ok;                  // type 254-255
    ok = d.sdo_write_u16(0x1400, 0x03, 0) && ok;                   // inhibit = 0
    ok = d.sdo_write_u8(0x1600, 0x00, 1) && ok;                    // START mapping

    uint8_t n = 0;
    uint32_t m0 = 0, cob = 0;
    d.sdo_read_u8(0x1600, 0x00, n);
    d.sdo_read_u32(0x1600, 0x01, m0);
    d.sdo_read_u32(0x1400, 0x01, cob);
    std::cout << "  mapping: n=" << static_cast<int>(n) << " m0=0x" << std::hex
              << m0 << " cobid=0x" << cob << std::dec
              << ((n == 1 && m0 == 0x60FF0320u && cob == cobid) ? "  [OK]" : "  [SAI]")
              << "\n";
    cobid_out = cob;
    return ok && n == 1 && m0 == 0x60FF0320u && cob == cobid;
}

void safe_stop(CiA402Drive& d, uint32_t cobid) {
    send_rpdo_velocity(cobid, 0, 0, 5);
    d.sdo_write_u32(TARGET_VELOCITY, 0x03, 0);
    d.sdo_write_u16(CONTROLWORD, 0x00, 0x0006);
    wait_ms(300);
    std::cout << "  → target=0, controlword=0x0006 (shutdown)\n";
}

} // namespace

int main(int argc, char* argv[]) {
    std::string iface = "can0";
    uint8_t node = 1;
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (!a.empty() && a[0] != '-') {
            if (positional == 0) iface = a;
            else if (positional == 1) node = static_cast<uint8_t>(std::stoi(a));
            ++positional;
        }
    }

    std::cout << "=== RPDO Velocity Verify (" << iface << ", node "
              << static_cast<int>(node) << ") ===\n\n";

    SocketCanBus bus(iface);
    if (bus.open() < 0 || !bus.is_up()) {
        std::cerr << "ERROR: cannot open " << iface << "\n";
        return 1;
    }
    g_bus = &bus;

    bus.add_route(0x700u + node, 0x7FF, [](const CANFrame& f) {
        g_nmt_state.store(f.get_u8(0));
    });
    bus.add_route(0x180u + node, 0x7FF, [](const CANFrame&) {
        g_tpdo_count.fetch_add(1);
    });

    drivers::ZLAC8015Driver driver(node, &bus);
    CiA402Drive& d = driver.drive();

    const uint32_t rpm = 50;
    const auto enc = static_cast<int16_t>(rpm);

    std::cout << "--- 1. Init: NMT reset → start → enable CiA402 ---\n";
    if (!driver.init()) {
        std::cerr << "ERROR: init thất bại\n";
        return 1;
    }

    std::cout << "\n--- 2. Mode profile velocity + profile ---\n";
    driver.set_operation_mode(3);
    driver.set_profile(1000, 800, 800);
    wait_ms(300);
    {
        uint8_t mode_disp = 0xFF;
        d.sdo_read_u8(0x6061, 0x00, mode_disp);
        std::cout << "  0x6061 (mode display) = "
                  << static_cast<int>(mode_disp) << "  (mong đợi 3)\n";
        std::cout << "  statusword = 0x" << std::hex << read_statusword(d)
                  << std::dec << "  (mong đợi 0x1C27 = operation enabled)\n";
    }

    std::cout << "\n--- 3. Cấu hình RPDO1 (1x32bit, DLC=4) ---\n";
    uint32_t cobid = 0;
    if (!setup_rpdo(d, node, cobid)) {
        std::cerr << "ERROR: cấu hình RPDO thất bại\n";
        safe_stop(d, cobid);
        return 1;
    }

    // ============ BƯỚC 4: BASELINE — SDO có làm motor chạy không? ============
    std::cout << "\n--- 4. BASELINE: SDO 0x60FF:03 (đo actual qua SDO) ---\n";
    d.sdo_write_u32(TARGET_VELOCITY, 0x03, 0);
    wait_ms(700);
    {
        const uint32_t t = read_target(d);
        const int16_t al = read_actual_left(d);
        const int16_t ar = read_actual_right(d);
        std::cout << "  target=0        actual=" << al << "/" << ar
                  << (al == 0 && ar == 0 ? "  [đứng, đúng]" : "  [CHƯA DỪNG]") << "\n";

        d.sdo_write_u32(TARGET_VELOCITY, 0x03,
                        static_cast<uint32_t>(static_cast<uint16_t>(enc)) |
                        (static_cast<uint32_t>(static_cast<uint16_t>(enc)) << 16));
        wait_ms(900);
        const uint32_t t2 = read_target(d);
        const int16_t al2 = read_actual_left(d);
        const int16_t ar2 = read_actual_right(d);
        std::cout << "  target=" << rpm << " RPM  actual=" << al2 << "/" << ar2
                  << ((al2 != 0 || ar2 != 0) ? "  [SDO CHẠY ✓]" : "  [SDO KHÔNG CHẠY ✗]")
                  << "   (target read-back 0x" << std::hex << t2 << std::dec << ")\n";
        (void)t;
    }

    // ============ BƯỚC 5: DỪNG, rồi thử RPDO với cùng giá trị ============
    std::cout << "\n--- 5. RPDO 0x" << std::hex << cobid << std::dec
              << " DLC=4, cùng giá trị " << rpm << " RPM ---\n";
    d.sdo_write_u32(TARGET_VELOCITY, 0x03, 0);
    wait_ms(800);
    {
        const int16_t al = read_actual_left(d);
        const int16_t ar = read_actual_right(d);
        std::cout << "  trước RPDO: target=0 actual=" << al << "/" << ar << "\n";
    }

    send_rpdo_velocity(cobid, enc, enc, 15);
    wait_ms(800);

    const uint32_t t_rpdo = read_target(d);
    const int16_t al_rpdo = read_actual_left(d);
    const int16_t ar_rpdo = read_actual_right(d);
    const bool accepted = t_rpdo != 0;
    const bool moving = (al_rpdo != 0 || ar_rpdo != 0);

    std::cout << "  sau RPDO:    target=0x" << std::hex << t_rpdo << std::dec
              << " actual=" << al_rpdo << "/" << ar_rpdo << "\n";
    std::cout << "  → frame RPDO " << (accepted ? "[ĐÃ GHI ĐƯỢC object]" : "[BỊ BỎ QUA]")
              << ", motor " << (moving ? "[CHẠY ✓]" : "[ĐỨNG ✗]") << "\n";

    // ============ BƯỚC 6: kết luận ============
    std::cout << "\n=== KẾT LUẬN ===\n";
    if (accepted && moving) {
        std::cout << "  ✅ RPDO HOẠT ĐỘNG HOÀN CHỈNH\n"
                  << "     mapping 0x1600:01=0x60FF0320 | COB-ID 0x200+node | DLC=4\n";
    } else if (accepted && !moving) {
        std::cout << "  ⚠ RPDO ghi được object nhưng motor không chạy\n"
                  << "     → cần thêm trigger (controlword trong RPDO) hoặc "
                     "kiểm tra mode/profile\n";
    } else {
        std::cout << "  ❌ RPDO bị drive bỏ qua hoàn toàn\n";
    }
    std::cout << "  NMT=0x" << std::hex << g_nmt_state.load() << std::dec
              << "  TPDO frames=" << g_tpdo_count.load()
              << "  (TPDO = 0 nghĩa là TPDO đã ngừng, actual đọc bằng SDO)\n";

    std::cout << "\n--- Dừng an toàn ---\n";
    safe_stop(d, cobid);

    std::cout << "\n=== Kết thúc ===\n";
    return 0;
}
