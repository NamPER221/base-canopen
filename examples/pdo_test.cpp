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
#include <atomic>

using namespace canopen;
using namespace canopen::drivers;

// Theo dõi NMT state qua heartbeat (0x700 + node)
static std::atomic<int> g_nmt_state{-1};
static std::atomic<uint32_t> g_tpdo_count{0};
static std::atomic<int32_t> g_tpdo_l{0};
static std::atomic<int32_t> g_tpdo_r{0};

static const char* nmt_name(int s) {
    switch (s) {
        case 0x00: return "Initialising";
        case 0x04: return "STOPPED";
        case 0x05: return "OPERATIONAL";
        case 0x7F: return "Pre-operational";
        default: return "unknown";
    }
}


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

// Đưa drive về Operational một cách CHẮC CHẮN:
//   1. NMT Reset Communication (0x82) → drive khởi động lại
//   2. CHỜ heartbeat báo đã lên Pre-operational (0x7F) — KHÔNG đoán bằng sleep
//   3. Gửi NMT Start (0x01)
//   4. CHỜ heartbeat báo Operational (0x05)
// Nếu gửi Start quá sớm (drive còn đang boot) thì lệnh bị bỏ qua và
// drive kẹt ở Pre-operational → PDO không được áp dụng.
static bool bring_operational(uint8_t node) {
    CANFrame nmt;
    nmt.set_id(0x000);
    nmt.set_len(2);
    nmt.set_u8(1, node);

    // 1. Reset Communication
    nmt.set_u8(0, 0x82);
    g_bus->send(nmt);
    g_nmt_state.store(-1);   // reset cache

    // 2. Chờ drive khởi động xong (bootup 0x00 → pre-op 0x7F)
    for (int i = 0; i < 40; ++i) {
        wait_ms(100);
        const int st = g_nmt_state.load();
        if (st == 0x7F || st == 0x05) break;
    }
    std::cout << "      sau Reset Comm: NMT=0x" << std::hex
              << (g_nmt_state.load() < 0 ? -1 : g_nmt_state.load()) << std::dec
              << " (" << nmt_name(g_nmt_state.load()) << ")\n";

    // 3. Start
    nmt.set_u8(0, 0x01);
    g_bus->send(nmt);

    // 4. Chờ Operational
    for (int i = 0; i < 20; ++i) {
        wait_ms(100);
        if (g_nmt_state.load() == 0x05) {
            std::cout << "      sau NMT Start: NMT=0x05 (OPERATIONAL)\n";
            return true;
        }
    }
    std::cout << "      sau NMT Start: NMT=0x" << std::hex
              << (g_nmt_state.load() < 0 ? -1 : g_nmt_state.load()) << std::dec
              << " — KHÔNG đạt Operational\n";
    return false;
}

static bool drive_alive(CiA402Drive& d) {
    uint32_t v = 0;
    return d.sdo_read_u32(0x6041, 0x00, v);
}



static void send_rpdo(uint32_t cobid, uint16_t left, uint16_t right, bool with_sync) {
    CANFrame f;
    f.set_id(cobid);
    f.set_len(8);   // ★ DLC = 8 (đệm 0) — firmware ZLAC yêu cầu đủ 8 byte ★
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
              << (accepted ? "  [NHẬN] " : "  [BỎ QUA] ")
              << "actual=" << actual_l << "/" << actual_r
              << (moving ? " [CHẠY]" : " [đứng]")
              << "  NMT=0x" << std::hex << g_nmt_state.load() << std::dec
              << " TPDO_n=" << g_tpdo_count.load() << "\n";
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

    // Theo dõi heartbeat (NMT state) + TPDO — để biết drive thật sự ở
    // trạng thái nào và có gửi PDO không
    bus.add_route(0x700u + node, 0x7FF, [](const CANFrame& f) {
        g_nmt_state.store(f.get_u8(0));
    });
    bus.add_route(0x180u + node, 0x7FF, [](const CANFrame& f) {
        g_tpdo_count.fetch_add(1);
        if (f.len() >= 8) {
            g_tpdo_l.store(static_cast<int32_t>(f.get_u32_le(0)));
            g_tpdo_r.store(static_cast<int32_t>(f.get_u32_le(4)));
        }
    });

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

    std::cout << "\n--- Chuẩn bị xong, bắt đầu test ---\n";

    // ============ 0) Kiểm tra NMT state — ĐIỀU KIỆN TIÊN QUYẾT cho PDO ============
    // Theo tài liệu ZLAC: PDO CHỈ hoạt động khi NMT state = 0x05 (Operational).
    // Pre-operational (0x7F) hoặc Stopped (0x04) → chỉ nhận SDO.
    std::cout << "  [0] NMT state hiện tại: ";
    if (g_nmt_state.load() < 0) {
        std::cout << "KHÔNG nhận heartbeat\n";
    } else {
        std::cout << "0x" << std::hex << g_nmt_state.load() << std::dec
                  << " (" << nmt_name(g_nmt_state.load()) << ")\n";
    }

    if (g_nmt_state.load() != 0x05) {
        std::cout << "      -> Gửi NMT Start (0x000 01 01) và chờ...\n";
        CANFrame nmt;
        nmt.set_id(0x000);
        nmt.set_len(2);
        nmt.set_u8(0, 0x01);
        nmt.set_u8(1, node);
        bus.send(nmt);
        wait_ms(600);
        std::cout << "      -> NMT state sau khi Start: ";
        if (g_nmt_state.load() < 0) {
            std::cout << "KHÔNG nhận heartbeat\n";
        } else {
            std::cout << "0x" << std::hex << g_nmt_state.load() << std::dec
                      << " (" << nmt_name(g_nmt_state.load()) << ")\n";
        }
    }

    if (g_nmt_state.load() != 0x05) {
        std::cout << "\n  *** KẾT LUẬN: drive KHÔNG ở Operational (0x05).\n"
                  << "      Theo tài liệu ZLAC, PDO bị bỏ qua ở state này.\n"
                  << "      -> RPDO sẽ không bao giờ hoạt động. Dùng SDO.\n\n";
    } else {
        std::cout << "      -> OK: Operational, PDO được phép hoạt động\n\n";
    }

    // ============ 0b) Watchdog 0x2000 — tắt để không cắt lệnh RPDO ============
    {
        uint16_t wd = 0;
        if (driver.drive().sdo_read_u16(0x2000, 0x00, wd)) {
            std::cout << "  [0b] Watchdog 0x2000 = " << wd << " ("
                      << (wd == 0 ? "tắt" : "ms") << ")\n";
            if (wd > 0 && wd < 5000) {
                driver.drive().sdo_write_u16(0x2000, 0x00, 5000);
                std::cout << "      -> đã tăng lên 5000ms (tránh cắt lệnh)\n";
            }
        }
        std::cout << "\n";
    }

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

    // ============ B) RPDO mapping 1x32bit (DLC=8) ============
    {
        bring_operational(node);
        // ★ NMT Reset Comm (0x82) XÓA cấu hình PDO → phải cấu hình lại ★
        driver.setup_pdo();
        std::cout << "  (B) sau reset + setup_pdo lại, drive alive: "
                  << (drive_alive(driver.drive()) ? "OK" : "KHÔNG") << "\n";

        before = read_target_via_sdo(driver.drive());
        send_rpdo(0x201u + node, L, R, false);
        wait_ms(600);
        after = read_target_via_sdo(driver.drive());
        const bool alive = drive_alive(driver.drive());
        if (!alive) std::cout << "  *** DRIVE ĐÃ TREO sau frame RPDO (DLC=8) ***\n";
        report("B) RPDO 0x201 (mapping 1x32bit)", before, after,
               driver.velocity_actual_left(), driver.velocity_actual_right());
        driver.stop();
        wait_ms(400);
    }

    // ============ C) RPDO + SYNC (DLC=8) ============
    {
        bring_operational(node);
        driver.setup_pdo();
        std::cout << "  (C) sau reset + setup_pdo lại, drive alive: "
                  << (drive_alive(driver.drive()) ? "OK" : "KHÔNG") << "\n";

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

    // ============ D) RPDO mapping 2x16bit (DLC=8) ============
    {
        bring_operational(node);
        // Đổi mapping sang 2 entry 16-bit — theo ĐÚNG thứ tự tài liệu ZLAC:
        //   clear → entry → COB-ID → type → inhibit → START
        driver.drive().sdo_write_u8(0x200F, 0x00, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 0);          // clear
        driver.drive().sdo_write_u32(0x1600, 0x01, 0x60FF0110u);
        driver.drive().sdo_write_u32(0x1600, 0x02, 0x60FF0220u);
        driver.drive().sdo_write_u32(0x1400, 0x01, 0x200u + node);
        driver.drive().sdo_write_u8(0x1400, 0x02, 255);
        driver.drive().sdo_write_u16(0x1400, 0x03, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 2);          // ★ START ★
        driver.use_pdo(true);
        std::cout << "  (D) sau reset + mapping 2x16bit, drive alive: "
                  << (drive_alive(driver.drive()) ? "OK" : "KHÔNG") << "\n";

        uint32_t m0 = 0, m1 = 0;
        driver.drive().sdo_read_u32(0x1600, 0x01, m0);
        driver.drive().sdo_read_u32(0x1600, 0x02, m1);
        std::cout << "  (2x16bit mapping: 0x" << std::hex << m0 << " / 0x" << m1
                  << std::dec << ")\n";

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
        bring_operational(node);
        driver.setup_pdo();
        std::cout << "  (E) sau reset + setup_pdo lại, drive alive: "
                  << (drive_alive(driver.drive()) ? "OK" : "KHÔNG") << "\n";
        std::cout << "  (ghi cấu hình vào EEPROM: 0x2010:00 = 2 ...)\n";
        driver.drive().sdo_write_u8(0x2010, 0x00, 2);
        wait_ms(800);
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


    // ============ F) Quét cả 4 nhóm RPDO (0x1400-0x1403) ============
    // Mỗi nhóm dùng COB-ID riêng: RPDO0=0x200, RPDO1=0x300, RPDO2=0x400, RPDO3=0x500
    {
        std::cout << "\n--- F) Quét 4 nhóm RPDO ---\n";
        struct RGroup { uint16_t comm; uint16_t map; uint32_t cobid; };
        const RGroup groups[] = {
            {0x1400, 0x1600, 0x200},
            {0x1401, 0x1601, 0x300},
            {0x1402, 0x1602, 0x400},
            {0x1403, 0x1603, 0x500},
        };

        for (const auto& g : groups) {
            const uint32_t cobid = g.cobid + node;

            // bring về operational + cấu hình lại RPDO của nhóm này
            bring_operational(node);
            driver.drive().sdo_write_u8(0x200F, 0x00, 0);
            // mapping: 0x60FF:01 (16 bit) + 0x60FF:02 (16 bit) — đơn giản nhất
            driver.drive().sdo_write_u8(g.map, 0x00, 0);                 // clear
            driver.drive().sdo_write_u32(g.map, 0x01, 0x60FF0110u);
            driver.drive().sdo_write_u32(g.map, 0x02, 0x60FF0220u);
            driver.drive().sdo_write_u32(g.comm, 0x01, cobid);
            driver.drive().sdo_write_u8(g.comm, 0x02, 255);
            driver.drive().sdo_write_u16(g.comm, 0x03, 0);
            driver.drive().sdo_write_u8(g.map, 0x00, 2);                 // START

            // verify
            uint8_t n = 0;
            uint32_t m0 = 0, cob = 0;
            driver.drive().sdo_read_u8(g.map, 0x00, n);
            driver.drive().sdo_read_u32(g.map, 0x01, m0);
            driver.drive().sdo_read_u32(g.comm, 0x01, cob);

            // dừng trước để đo sạch
            driver.stop();
            wait_ms(400);
            const uint32_t before = read_target_via_sdo(driver.drive());
            send_rpdo(cobid, L, R, false);
            wait_ms(500);
            const uint32_t after = read_target_via_sdo(driver.drive());

            char label[64];
            std::snprintf(label, sizeof(label), "F) 0x%04X cobid=0x%X", g.comm, cobid);
            report(label, before, after,
                   static_cast<int16_t>(g_tpdo_l.load()),
                   static_cast<int16_t>(g_tpdo_r.load()));
            std::cout << "      n=" << static_cast<int>(n)
                      << " m0=0x" << std::hex << m0 << std::dec
                      << " cobid_read=0x" << std::hex << cob << std::dec
                      << "  (mong đợi n=2, m0=0x60FF0110)\n";
        }
    }


    // ============ G) Chế độ SYNCHRONOUS (0x200F=1) + RPDO + SYNC liên tục ============
    {
        std::cout << "\n--- G) Synchronous mode (0x200F=1) ---\n";
        bring_operational(node);

        // 0x200F = 1 → chế độ đồng bộ (theo PDF: "Set synchronization control")
        driver.drive().sdo_write_u8(0x200F, 0x00, 1);
        // Mode profile velocity
        driver.drive().sdo_write_u8(0x6060, 0x00, 3);
        // Enable motor
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0006);
        wait_ms(100);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0007);
        wait_ms(100);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x000F);
        wait_ms(200);

        // RPDO1: mapping 2x16bit, transmission type 1 (cyclic — mỗi SYNC)
        driver.drive().sdo_write_u8(0x1600, 0x00, 0);
        driver.drive().sdo_write_u32(0x1600, 0x01, 0x60FF0110u);
        driver.drive().sdo_write_u32(0x1600, 0x02, 0x60FF0220u);
        driver.drive().sdo_write_u32(0x1400, 0x01, 0x200u + node);
        driver.drive().sdo_write_u8(0x1400, 0x02, 1);    // type 1 = mỗi SYNC
        driver.drive().sdo_write_u16(0x1400, 0x03, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 2);    // START

        driver.stop();
        wait_ms(400);

        const uint32_t before = read_target_via_sdo(driver.drive());

        // Gửi SYNC + RPDO liên tục (10 vòng)
        for (int i = 0; i < 10; ++i) {
            CANFrame sync;
            sync.set_id(0x080);
            sync.set_len(0);
            bus.send(sync);
            send_rpdo(0x200u + node, L, R, false);
            wait_ms(30);
        }
        wait_ms(400);

        const uint32_t after = read_target_via_sdo(driver.drive());
        report("G) SYNC mode + RPDO + SYNC frames", before, after,
               static_cast<int16_t>(g_tpdo_l.load()),
               static_cast<int16_t>(g_tpdo_r.load()));
        driver.stop();
    }

    // ============ H) RPDO chứa Controlword 0x6040 + Velocity trong CÙNG frame ============
    // Một số firmware ZLAC chỉ chấp nhận RPDO khi có Controlword kèm theo.
    // Mapping: 0x6040:00 (16bit) + 0x60FF:03 (32bit) = 6 bytes
    {
        std::cout << "\n--- H) RPDO = Controlword(0x6040) + Velocity(0x60FF:03) ---\n";
        const uint32_t combined =
            static_cast<uint32_t>(L) | (static_cast<uint32_t>(R) << 16);
        bring_operational(node);
        driver.drive().sdo_write_u8(0x200F, 0x00, 0);

        // Enable qua SDO trước (để statusword đạt 0x1C27)
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0006); wait_ms(80);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0007); wait_ms(80);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x000F); wait_ms(150);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x007F); wait_ms(150);

        driver.drive().sdo_write_u8(0x1600, 0x00, 0);
        driver.drive().sdo_write_u32(0x1600, 0x01, 0x60400010u);  // Controlword 16bit
        driver.drive().sdo_write_u32(0x1600, 0x02, 0x60FF0320u);  // Velocity 32bit
        driver.drive().sdo_write_u32(0x1400, 0x01, 0x200u + node);
        driver.drive().sdo_write_u8(0x1400, 0x02, 255);
        driver.drive().sdo_write_u16(0x1400, 0x03, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 2);

        driver.stop();
        wait_ms(400);
        const uint32_t before = read_target_via_sdo(driver.drive());

        // DLC = 6 khớp chính xác tổng mapping (2 + 4 byte)
        for (int i = 0; i < 10; ++i) {
            CANFrame f;
            f.set_id(0x200u + node);
            f.set_len(6);
            f.set_u16_le(0, 0x000F);   // Controlword: enable operation
            f.set_u32_le(2, combined);
            bus.send(f);
            wait_ms(40);
        }
        wait_ms(400);

        const uint32_t after = read_target_via_sdo(driver.drive());
        report("H) CW(0x6040)+vel, DLC=6", before, after,
               static_cast<int16_t>(g_tpdo_l.load()),
               static_cast<int16_t>(g_tpdo_r.load()));
        driver.stop();
    }

    // ============ I) DLC khớp CHÍNH XÁC mapping (không đệm) ============
    // 0x60FF:03 = 4 byte → DLC=4 ; 0x6040+0x60FF:01+02 = 6 byte → DLC=6
    {
        std::cout << "\n--- I) DLC khớp chính xác mapping ---\n";
        const uint32_t combined =
            static_cast<uint32_t>(L) | (static_cast<uint32_t>(R) << 16);
        const uint32_t before_cob = 0x200u + node;

        // I1: mapping 1x32bit, DLC = 4
        bring_operational(node);
        driver.drive().sdo_write_u8(0x200F, 0x00, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 0);
        driver.drive().sdo_write_u32(0x1600, 0x01, 0x60FF0320u);
        driver.drive().sdo_write_u32(0x1400, 0x01, before_cob);
        driver.drive().sdo_write_u8(0x1400, 0x02, 255);
        driver.drive().sdo_write_u16(0x1400, 0x03, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 1);
        driver.stop();
        wait_ms(400);
        const uint32_t b1 = read_target_via_sdo(driver.drive());
        for (int i = 0; i < 10; ++i) {
            CANFrame f;
            f.set_id(before_cob);
            f.set_len(4);                    // DLC = 4, đúng bằng mapping
            f.set_u32_le(0, combined);
            bus.send(f);
            wait_ms(40);
        }
        wait_ms(400);
        report("I1) 1x32bit, DLC=4 (không đệm)", b1,
               read_target_via_sdo(driver.drive()),
               static_cast<int16_t>(g_tpdo_l.load()),
               static_cast<int16_t>(g_tpdo_r.load()));
        const bool alive1 = drive_alive(driver.drive());
        std::cout << "      drive alive sau I1: " << (alive1 ? "OK" : "MẤT GIAO TIẾP") << "\n";

        // I2: mapping 0x6040(16) + 0x60FF:01(16) + 0x60FF:02(16) = 6 byte, DLC = 6
        bring_operational(node);
        driver.drive().sdo_write_u8(0x200F, 0x00, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 0);
        driver.drive().sdo_write_u32(0x1600, 0x01, 0x60400010u);
        driver.drive().sdo_write_u32(0x1600, 0x02, 0x60FF0110u);
        driver.drive().sdo_write_u32(0x1600, 0x03, 0x60FF0220u);
        driver.drive().sdo_write_u32(0x1400, 0x01, before_cob);
        driver.drive().sdo_write_u8(0x1400, 0x02, 255);
        driver.drive().sdo_write_u16(0x1400, 0x03, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 3);
        driver.stop();
        wait_ms(400);
        const uint32_t b2 = read_target_via_sdo(driver.drive());
        for (int i = 0; i < 10; ++i) {
            CANFrame f;
            f.set_id(before_cob);
            f.set_len(6);                    // DLC = 6, đúng bằng mapping
            f.set_u16_le(0, 0x000F);
            f.set_u16_le(2, static_cast<uint16_t>(L));
            f.set_u16_le(4, static_cast<uint16_t>(R));
            bus.send(f);
            wait_ms(40);
        }
        wait_ms(400);
        report("I2) CW+vel, DLC=6 (không đệm)", b2,
               read_target_via_sdo(driver.drive()),
               static_cast<int16_t>(g_tpdo_l.load()),
               static_cast<int16_t>(g_tpdo_r.load()));
        std::cout << "      drive alive sau I2: "
                  << (drive_alive(driver.drive()) ? "OK" : "MẤT GIAO TIẾP") << "\n";
        driver.stop();
    }

    // ============ J) END-TO-END: enable lại rồi gửi RPDO, motor phải CHẠY ============
    // I1 chứng minh RPDO được nhận, nhưng actual=0 vì sau NMT Reset Comm
    // CiA402 mất trạng thái enable. Test này enable rồi gửi RPDO để xác nhận
    // motor thực sự quay.
    {
        std::cout << "\n--- J) END-TO-END: enable + RPDO DLC=4 ---\n";
        bring_operational(node);
        driver.drive().sdo_write_u8(0x200F, 0x00, 0);

        // Enable CiA402 (sau NMT reset bắt buộc phải enable lại)
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0006); wait_ms(100);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0007); wait_ms(100);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x000F); wait_ms(150);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x007F); wait_ms(150);
        uint16_t sw = 0;
        driver.drive().sdo_read_u16(0x6041, 0x00, sw);
        std::cout << "      statusword sau enable = 0x" << std::hex << sw << std::dec << "\n";

        // Mapping 1x32bit — cấu hình đã chứng minh (I1)
        driver.drive().sdo_write_u8(0x1600, 0x00, 0);
        driver.drive().sdo_write_u32(0x1600, 0x01, 0x60FF0320u);
        driver.drive().sdo_write_u32(0x1400, 0x01, 0x200u + node);
        driver.drive().sdo_write_u8(0x1400, 0x02, 255);
        driver.drive().sdo_write_u16(0x1400, 0x03, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 1);

        driver.stop();
        wait_ms(500);
        const uint32_t b = read_target_via_sdo(driver.drive());

        // Gửi RPDO DLC=4 liên tục 15 vòng
        const uint32_t comb = static_cast<uint32_t>(L) | (static_cast<uint32_t>(R) << 16);
        for (int i = 0; i < 15; ++i) {
            CANFrame f;
            f.set_id(0x200u + node);
            f.set_len(4);
            f.set_u32_le(0, comb);
            bus.send(f);
            wait_ms(40);
        }
        wait_ms(500);

        report("J) enable + RPDO DLC=4", b, read_target_via_sdo(driver.drive()),
               static_cast<int16_t>(g_tpdo_l.load()),
               static_cast<int16_t>(g_tpdo_r.load()));
        std::cout << "      (mong đợi [NHẬN] + [CHẠY])\n";
        driver.stop();
    }

    // ============ K) END-TO-END đầy đủ: mode + profile + enable + RPDO ============
    // J nhận được RPDO nhưng motor đứng: sau NMT Reset Comm các tham số
    // vận hành (0x6060 mode, 0x6081 profile, 0x6083/0x6084 ramp) bị xóa
    // về mặc định nên 0x60FF không có tác dụng. Test này set lại đầy đủ.
    {
        std::cout << "\n--- K) mode + profile + enable + RPDO DLC=4 ---\n";
        bring_operational(node);
        driver.drive().sdo_write_u8(0x200F, 0x00, 0);

        // Đọc trạng thái sau reset để thấy rõ drive đã mất gì
        uint8_t mode_rb = 0xFF;
        uint16_t pv = 0, accel = 0, decel = 0;
        driver.drive().sdo_read_u8(0x6060, 0x00, mode_rb);
        driver.drive().sdo_read_u16(0x6081, 0x01, pv);
        driver.drive().sdo_read_u16(0x6083, 0x01, accel);
        driver.drive().sdo_read_u16(0x6084, 0x01, decel);
        std::cout << "      sau reset: 0x6060=" << static_cast<int>(mode_rb)
                  << " 0x6081=" << pv << " 0x6083=" << accel
                  << " 0x6084=" << decel << "\n";

        // Mode profile velocity + profile + ramp
        driver.set_operation_mode(3);
        driver.set_profile(100, 100, 100);
        wait_ms(300);

        uint16_t pv2 = 0, ac2 = 0, dc2 = 0;
        driver.drive().sdo_read_u16(0x6081, 0x01, pv2);
        driver.drive().sdo_read_u16(0x6083, 0x01, ac2);
        driver.drive().sdo_read_u16(0x6084, 0x01, dc2);
        std::cout << "      verify: 0x6081=" << pv2 << " 0x6083=" << ac2
                  << " 0x6084=" << dc2 << " (16-bit)\n";

        // Enable CiA402
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0006); wait_ms(100);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0007); wait_ms(100);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x000F); wait_ms(150);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x007F); wait_ms(150);

        uint8_t mode_rb2 = 0xFF;
        uint16_t sw = 0;
        driver.drive().sdo_read_u8(0x6061, 0x00, mode_rb2);  // modes of operation display
        driver.drive().sdo_read_u16(0x6041, 0x00, sw);
        std::cout << "      sau setup: 0x6061=" << static_cast<int>(mode_rb2)
                  << " statusword=0x" << std::hex << sw << std::dec << "\n";

        // Mapping 1x32bit
        driver.drive().sdo_write_u8(0x1600, 0x00, 0);
        driver.drive().sdo_write_u32(0x1600, 0x01, 0x60FF0320u);
        driver.drive().sdo_write_u32(0x1400, 0x01, 0x200u + node);
        driver.drive().sdo_write_u8(0x1400, 0x02, 255);
        driver.drive().sdo_write_u16(0x1400, 0x03, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 1);

        driver.stop();
        wait_ms(500);
        const uint32_t b = read_target_via_sdo(driver.drive());
        const int32_t tpdo_before = g_tpdo_l.load();

        const uint32_t comb = static_cast<uint32_t>(L) | (static_cast<uint32_t>(R) << 16);
        for (int i = 0; i < 20; ++i) {
            CANFrame f;
            f.set_id(0x200u + node);
            f.set_len(4);
            f.set_u32_le(0, comb);
            bus.send(f);
            wait_ms(40);
        }
        wait_ms(600);

        report("K) mode+profile+enable+RPDO", b, read_target_via_sdo(driver.drive()),
               static_cast<int16_t>(g_tpdo_l.load()),
               static_cast<int16_t>(g_tpdo_r.load()));
        std::cout << "      TPDO frames: " << tpdo_before << " -> "
                  << g_tpdo_l.load() << "\n";
        std::cout << "      (mong đợi [NHẬN] + [CHẠY] và TPDO tăng)\n";
        driver.stop();
    }

    // ============ L/M) RPDO + Controlword TRIGGER ============
    // SDO ghi 0x60FF:03 → motor chạy (test A). RPDO ghi cùng object, read-back
    // cũng đúng, nhưng motor đứng. Giả thuyết: RPDO cần Controlword đi kèm
    // để trigger (bit4=new setpoint, bit6/7=unlock ramp của ZLAC).
    {
        std::cout << "\n--- L) RPDO CW=0x007F + vel, DLC=6 ---\n";
        bring_operational(node);
        driver.drive().sdo_write_u8(0x200F, 0x00, 0);
        driver.set_operation_mode(3);
        driver.set_profile(120, 500, 500);
        wait_ms(200);

        // Enable qua SDO trước
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0006); wait_ms(100);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0007); wait_ms(100);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x000F); wait_ms(150);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x007F); wait_ms(150);

        // Mapping: Controlword(16) + Target_velocity(32) = 6 byte
        driver.drive().sdo_write_u8(0x1600, 0x00, 0);
        driver.drive().sdo_write_u32(0x1600, 0x01, 0x60400010u);
        driver.drive().sdo_write_u32(0x1600, 0x02, 0x60FF0320u);
        driver.drive().sdo_write_u32(0x1400, 0x01, 0x200u + node);
        driver.drive().sdo_write_u8(0x1400, 0x02, 255);
        driver.drive().sdo_write_u16(0x1400, 0x03, 0);
        driver.drive().sdo_write_u8(0x1600, 0x00, 2);

        driver.stop();
        wait_ms(400);
        const uint32_t b = read_target_via_sdo(driver.drive());
        const uint32_t comb = static_cast<uint32_t>(L) | (static_cast<uint32_t>(R) << 16);

        for (int i = 0; i < 20; ++i) {
            CANFrame f;
            f.set_id(0x200u + node);
            f.set_len(6);
            f.set_u16_le(0, 0x007F);   // Controlword đầy đủ (bit4+6+7)
            f.set_u32_le(2, comb);
            bus.send(f);
            wait_ms(40);
        }
        wait_ms(500);
        report("L) CW=0x007F + vel", b, read_target_via_sdo(driver.drive()),
               static_cast<int16_t>(g_tpdo_l.load()),
               static_cast<int16_t>(g_tpdo_r.load()));

        std::cout << "\n--- M) RPDO CW toggle bit4 (0x000F / 0x001F) ---\n";
        driver.stop();
        wait_ms(400);
        const uint32_t b2 = read_target_via_sdo(driver.drive());
        for (int i = 0; i < 20; ++i) {
            CANFrame f;
            f.set_id(0x200u + node);
            f.set_len(6);
            // Xoay vòng bit4 để phát sinh "new setpoint" mỗi frame
            f.set_u16_le(0, (i % 2) ? 0x001Fu : 0x000Fu);
            f.set_u32_le(2, comb);
            bus.send(f);
            wait_ms(40);
        }
        wait_ms(500);
        report("M) CW toggle bit4", b2, read_target_via_sdo(driver.drive()),
               static_cast<int16_t>(g_tpdo_l.load()),
               static_cast<int16_t>(g_tpdo_r.load()));

        std::cout << "\n--- N) Đối chiếu: SDO ghi 0x60FF:03 cùng giá trị ---\n";
        driver.stop();
        wait_ms(400);
        const uint32_t b3 = read_target_via_sdo(driver.drive());
        driver.drive().sdo_write_u32(0x60FF, 0x03, comb);
        wait_ms(700);
        report("N) SDO 0x60FF:03 (đối chiếu)", b3,
               read_target_via_sdo(driver.drive()),
               static_cast<int16_t>(g_tpdo_l.load()),
               static_cast<int16_t>(g_tpdo_r.load()));
        std::cout << "      (nếu N [CHẠY] còn L/M [đứng] → RPDO cần trigger riêng)\n";
        driver.stop();
    }

    // ==================== DỪNG AN TOÀN ====================
    {
        std::cout << "\n--- Dừng an toàn ---\n";
        CANFrame f;
        f.set_id(0x200u + node);
        f.set_len(4);
        f.set_u32_le(0, 0);
        for (int i = 0; i < 5; ++i) { bus.send(f); wait_ms(40); }
        driver.drive().sdo_write_u32(0x60FF, 0x03, 0);
        driver.drive().sdo_write_u16(0x6040, 0x00, 0x0006);
        wait_ms(300);
        std::cout << "      target=0, controlword=0x0006 (shutdown)\n";
    }

    std::cout << "\n=== Kết thúc ===\n";
    return 0;
}
