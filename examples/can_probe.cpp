/**
 * @file can_probe.cpp
 * @brief CAN bus diagnostic tool — quét node + test SDO
 *
 * 1. Quét node ID 1-127: gửi SDO upload 0x6041 (Statusword) tới từng node,
 *    báo node nào trả lời.
 * 2. Với node tìm thấy: đọc thêm 0x1000 (Device type) và test viết 0x6040.
 *
 * Usage:
 *   ./can_probe              # dùng interface từ YAML (can0)
 *   ./can_probe can0 1       # override interface + node
 */

#include <canopen/co/sdo/sdo.hpp>
#include <canopen/can/raw/socket_can_bus.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdlib>

#if HAVE_YAML
#include <yaml-cpp/yaml.h>
#endif

using namespace canopen;

int main(int argc, char* argv[]) {
    std::string interface = "can0";
    int probe_node = 0;  // 0 = quét tất cả

    // Simple args: [interface] [node] [--config file]
    std::string config_path;
#if HAVE_YAML
    config_path = CANOPEN_DEFAULT_CONFIG;
#endif
    int positional = 0;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (!arg.empty() && arg[0] != '-') {
            if (positional == 0) interface = arg;
            else if (positional == 1) probe_node = std::stoi(arg);
            positional++;
        }
    }

#if HAVE_YAML
    if (probe_node == 0 && !config_path.empty()) {
        try {
            YAML::Node cfg = YAML::LoadFile(config_path);
            if (cfg["canopen"]) {
                interface = cfg["canopen"]["interface"].as<std::string>(interface);
            }
        } catch (...) {}
    }
#endif

    std::cout << "=== CAN Probe: " << interface << " ===\n";

    SocketCanBus bus(interface);
    if (bus.open() < 0 || !bus.is_up()) {
        std::cerr << "ERROR: cannot open " << interface << " (errno "
                  << bus.socket().get_error_num() << ")\n";
        return 1;
    }
    std::cout << "Interface open OK\n\n";

    // Đếm TẤT CẢ frame nhận được (để biết drive có trả lời frame nào không)
    std::atomic<int> rx_count{0};
    const int counter_route = bus.add_route_all(
        [&rx_count](const CANFrame&) { rx_count++; });

    // ==================== Phase A: kiểm tra RX (chờ heartbeat 2s) ====================
    std::cout << "Phase A: cho frame bat ky trong 2s (heartbeat ~1Hz)...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    uint64_t tx_frames = 0;
    {
        CANStats stats;
        bus.socket().get_stats(stats);
        tx_frames = stats.tx_frames;
    }

    // ==================== Phase A0: NMT Start → Operational ====================
    // Nhiều firmware (kể cả ZLAC) chỉ xử lý SDO khi node ở
    // pre-operational/operational. Drive có thể đang ở STOPPED (heartbeat 0x04).
    if (probe_node > 0) {
        std::cout << "Phase A0: gui NMT Start cho node " << probe_node << "...\n";
        const uint8_t node = static_cast<uint8_t>(probe_node);

        // 0x000: cmd=0x01 (Start), node
        CANFrame nmt;
        nmt.set_id(0x000);
        nmt.set_len(2);
        nmt.set_u8(0, 0x01);
        nmt.set_u8(1, node);
        bus.send(nmt);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // Reset Communication (0x82) để drive về pre-op rồi Start lại
        nmt.set_u8(0, 0x82);
        bus.send(nmt);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        nmt.set_u8(0, 0x01);
        bus.send(nmt);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        std::cout << "  => Heartbeat sau NMT nen la 0x05 (operating)\n";
    }
    std::cout << "\n";

    const int rx_a = rx_count.load();
    std::cout << "  Frames received: " << rx_a << "\n";
    if (rx_a == 0) {
        std::cout << "  => RX KHONG HOAT DONG! Frame co tren bus (candump thay)\n"
                  << "     nhung socket khong nhan duoc -> bug trong RX path.\n";
    } else {
        std::cout << "  => RX hoat dong binh thuong.\n";
    }
    std::cout << "\n";

    // ==================== Phase B: quét node ====================
    std::vector<uint8_t> found;
    const int first = (probe_node > 0) ? probe_node : 1;
    const int last  = (probe_node > 0) ? probe_node : 127;

    // Phase B0: RAW frame test — bypass SDO layer hoàn toàn
    if (probe_node > 0) {
        const uint8_t node = static_cast<uint8_t>(probe_node);
        std::cout << "Phase B0: RAW frame test (bypass SDO layer)...\n";

        const int before = rx_count.load();
        CANFrame raw;
        raw.set_id(0x600u + node);
        raw.set_len(8);
        raw.set_u8(0, 0x40);     // upload initiate
        raw.set_u16_le(1, 0x6041);
        raw.set_u8(3, 0x00);
        // bytes 4-7 = 0
        bus.send(raw);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        const int after = rx_count.load();
        std::cout << "  Frames sau raw request: " << (after - before) << "\n";
        if (after - before == 0) {
            std::cout << "  => DRIVE KHONG TRA LOI (raw frame cung khong)\n"
                      << "     => van de phan cung / cau hinh drive, khong phai code\n";
        } else {
            std::cout << "  => Co frame tra ve! (SDO layer can co van de)\n";
        }
        std::cout << "\n";
    }

    std::cout << "Phase B: quet node " << first << "-" << last
              << " (SDO 0x6041, 300ms/node)...\n";

    const int rx_before = rx_count.load();
    for (int node = first; node <= last; node++) {
        SDOClient client(&bus, static_cast<uint8_t>(node));
        client.set_timeout(300);
        client.attach(bus);

        uint16_t sw = 0;
        SDOError err = client.upload(0x6041, 0x00, sw);

        if (err == SDOError::OK) {
            std::cout << "  NODE " << std::setw(3) << node
                      << " RESPONDS: Statusword = 0x" << std::hex << sw
                      << std::dec << "\n";
            found.push_back(static_cast<uint8_t>(node));
        }
    }
    const int rx_total = rx_count.load() - rx_before;
    std::cout << "\n  Frames received during scan: " << rx_total << "\n";

    // TX stats sau quét — verify SDO requests có ra bus không
    uint64_t tx_after = 0;
    {
        CANStats stats;
        bus.socket().get_stats(stats);
        tx_after = stats.tx_frames;
    }
    std::cout << "  Frames TRANSMITTED during scan: " << (tx_after - tx_frames) << "\n";
    if (tx_after - tx_frames == 0) {
        std::cout << "  => KHONG frame nao ra bus -> bug trong TX path!\n";
    } else if (rx_total == 0) {
        std::cout << "  => Requests DA RA bus nhung drive KHONG TRA LOI.\n"
                  << "     Kiem tra: nguon dong co, day CAN_H/L, node id.\n";
    }
    bus.remove_route(counter_route);

    if (found.empty()) {
        std::cout << "\n=== KHONG co node nao tra loi ===\n\n";
        std::cout << "Kiem tra:\n";
        std::cout << "  1. Nguon dong co (motor power, khong chi logic power)\n";
        std::cout << "  2. Day CAN_H/CAN_L (co the bi dao) + dien tro 120 ohm\n";
        std::cout << "  3. Baudrate drive co phai 500Kbps khong (bus dang 500K)\n";
        std::cout << "  4. Node ID cua drive (thu ./can_probe " << interface
                  << " <node_id>)\n";
        std::cout << "  5. candump " << interface
                  << " trong terminal khac de xem co frame nao khong\n";
        return 1;
    }

    // ==================== Chi tiết node tìm thấy ====================
    std::cout << "\n=== Chi tiet node ===\n";
    for (uint8_t node : found) {
        SDOClient client(&bus, node);
        client.set_timeout(200);
        client.attach(bus);

        std::cout << "Node " << static_cast<int>(node) << ":\n";

        uint32_t dev_type = 0;
        if (client.upload(0x1000, 0x00, dev_type) == SDOError::OK) {
            std::cout << "  0x1000 Device type : 0x" << std::hex << dev_type
                      << std::dec << "\n";
        } else {
            std::cout << "  0x1000 Device type : KHONG DOC DUOC\n";
        }

        uint16_t sw = 0;
        if (client.upload(0x6041, 0x00, sw) == SDOError::OK) {
            std::cout << "  0x6041 Statusword  : 0x" << std::hex << sw << std::dec;
            if (sw & 0x0040) std::cout << " (SWITCH_ON_DISABLED)";
            if (sw & 0x0001) std::cout << " (READY_TO_SWITCH_ON)";
            if (sw & 0x0002) std::cout << " (SWITCHED_ON)";
            if (sw & 0x0004) std::cout << " (OPERATION_ENABLED)";
            if (sw & 0x0008) std::cout << " (FAULT)";
            std::cout << "\n";
        }

        uint16_t mode = 0;
        if (client.upload(0x6060, 0x00, mode) == SDOError::OK) {
            std::cout << "  0x6060 Op mode     : " << mode << "\n";
        }
    }

    std::cout << "\n=== Done: " << found.size() << " node(s) found ===\n";
    return 0;
}
