/**
 * @file consumer.cpp
 * @brief Project mẫu dùng thư viện đã cài — không nằm trong repo canopen
 *
 * Chỉ dùng header công khai qua tên target canopen::canopen, đúng như một
 * dự án khác sẽ dùng. Nếu compile + chạy được thì phần đóng gói đã đúng.
 */

#include <canopen/can/frame/frame.hpp>
#include <canopen/version.hpp>
#include <canopen/can/msg/message_factory.hpp>
#include <canopen/co/nmt/nmt.hpp>
#include <canopen/co/sdo/sdo.hpp>
#include <canopen/co/sync/sync.hpp>

#include <cstdio>
#include <cstring>

int main() {
    int failures = 0;
    auto check = [&failures](bool ok, const char* what) {
        std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
        if (!ok) ++failures;
    };

    // 1. Frame cơ bản
    canopen::CANFrame f;
    f.set_id(0x601);
    f.set_len(8);
    f.set_u8(0, 0x40);
    check(f.id() == 0x601 && f.len() == 8 && f.get_u8(0) == 0x40,
          "CANFrame cơ bản");

    // 2. MessageFactory — NMT
    const auto nmt = canopen::MessageFactory::create_nmt(
        1, canopen::NMTCommand::OPERATIONAL);
    check(nmt.id() == 0x000 && nmt.get_u8(1) == 1, "MessageFactory::create_nmt");

    // 3. Heartbeat
    const auto hb = canopen::MessageFactory::create_heartbeat(
        1, canopen::NMTState::OPERATIONAL);
    check(hb.id() == 0x701 && hb.get_u8(0) == 0x05, "MessageFactory::create_heartbeat");

    // 4. SDO expedited — cả hai chế độ encoding đều phải dùng được
    const uint16_t val16 = 0x0006;
    const auto sdo_std = canopen::MessageFactory::create_sdo_download_request(
        1, 0x6040, 0, &val16, 2, canopen::SdoEncoding::Standard);
    const auto sdo_leg = canopen::MessageFactory::create_sdo_download_request(
        1, 0x6040, 0, &val16, 2, canopen::SdoEncoding::Legacy);
    check(sdo_std.get_u8(0) == 0x2E && sdo_leg.get_u8(0) == 0x2B,
          "SDO download: chuẩn 0x2E / cũ 0x2B");

    // 5. ByteOrder enum dùng được từ bên ngoài
    const canopen::ByteOrder be = canopen::ByteOrder::BigEndian;
    check(be == canopen::ByteOrder::BigEndian, "enum ByteOrder");

    // 6. Lớp SDOClient tạo được (chưa gắn bus — chỉ kiểm tra link/linker)
    canopen::SDOClient client(nullptr, 1);
    client.set_timeout(50);
    check(client.encoding() == canopen::SdoEncoding::Standard,
          "SDOClient khởi tạo và đọc được encoding");

    // 7. Phiên bản thư viện — macro từ header sinh phải khớp hàm runtime
    std::printf("  lib version: %s (build %s)\n",
                canopen::canopen_version(), canopen::canopen_build_stamp());
    check(std::strcmp(CANOPEN_VERSION_STRING, canopen::canopen_version()) == 0,
          "macro CANOPEN_VERSION_STRING khớp canopen_version()");

    // Compile-time kiểm tra: dự án bên ngoài yêu cầu >= 0.2 phải biên dịch được
    static_assert(CANOPEN_VERSION_MAJOR == 0, "major version");
    static_assert(CANOPEN_VERSION_MINOR >= 2, "yêu cầu >= 0.2");
    check(canopen::canopen_version() != nullptr, "canopen_version()");

    if (failures == 0) {
        std::printf("consumer: OK — thư viện dùng được từ bên ngoài\n");
        return 0;
    }
    std::printf("consumer: %d kiểm tra thất bại\n", failures);
    return 1;
}
