/**
 * @file test_sdo.cpp
 * @brief SDO Client/Server exchange tests over FakeBus (no hardware)
 */

#include <canopen/co/sdo/sdo.hpp>
#include <canopen/can/msg/message_factory.hpp>
#include <canopen/mock/fake_bus.hpp>
#include <iostream>
#include <cstring>

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

int main() {
    std::cout << "=== SDO Protocol Tests (FakeBus) ===\n";

    // ==================== Setup: OD with test objects ====================
    ObjectDictionary od(1);

    uint32_t device_type = 0x00000192;
    ObjectEntry e1;
    e1.index = 0x1000;
    e1.subindex = 0x00;
    e1.data_type = DataType::UNSIGNED32;
    e1.size = 4;
    e1.access = AccessType::RO;
    e1.name = "Device Type";
    e1.value.assign(reinterpret_cast<const uint8_t*>(&device_type),
                    reinterpret_cast<const uint8_t*>(&device_type) + 4);
    od.add_object(e1);

    uint16_t hb_time = 500;
    ObjectEntry e2;
    e2.index = 0x1017;
    e2.subindex = 0x00;
    e2.data_type = DataType::UNSIGNED16;
    e2.size = 2;
    e2.access = AccessType::RW;
    e2.name = "Heartbeat Time";
    e2.value.assign(reinterpret_cast<const uint8_t*>(&hb_time),
                    reinterpret_cast<const uint8_t*>(&hb_time) + 2);
    od.add_object(e2);

    // ==================== Setup: fake bus + client/server ====================
    test::FakeBus bus;
    SDOServer server(od, &bus);
    server.set_node_id(1);
    server.attach(bus);

    SDOClient client(&bus, 1);
    client.set_timeout(500);
    client.attach(bus);

    // ==================== Test 1: upload existing u32 ====================
    {
        uint32_t value = 0;
        size_t size = sizeof(value);
        SDOError err = client.upload_sync(0x1000, 0x00, &value, size);
        CHECK(err == SDOError::OK, "upload u32 returns OK");
        CHECK(value == 0x00000192, "upload u32 correct value");
        CHECK(size == 4, "upload u32 correct size");
    }

    // ==================== Test 2: template upload u16 ====================
    {
        uint16_t value = 0;
        SDOError err = client.upload(0x1017, 0x00, value);
        CHECK(err == SDOError::OK, "upload u16 template returns OK");
        CHECK(value == 500, "upload u16 template correct value");
    }

    // ==================== Test 3: download u16 then verify in OD ====================
    {
        uint16_t new_time = 800;
        SDOError err = client.download(0x1017, 0x00, new_time);
        CHECK(err == SDOError::OK, "download u16 returns OK");

        uint16_t od_value = 0;
        size_t sz = sizeof(od_value);
        od.read(0x1017, 0x00, &od_value, sz);
        CHECK(od_value == 800, "download u16 written to OD");
    }

    // ==================== Test 4: upload nonexistent object -> ABORT ====================
    {
        uint32_t value = 0;
        size_t size = sizeof(value);
        SDOError err = client.upload_sync(0x9999, 0x00, &value, size);
        CHECK(err == SDOError::ABORT, "upload nonexistent returns ABORT");
        CHECK(client.last_abort_code() ==
                  static_cast<uint32_t>(SDOAbortCode::OBJECT_NOT_EXIST),
              "abort code = OBJECT_NOT_EXIST");
    }

    // ==================== Test 5: download to RO object -> ABORT ====================
    {
        uint32_t dummy = 0x123;
        SDOError err = client.download(0x1000, 0x00, dummy);
        CHECK(err == SDOError::ABORT, "download to RO returns ABORT");
    }

    // ==================== Test 6: server writes value via request frame ====================
    {
        // Simulate a download request built manually (0x601, u16 0x1017 = 1234)
        // CiA 301 expedited download: bit3 e=1, bit2 s=1, bit1-0 n=2
        // → 0x20 | 0x0C | 0x02 = 0x2E  (truyền 2 byte)
        CANFrame req;
        req.set_id(0x601);
        req.set_len(8);
        req.set_u8(0, 0x2E);
        req.set_u16_le(1, 0x1017);
        req.set_u8(3, 0x00);
        req.set_u16_le(4, 1234);
        server.handle_frame(req);

        uint16_t od_value = 0;
        size_t sz = sizeof(od_value);
        od.read(0x1017, 0x00, &od_value, sz);
        CHECK(od_value == 1234, "server handle_frame writes to OD");
    }

    // ==================== Test 7: server response frame is correct ====================
    {
        bus.clear_sent();
        CANFrame up_req;
        up_req.set_id(0x601);
        up_req.set_len(4);
        up_req.set_u8(0, 0x40);  // upload initiate
        up_req.set_u16_le(1, 0x1000);
        up_req.set_u8(3, 0x00);
        server.handle_frame(up_req);

        CANFrame resp = bus.last_sent();
        CHECK(resp.id() == 0x581, "response COB-ID = 0x580+1");
        // Expedited upload 4 byte: 0x40 | 0x0C | n=0 = 0x4C
        CHECK(resp.get_u8(0) == 0x4C, "response cmd = 0x4C (e=1,s=1,n=0)");
        CHECK(resp.get_u16_le(1) == 0x1000, "response index matches");
        CHECK(resp.get_u32_le(4) == 0x00000192, "response data matches");
    }

    // ==================== Test 7b: response kiểu cũ (0x43) vẫn đọc được ======
    // ZLAC8015D trả về byte command theo cách đặt bit cũ (e=bit1, n=bit2-3)
    // → 0x43 thay vì 0x4C. Parser phải chấp nhận cả hai.
    {
        SDOClient c2(&bus, 9);
        c2.set_timeout(500);
        c2.attach(bus);

        auto legacy_route = bus.add_route(0x600u + 9u, 0x7FF, [&bus](const CANFrame& req) {
            CANFrame resp;
            resp.set_id(0x580u + 9u);
            resp.set_len(8);
            resp.set_u8(0, 0x43);                 // kiểu cũ, 4 byte
            resp.set_u16_le(1, req.get_u16_le(1));
            resp.set_u8(3, req.get_u8(3));
            resp.set_u32_le(4, 0x0000ABCDu);
            bus.dispatch(resp);
        });

        uint32_t v = 0;
        size_t sz = sizeof(v);
        const SDOError e = c2.upload_sync(0x1234, 0x00, &v, sz);
        CHECK(e == SDOError::OK, "upload response kiểu cũ 0x43 trả OK");
        CHECK(v == 0x0000ABCDu, "đọc đúng dữ liệu từ response kiểu cũ");
        CHECK(sz == 4, "nhận đủ 4 byte từ response kiểu cũ");
        c2.detach(bus);
        bus.remove_route(legacy_route);
    }

    // ==================== Test 7c: confirm 0x60 không echo index ==================
    // CiA 301 yêu cầu confirm lệnh ghi phải echo index/subindex, nhưng một số
    // drive (ZLAC8015D) trả về 0x60 với phần index = 0. Client phải nhận
    // confirm theo LOẠI giao dịch, nếu không sẽ bỏ rơi và request kế tiếp bị
    // drive abort.
    {
        SDOClient c3(&bus, 8);
        c3.set_timeout(300);
        c3.attach(bus);

        int downloads = 0, uploads = 0;
        auto r3 = bus.add_route(0x600u + 8u, 0x7FF, [&bus, &downloads, &uploads]
                                (const CANFrame& req) {
            const uint8_t cmd = req.get_u8(0);
            CANFrame resp;
            resp.set_id(0x580u + 8u);
            resp.set_len(8);
            if ((cmd & 0xE0) == 0x20) {          // download
                ++downloads;
                // Cố tình KHÔNG echo index: đặt 0x00 0x00 0x00
                resp.set_u8(0, 0x60);
                resp.set_u16_le(1, 0x0000);
                resp.set_u8(3, 0x00);
            } else {                              // upload
                ++uploads;
                resp.set_u8(0, 0x43);
                resp.set_u16_le(1, req.get_u16_le(1));
                resp.set_u8(3, req.get_u8(3));
                resp.set_u32_le(4, 0x00001234u);
            }
            bus.dispatch(resp);
        });

        // Ghi rồi đọc ngay — cả hai phải thành công
        const SDOError e1 = c3.download_sync(0x1600, 0x00, nullptr, 0) == SDOError::OK
                                ? SDOError::OK : SDOError::ABORT;
        uint32_t v = 0;
        size_t sz = sizeof(v);
        const SDOError e2 = c3.upload_sync(0x1018, 0x00, &v, sz);

        CHECK(downloads == 1 && e1 == SDOError::OK,
              "download thành công dù confirm không echo index");
        CHECK(uploads == 1 && e2 == SDOError::OK && v == 0x1234,
              "đọc ngay sau đó vẫn thành công (confirm đã được nhận đúng)");
        c3.detach(bus);
        bus.remove_route(r3);
    }

    // ==================== Test 8: timeout when bus down ====================
    {
        bus.clear_sent();
        SDOClient lonely(&bus, 2);  // node 2 never responds
        lonely.set_timeout(100);
        lonely.attach(bus);

        uint32_t value = 0;
        size_t size = sizeof(value);
        SDOError err = lonely.upload_sync(0x1000, 0x00, &value, size);
        CHECK(err == SDOError::TIMEOUT, "upload to missing node times out");
    }

    // ==================== Test 9: no bus -> NO_BUS ====================
    {
        SDOClient noclient(nullptr, 1);
        uint32_t value = 0;
        size_t size = sizeof(value);
        SDOError err = noclient.upload_sync(0x1000, 0x00, &value, size);
        CHECK(err == SDOError::NO_BUS, "upload without bus returns NO_BUS");
    }

    // ==================== Summary ====================
    std::cout << "\n=== Results: " << tests_passed << "/" << tests_total
              << " passed ===\n";
    return tests_passed == tests_total ? 0 : 1;
}
