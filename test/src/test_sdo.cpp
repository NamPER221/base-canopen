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
        // cmd 0x2B = e=1, s=1, n=2 (2-byte expedited)
        CANFrame req;
        req.set_id(0x601);
        req.set_len(8);
        req.set_u8(0, 0x2B);
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
        CHECK(resp.get_u8(0) == 0x43, "response cmd = 0x43 (e=1,s=1,n=0)");
        CHECK(resp.get_u16_le(1) == 0x1000, "response index matches");
        CHECK(resp.get_u32_le(4) == 0x00000192, "response data matches");
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
