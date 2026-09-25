/**
 * @file test_pdo.cpp
 * @brief PDO (TPDO/RPDO) tests over FakeBus (no hardware)
 */

#include <canopen/co/pdo/pdo.hpp>
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
    std::cout << "=== PDO Tests (FakeBus) ===\n";

    ObjectDictionary od(1);

    // OD object for PDO mapping: 0x2000 u32
    uint32_t proc_value = 0;
    ObjectEntry e1;
    e1.index = 0x2000;
    e1.subindex = 0x00;
    e1.data_type = DataType::UNSIGNED32;
    e1.size = 4;
    e1.access = AccessType::RW;
    e1.name = "Process Value";
    e1.value.assign(reinterpret_cast<const uint8_t*>(&proc_value),
                    reinterpret_cast<const uint8_t*>(&proc_value) + 4);
    od.add_object(e1);

    test::FakeBus bus;

    // ==================== TPDO ====================
    {
        TPDO tpdo(od, 1);
        PDOConfiguration cfg;
        cfg.set_cob_id(0x181);  // TPDO1 + node 1
        cfg.set_transmission_type(PDOTransmissionType::ASYNCHRONOUS);
        cfg.add_mapping_entry(0x2000, 0x00, 32);
        tpdo.configure(cfg);
        tpdo.set_bus(&bus);
        tpdo.start();

        // Write a value to OD, then send
        uint32_t value = 0xDEADBEEF;
        od.write(0x2000, 0x00, &value, sizeof(value));
        tpdo.update_from_od();

        bus.clear_sent();
        tpdo.send();

        CANFrame frame = bus.last_sent();
        CHECK(frame.id() == 0x181, "TPDO COB-ID = 0x180+1");
        CHECK(frame.len() == 4, "TPDO length = 4");
        CHECK(frame.get_u32_le(0) == 0xDEADBEEF, "TPDO payload = OD value");

        tpdo.stop();
    }

    // ==================== RPDO ====================
    {
        RPDO rpdo(od, 1);
        PDOConfiguration cfg;
        cfg.set_cob_id(0x201);  // RPDO1 + node 1
        cfg.set_transmission_type(PDOTransmissionType::ASYNCHRONOUS);
        cfg.add_mapping_entry(0x2000, 0x00, 32);
        rpdo.configure(cfg);
        rpdo.set_bus(&bus);
        rpdo.attach(bus);
        rpdo.start();

        // Simulate incoming RPDO frame
        CANFrame frame;
        frame.set_id(0x201);
        frame.set_len(4);
        frame.set_u32_le(0, 0x12345678);
        bus.dispatch(frame);

        uint32_t od_value = 0;
        size_t sz = sizeof(od_value);
        od.read(0x2000, 0x00, &od_value, sz);
        CHECK(od_value == 0x12345678, "RPDO frame written to OD");

        auto data = rpdo.get_data();
        CHECK(data.size() == 4, "RPDO data size = 4");

        rpdo.stop();
    }

    // ==================== RPDO callback ====================
    {
        RPDO rpdo(od, 2);
        PDOConfiguration cfg;
        cfg.set_cob_id(0x202);
        cfg.add_mapping_entry(0x2000, 0x00, 32);
        rpdo.configure(cfg);
        rpdo.attach(bus);
        rpdo.start();

        bool received = false;
        size_t rx_len = 0;
        rpdo.on_receive = [&received, &rx_len](const void*, size_t len) {
            received = true;
            rx_len = len;
        };

        CANFrame frame;
        frame.set_id(0x202);
        frame.set_len(4);
        frame.set_u32_le(0, 42);
        bus.dispatch(frame);

        CHECK(received, "RPDO on_receive callback fired");
        CHECK(rx_len == 4, "RPDO callback length = 4");

        rpdo.stop();
    }

    // ==================== RPDO ignores other COB-IDs ====================
    {
        RPDO rpdo(od, 3);
        PDOConfiguration cfg;
        cfg.set_cob_id(0x203);
        rpdo.configure(cfg);
        rpdo.start();

        CANFrame frame;
        frame.set_id(0x204);  // different PDO
        frame.set_len(4);
        frame.set_u32_le(0, 0xFFFFFFFF);
        rpdo.handle_frame(frame);

        uint32_t od_value = 0;
        size_t sz = sizeof(od_value);
        od.read(0x2000, 0x00, &od_value, sz);
        CHECK(od_value != 0xFFFFFFFF, "RPDO ignored foreign COB-ID");
    }

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_total
              << " passed ===\n";
    return tests_passed == tests_total ? 0 : 1;
}
