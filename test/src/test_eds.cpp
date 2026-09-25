/**
 * @file test_eds.cpp
 * @brief EDS parser tests with the real ZLAC8015D.eds device file
 */

#include <canopen/co/object_dictionary/object_dictionary.hpp>
#include <iostream>
#include <cmath>

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
    std::cout << "=== EDS Parser Tests (ZLAC8015D.eds) ===\n";

    ObjectDictionary od;
    int rc = od.load_eds("ZLAC8015D.eds");
    if (rc == -ENOENT) {
        // Try from source dir (ctest working directory may differ)
        rc = od.load_eds("/home/namnc/base-canopen/ZLAC8015D.eds");
    }
    CHECK(rc == 0, "load_eds returns OK");
    CHECK(od.size() > 20, "OD has many entries loaded");
    CHECK(od.get_node_id() == 0, "no NodeID in EDS -> node id stays 0");

    // ==================== $NODEID expression resolution ====================
    {
        // Load again with node_id = 1: $NODEID+0x200 must resolve to 0x201
        ObjectDictionary od2;
        int rc2 = od2.load_eds("ZLAC8015D.eds", 1);
        if (rc2 == -ENOENT) {
            rc2 = od2.load_eds("/home/namnc/base-canopen/ZLAC8015D.eds", 1);
        }
        CHECK(rc2 == 0, "load_eds with node_id=1 returns OK");

        const ObjectEntry* rpdo_cob = od2.get_object(0x1400, 0x01);
        CHECK(rpdo_cob != nullptr, "0x1400sub1 (RPDO COB-ID) loaded");
        if (rpdo_cob) {
            uint32_t val = 0;
            size_t sz = sizeof(val);
            od2.read(0x1400, 0x01, &val, sz);
            CHECK(val == 0x201, "$NODEID+0x200 resolves to 0x201 (node 1)");
        }

        const ObjectEntry* tpdo_cob = od2.get_object(0x1800, 0x01);
        CHECK(tpdo_cob != nullptr, "0x1800sub1 (TPDO COB-ID) loaded");
        if (tpdo_cob) {
            uint32_t val = 0;
            size_t sz = sizeof(val);
            od2.read(0x1800, 0x01, &val, sz);
            CHECK(val == 0x181, "TPDO COB-ID = 0x181");
        }
    }

    // ==================== Mandatory objects ====================
    {
        const ObjectEntry* e = od.get_object(0x1000, 0x00);
        CHECK(e != nullptr, "0x1000 Device Type loaded");
        if (e) {
            CHECK(e->name == "Device Type", "0x1000 name parsed");
            CHECK(e->access == AccessType::RO, "0x1000 access = RO");
            uint32_t val = 0;
            size_t sz = sizeof(val);
            CHECK(od.read(0x1000, 0x00, &val, sz) == 0 && val == 0x20192,
                  "0x1000 DefaultValue = 0x20192");
        }
    }

    // ==================== Sub-objects (identity) ====================
    {
        const ObjectEntry* vendor = od.get_object(0x1018, 0x01);
        CHECK(vendor != nullptr, "0x1018sub1 Vendor Id loaded");
        if (vendor) {
            uint32_t val = 0;
            size_t sz = sizeof(val);
            od.read(0x1018, 0x01, &val, sz);
            CHECK(val == 0x00000100, "Vendor Id = 0x100");
        }
    }

    // ==================== CiA 402 objects ====================
    {
        const ObjectEntry* cw = od.get_object(0x6040, 0x00);
        CHECK(cw != nullptr, "0x6040 Controlword loaded");
        CHECK(cw->access == AccessType::RW, "0x6040 access = RW");

        const ObjectEntry* sw = od.get_object(0x6041, 0x00);
        CHECK(sw != nullptr, "0x6041 Statusword loaded");
    }

    // ==================== ZLAC manufacturer objects ====================
    {
        const ObjectEntry* node = od.get_object(0x200A, 0x00);
        CHECK(node != nullptr, "0x200A CAN Node ID loaded");
        CHECK(node->access == AccessType::RW, "0x200A access = RW");
    }

    // ==================== 0x60FF with subindexes ====================
    {
        const ObjectEntry* sub1 = od.get_object(0x60FF, 0x01);
        const ObjectEntry* sub2 = od.get_object(0x60FF, 0x02);
        const ObjectEntry* sub3 = od.get_object(0x60FF, 0x03);
        CHECK(sub1 != nullptr, "0x60FF:01 (Left target velocity) loaded");
        CHECK(sub2 != nullptr, "0x60FF:02 (Right target velocity) loaded");
        CHECK(sub3 != nullptr, "0x60FF:03 (Combined 32-bit) loaded");
        if (sub1) {
            CHECK(sub1->data_type == DataType::INTEGER32, "0x60FF:01 dtype = i32");
            int32_t val = 0;
            size_t sz = sizeof(val);
            od.read(0x60FF, 0x01, &val, sz);
            CHECK(val == 0, "0x60FF:01 default = 0");
        }
    }

    // ==================== PDO mapping entries ====================
    {
        const ObjectEntry* m = od.get_object(0x1600, 0x01);
        CHECK(m != nullptr, "0x1600:01 (RPDO1 mapping) loaded");
    }

    // ==================== Read-write via OD ====================
    {
        int8_t mode = 3;  // 0x6060 is INTEGER8 (1 byte) in ZLAC8015D.eds
        CHECK(od.write(0x6060, 0x00, &mode, sizeof(mode)) == 0, "write 0x6060 = 3");

        uint32_t speed = 500;
        CHECK(od.write(0x60FF, 0x01, &speed, sizeof(speed)) == 0, "write 0x60FF:01 = 500");

        uint32_t readback = 0;
        size_t sz = sizeof(readback);
        od.read(0x60FF, 0x01, &readback, sz);
        CHECK(readback == 500, "readback 0x60FF:01 = 500");
    }

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_total
              << " passed ===\n";
    return tests_passed == tests_total ? 0 : 1;
}
