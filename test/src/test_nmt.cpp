/**
 * @file test_nmt.cpp
 * @brief NMT service tests over FakeBus (no hardware)
 */

#include <canopen/co/nmt/nmt.hpp>
#include <canopen/mock/fake_bus.hpp>
#include <iostream>

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
    std::cout << "=== NMT Service Tests (FakeBus) ===\n";

    ObjectDictionary od(1);
    test::FakeBus bus;

    NMTService nmt(od, &bus, 1);
    nmt.attach(bus);
    nmt.start();

    // ==================== Test 1: send NMT command ====================
    {
        bus.clear_sent();
        nmt.send_command(2, NMTCommand::OPERATIONAL);

        CANFrame frame = bus.last_sent();
        CHECK(frame.id() == 0x000, "NMT frame COB-ID = 0x000");
        CHECK(frame.get_u8(0) == 0x01, "NMT command = START (0x01)");
        CHECK(frame.get_u8(1) == 0x02, "NMT target node = 2");
    }

    // ==================== Test 2: receive heartbeat from node 2 ====================
    {
        bus.clear_sent();
        CANFrame hb = MessageFactory::create_heartbeat(2, NMTState::OPERATIONAL);
        bus.dispatch(hb);

        auto info = nmt.get_node_state(2);
        CHECK(info.state == NMTState::OPERATIONAL, "node 2 state = OPERATIONAL");
        CHECK(info.bootup_received == false, "node 2 not in bootup");
    }

    // ==================== Test 3: bootup detection ====================
    {
        bool bootup_cb = false;
        nmt.on_bootup = [&bootup_cb](uint8_t) { bootup_cb = true; };

        CANFrame hb = MessageFactory::create_heartbeat(3, NMTState::INITIALISING);
        bus.dispatch(hb);

        CHECK(bootup_cb, "bootup callback fired for node 3");
        auto info = nmt.get_node_state(3);
        CHECK(info.bootup_received, "node 3 bootup_received = true");
    }

    // ==================== Test 4: NMT command received (broadcast) ====================
    {
        CANFrame cmd = MessageFactory::create_nmt(0, NMTCommand::STOP);
        bus.dispatch(cmd);

        CHECK(nmt.get_state() == NMTState::STOPPED, "broadcast STOP sets local state");
    }

    // ==================== Test 5: NMT start received ====================
    {
        CANFrame cmd = MessageFactory::create_nmt(0, NMTCommand::OPERATIONAL);
        bus.dispatch(cmd);

        CHECK(nmt.get_state() == NMTState::OPERATIONAL, "broadcast START sets local state");
    }

    // ==================== Test 6: heartbeat producer ====================
    {
        bus.clear_sent();
        nmt.set_state(NMTState::OPERATIONAL);
        nmt.start_heartbeat_producer(50);  // 50ms interval

        std::this_thread::sleep_for(std::chrono::milliseconds(180));
        nmt.stop_heartbeat_producer();

        CHECK(bus.send_count() >= 2, "heartbeat producer sent frames");

        CANFrame hb = bus.last_sent();
        CHECK(hb.id() == 0x701, "heartbeat COB-ID = 0x700+1");
        CHECK(hb.get_u8(0) == 0x05, "heartbeat state = OPERATIONAL");
    }

    // ==================== Test 7: heartbeat COB-ID includes node ====================
    {
        bus.clear_sent();
        CANFrame hb5 = MessageFactory::create_heartbeat(5, NMTState::PREOPERATIONAL);
        bus.dispatch(hb5);

        auto info = nmt.get_node_state(5);
        CHECK(info.state == NMTState::PREOPERATIONAL, "node 5 heartbeat parsed");
    }

    nmt.stop();

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_total
              << " passed ===\n";
    return tests_passed == tests_total ? 0 : 1;
}
