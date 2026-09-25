/**
 * @file test_message_factory.cpp
 * @brief Basic Message Factory tests
 */

#include <canopen/can/msg/message_factory.hpp>
#include <canopen/can/frame/frame.hpp>
#include <linux/can.h>
#include <cassert>
#include <iostream>
#include <cstring>

using namespace canopen;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) void test_##name()
#define RUN_TEST(name) do { \
    std::cout << "  " << #name << "... "; \
    try { \
        test_##name(); \
        std::cout << "PASSED\n"; \
        tests_passed++; \
    } catch (const std::exception& e) { \
        std::cout << "FAILED: " << e.what() << "\n"; \
        tests_failed++; \
    } \
} while(0)

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) throw std::runtime_error("Assertion failed: " #cond); \
} while(0)

#define ASSERT_EQ(a, b) do { \
    if ((a) != (b)) throw std::runtime_error("Expected " + std::to_string(a) + " == " + std::to_string(b)); \
} while(0)

// Test: COBID calculations
TEST(cobid_nmt) {
    ASSERT_EQ(COBID::NMT, 0x000u);
}

TEST(cobid_sync) {
    ASSERT_EQ(COBID::SYNC, 0x080u);
}

TEST(cobid_sdo_tx) {
    ASSERT_EQ(COBID::sdo_tx(1), 0x581u);
    ASSERT_EQ(COBID::sdo_tx(127), 0x5FFu);
}

TEST(cobid_sdo_rx) {
    ASSERT_EQ(COBID::sdo_rx(1), 0x601u);
    ASSERT_EQ(COBID::sdo_rx(127), 0x67Fu);
}

TEST(cobid_tpdo_tx) {
    ASSERT_EQ(COBID::tpdo_tx(1, 1), 0x181u);
    ASSERT_EQ(COBID::tpdo_tx(1, 2), 0x281u);
    ASSERT_EQ(COBID::tpdo_tx(1, 3), 0x381u);
    ASSERT_EQ(COBID::tpdo_tx(1, 4), 0x481u);
}

TEST(cobid_rpdo_rx) {
    ASSERT_EQ(COBID::tpdo_rx(1, 1), 0x201u);
    ASSERT_EQ(COBID::tpdo_rx(1, 2), 0x301u);
    ASSERT_EQ(COBID::tpdo_rx(1, 3), 0x401u);
    ASSERT_EQ(COBID::tpdo_rx(1, 4), 0x501u);
}

TEST(cobid_heartbeat) {
    ASSERT_EQ(COBID::heartbeat(1), 0x701u);
    ASSERT_EQ(COBID::heartbeat(127), 0x77Fu);
}

TEST(cobid_valid_node_id) {
    ASSERT_TRUE(COBID::is_valid_node_id(1));
    ASSERT_TRUE(COBID::is_valid_node_id(127));
    ASSERT_TRUE(!COBID::is_valid_node_id(0));
    ASSERT_TRUE(!COBID::is_valid_node_id(128));
}

TEST(cobid_valid) {
    ASSERT_TRUE(COBID::is_valid(0x000));
    ASSERT_TRUE(COBID::is_valid(0x7FF));
    ASSERT_TRUE(!COBID::is_valid(0x800));  // Too large
}

// Test: Create NMT command
TEST(create_nmt) {
    CANFrame frame = MessageFactory::create_nmt(5, NMTCommand::OPERATIONAL);

    ASSERT_EQ(frame.id(), 0x000u);
    ASSERT_EQ(frame.len(), 2u);
    ASSERT_EQ(frame.get_u8(0), static_cast<uint8_t>(NMTCommand::OPERATIONAL));
    ASSERT_EQ(frame.get_u8(1), 5);
}

// Test: Parse NMT command
TEST(parse_nmt) {
    CANFrame frame = MessageFactory::create_nmt(10, NMTCommand::STOP);

    uint8_t node_id = 0;
    NMTCommand cmd;
    bool result = MessageFactory::parse_nmt(frame, node_id, cmd);

    ASSERT_TRUE(result);
    ASSERT_EQ(node_id, 10);
    ASSERT_EQ(static_cast<uint8_t>(cmd), static_cast<uint8_t>(NMTCommand::STOP));
}

// Test: Create SYNC
TEST(create_sync) {
    CANFrame frame = MessageFactory::create_sync(0);

    ASSERT_EQ(frame.id(), 0x080u);
    ASSERT_EQ(frame.len(), 0u);
}

// Test: Create SYNC with counter
TEST(create_sync_counter) {
    CANFrame frame = MessageFactory::create_sync(5);

    ASSERT_EQ(frame.id(), 0x080u);
    ASSERT_EQ(frame.len(), 1u);
    ASSERT_EQ(frame.get_u8(0), 5);
}

// Test: Parse SYNC
TEST(parse_sync) {
    CANFrame frame = MessageFactory::create_sync(10);

    uint8_t counter = 0;
    bool result = MessageFactory::parse_sync(frame, counter);

    ASSERT_TRUE(result);
    ASSERT_EQ(counter, 10);
}

// Test: Create Heartbeat
TEST(create_heartbeat) {
    CANFrame frame = MessageFactory::create_heartbeat(NMTState::OPERATIONAL);

    ASSERT_EQ(frame.len(), 1u);
    ASSERT_EQ(frame.get_u8(0), static_cast<uint8_t>(NMTState::OPERATIONAL));
}

// Test: Parse Heartbeat
TEST(parse_heartbeat) {
    CANFrame frame = MessageFactory::create_heartbeat(NMTState::PREOPERATIONAL);

    uint8_t node_id = 0;
    NMTState state = NMTState::INITIALISING;
    bool result = MessageFactory::parse_heartbeat(frame, node_id, state);

    ASSERT_TRUE(result);
    ASSERT_EQ(node_id, 0x00u);  // Default broadcast
    ASSERT_EQ(static_cast<uint8_t>(state), static_cast<uint8_t>(NMTState::PREOPERATIONAL));
}

// Test: Create SDO upload request
TEST(create_sdo_upload_request) {
    CANFrame frame = MessageFactory::create_sdo_upload_request(5, 0x1018, 1);

    ASSERT_EQ(frame.id(), 0x605u);
    // SDO messages must ALWAYS be 8 bytes (CiA 306 / ZLAC8015D requirement)
    ASSERT_EQ(frame.len(), 8u);
    ASSERT_EQ(frame.get_u16_le(1), 0x1018u);
    ASSERT_EQ(frame.get_u8(3), 1);
}

// Test: Create SDO download request (expedited)
TEST(create_sdo_download_request) {
    uint32_t data = 0x12345678;
    CANFrame frame = MessageFactory::create_sdo_download_request(
        10, 0x2000, 1, &data, 4);

    ASSERT_EQ(frame.id(), 0x60Au);
}

// Test: Create SDO abort
TEST(create_sdo_abort) {
    CANFrame frame = MessageFactory::create_sdo_abort(
        5, 0x1018, 1, 0x06020000);

    ASSERT_EQ(frame.id(), 0x585u);
    ASSERT_EQ(frame.len(), 8u);
}

// Test: Create EMCY
TEST(create_emcy) {
    CANFrame frame = MessageFactory::create_emcy(5, 0x0000, 0x01);

    ASSERT_TRUE(frame.id() >= 0x080 && frame.id() <= 0x0FF);  // Valid EMCY range
    ASSERT_EQ(frame.len(), 8u);
}

// Test: Get message type name
TEST(message_type_name) {
    ASSERT_TRUE(MessageFactory::get_message_type_name(0x000) == "NMT");
    ASSERT_TRUE(MessageFactory::get_message_type_name(0x080) == "SYNC");
    ASSERT_TRUE(MessageFactory::get_message_type_name(0x181) == "TPDO1");
    ASSERT_TRUE(MessageFactory::get_message_type_name(0x201) == "RPDO1");
    ASSERT_TRUE(MessageFactory::get_message_type_name(0x605) == "RSDO");
    ASSERT_TRUE(MessageFactory::get_message_type_name(0x585) == "TSDO");
    ASSERT_TRUE(MessageFactory::get_message_type_name(0x701) == "HEARTBEAT");
}

// Test: Check CANopen COB-ID
TEST(is_canopen_cobid) {
    ASSERT_TRUE(MessageFactory::is_canopen_cobid(0x000));   // NMT
    ASSERT_TRUE(MessageFactory::is_canopen_cobid(0x080));  // SYNC
    ASSERT_TRUE(MessageFactory::is_canopen_cobid(0x181)); // TPDO1
    ASSERT_TRUE(MessageFactory::is_canopen_cobid(0x701));  // Heartbeat
    ASSERT_TRUE(!MessageFactory::is_canopen_cobid(0x1000)); // Too high
}

// Test: Create RTR frame
TEST(create_rtr_frame) {
    CANFrame frame;
    frame.set_id(0x181 | CAN_RTR_FLAG);
    frame.set_len(8);

    ASSERT_TRUE(frame.is_rtr());
    ASSERT_EQ(frame.len(), 8u);
}

// Main
int main() {
    std::cout << "Message Factory Tests\n";
    std::cout << "====================\n\n";

    RUN_TEST(cobid_nmt);
    RUN_TEST(cobid_sync);
    RUN_TEST(cobid_sdo_tx);
    RUN_TEST(cobid_sdo_rx);
    RUN_TEST(cobid_tpdo_tx);
    RUN_TEST(cobid_rpdo_rx);
    RUN_TEST(cobid_heartbeat);
    RUN_TEST(cobid_valid_node_id);
    RUN_TEST(cobid_valid);
    RUN_TEST(create_nmt);
    RUN_TEST(parse_nmt);
    RUN_TEST(create_sync);
    RUN_TEST(create_sync_counter);
    RUN_TEST(parse_sync);
    RUN_TEST(create_heartbeat);
    RUN_TEST(parse_heartbeat);
    RUN_TEST(create_sdo_upload_request);
    RUN_TEST(create_sdo_download_request);
    RUN_TEST(create_sdo_abort);
    RUN_TEST(create_emcy);
    RUN_TEST(message_type_name);
    RUN_TEST(is_canopen_cobid);
    RUN_TEST(create_rtr_frame);

    std::cout << "\n====================\n";
    std::cout << "Results: " << tests_passed << " passed, " << tests_failed << " failed\n";

    return tests_failed > 0 ? 1 : 0;
}
