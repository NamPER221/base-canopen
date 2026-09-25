/**
 * @file test_frame.cpp
 * @brief Basic CAN Frame tests
 */

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

// Test: Create empty frame
TEST(create_empty_frame) {
    CANFrame frame;
    ASSERT_EQ(frame.id(), 0u);
    ASSERT_EQ(frame.len(), 0u);
}

// Test: Create frame with ID and data
TEST(create_frame_with_data) {
    uint8_t data[] = {0x11, 0x22, 0x33, 0x44};
    CANFrame frame(0x123, data, 4);

    ASSERT_EQ(frame.id(), 0x123u);
    ASSERT_EQ(frame.len(), 4u);
}

// Test: Set and get ID
TEST(set_get_id) {
    CANFrame frame;
    frame.set_id(0x456);
    ASSERT_EQ(frame.id(), 0x456u);
}

// Test: Set and get length
TEST(set_get_length) {
    CANFrame frame;
    frame.set_len(8);
    ASSERT_EQ(frame.len(), 8u);
}

// Test: Create from native can_frame
TEST(from_native_can_frame) {
    struct can_frame native;
    native.can_id = 0x5AA;
    native.can_dlc = 4;
    native.data[0] = 0x11;
    native.data[1] = 0x22;
    native.data[2] = 0x33;
    native.data[3] = 0x44;

    CANFrame frame(native);

    ASSERT_EQ(frame.id(), 0x5AAu);
    ASSERT_EQ(frame.len(), 4u);
    ASSERT_EQ(frame.get_u8(0), 0x11);
}

// Test: Create from native canfd_frame
TEST(from_native_canfd_frame) {
    struct canfd_frame native;
    native.can_id = 0x5AA;
    native.len = 8;
    native.flags = 0;
    for (int i = 0; i < 8; i++) {
        native.data[i] = i;
    }

    CANFrame frame(native);

    ASSERT_EQ(frame.id(), 0x5AAu);
    ASSERT_EQ(frame.len(), 8u);
    ASSERT_EQ(frame.get_u8(0), 0u);
    ASSERT_EQ(frame.get_u8(7), 7u);
}

// Test: Convert to native
TEST(to_native) {
    CANFrame frame;
    frame.set_id(0x7FF);
    frame.set_len(8);

    const can_frame& native = frame.to_native();

    ASSERT_EQ(native.can_id, 0x7FFu);
    ASSERT_EQ(native.can_dlc, 8u);
}

// Test: RTR flag
TEST(rtr_flag) {
    CANFrame frame;
    frame.set_id(0x180 | CAN_RTR_FLAG);

    ASSERT_TRUE(frame.is_rtr());
}

// Test: Create with data constructor
TEST(create_with_data) {
    uint8_t data[] = {1, 2, 3, 4, 5};
    CANFrame frame(0x123, data, 5);

    ASSERT_EQ(frame.id(), 0x123u);
    ASSERT_EQ(frame.len(), 5u);
}

// Test: Set len truncates data
TEST(len_truncation) {
    uint8_t data[] = {1, 2, 3, 4, 5, 6, 7, 8};
    CANFrame frame(0x100, data, 8);

    frame.set_len(3);
    ASSERT_EQ(frame.len(), 3u);
}

// Test: Clear frame (reset to default)
TEST(clear_frame) {
    CANFrame frame;
    frame.set_id(0x123);
    frame.set_len(1);

    // Reset to default state
    frame.set_id(0);
    frame.set_len(0);

    ASSERT_EQ(frame.id(), 0u);
    ASSERT_EQ(frame.len(), 0u);
}

// Main
int main() {
    std::cout << "CAN Frame Tests\n";
    std::cout << "==============\n\n";

    RUN_TEST(create_empty_frame);
    RUN_TEST(create_frame_with_data);
    RUN_TEST(set_get_id);
    RUN_TEST(set_get_length);
    RUN_TEST(from_native_can_frame);
    RUN_TEST(from_native_canfd_frame);
    RUN_TEST(to_native);
    RUN_TEST(rtr_flag);
    RUN_TEST(create_with_data);
    RUN_TEST(len_truncation);
    RUN_TEST(clear_frame);

    std::cout << "\n==============\n";
    std::cout << "Results: " << tests_passed << " passed, " << tests_failed << " failed\n";

    return tests_failed > 0 ? 1 : 0;
}
