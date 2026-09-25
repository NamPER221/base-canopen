/**
 * @file test_object_dictionary.cpp
 * @brief Basic Object Dictionary tests
 */

#include <canopen/co/object_dictionary/object_dictionary.hpp>
#include <canopen/co/object_dictionary/object.hpp>
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

#define ASSERT_NE(a, b) do { \
    if ((a) == (b)) throw std::runtime_error("Expected " + std::to_string(a) + " != " + std::to_string(b)); \
} while(0)

// Test: Create Object Dictionary
TEST(create_object_dictionary) {
    ObjectDictionary od;
    ASSERT_TRUE(od.empty());
    ASSERT_EQ(od.size(), 0u);
}

// Test: Add object entry
TEST(add_object) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x1000;
    entry.subindex = 0x00;
    entry.data_type = DataType::UNSIGNED32;
    entry.access = AccessType::RO;

    int result = od.add_object(entry);
    ASSERT_EQ(result, 0);
    ASSERT_EQ(od.size(), 1u);
}

// Test: Add duplicate object fails
TEST(add_duplicate_object) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x1000;
    entry.subindex = 0x00;

    od.add_object(entry);
    int result = od.add_object(entry);  // Should fail
    ASSERT_NE(result, 0);
}

// Test: Get object
TEST(get_object) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x1018;
    entry.subindex = 0x01;
    entry.data_type = DataType::UNSIGNED32;

    od.add_object(entry);

    ObjectEntry* retrieved = od.get_object(0x1018, 0x01);
    ASSERT_TRUE(retrieved != nullptr);
    ASSERT_EQ(retrieved->index, 0x1018);
    ASSERT_EQ(retrieved->subindex, 0x01);
}

// Test: Get non-existent object
TEST(get_nonexistent_object) {
    ObjectDictionary od;
    ObjectEntry* retrieved = od.get_object(0x9999, 0x00);
    ASSERT_TRUE(retrieved == nullptr);
}

// Test: Object exists check
TEST(object_exists) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x1000;
    entry.subindex = 0x00;

    ASSERT_TRUE(!od.exists(0x1000, 0x00));
    od.add_object(entry);
    ASSERT_TRUE(od.exists(0x1000, 0x00));
}

// Test: Remove object
TEST(remove_object) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x1000;
    entry.subindex = 0x00;

    od.add_object(entry);
    ASSERT_EQ(od.size(), 1u);

    bool removed = od.remove_object(0x1000, 0x00);
    ASSERT_TRUE(removed);
    ASSERT_EQ(od.size(), 0u);
}

// Test: Read/Write object value
TEST(read_write_value) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x2000;
    entry.subindex = 0x01;
    entry.data_type = DataType::UNSIGNED32;
    entry.access = AccessType::RW;

    od.add_object(entry);

    uint32_t write_val = 0x12345678;
    int result = od.write(0x2000, 0x01, &write_val, sizeof(write_val));
    ASSERT_EQ(result, 0);

    uint32_t read_val = 0;
    size_t read_size = sizeof(read_val);
    result = od.read(0x2000, 0x01, static_cast<void*>(&read_val), read_size);
    ASSERT_EQ(result, 0);
    ASSERT_EQ(read_val, write_val);
}

// Test: Write to read-only object fails
TEST(write_readonly_fails) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x1000;
    entry.subindex = 0x00;
    entry.access = AccessType::RO;

    od.add_object(entry);

    uint32_t val = 100;
    int result = od.write(0x1000, 0x00, static_cast<const void*>(&val), sizeof(val));
    ASSERT_NE(result, 0);  // Should fail
}

// Test: Template read/write
TEST(template_read_write) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x2000;
    entry.subindex = 0x01;
    entry.data_type = DataType::UNSIGNED16;
    entry.access = AccessType::RW;

    od.add_object(entry);

    uint16_t write_val = 0xABCD;
    int result = od.write(0x2000, 0x01, write_val);
    ASSERT_EQ(result, 0);

    uint16_t read_val = 0;
    result = od.read(0x2000, 0x01, read_val);
    ASSERT_EQ(result, 0);
    ASSERT_EQ(read_val, write_val);
}

// Test: Get subentries
TEST(get_subentries) {
    ObjectDictionary od;

    // Add multiple subentries
    for (uint8_t i = 0; i <= 4; i++) {
        ObjectEntry entry;
        entry.index = 0x1018;
        entry.subindex = i;
        entry.data_type = DataType::UNSIGNED32;
        od.add_object(entry);
    }

    auto subentries = od.get_subentries(0x1018);
    ASSERT_EQ(subentries.size(), 5u);
}

// Test: Highest subindex
TEST(highest_subindex) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x1018;
    entry.subindex = 0x03;
    od.add_object(entry);

    uint8_t highest = od.get_highest_subindex(0x1018);
    ASSERT_EQ(highest, 0x03);
}

// Test: Make key
TEST(make_key) {
    uint32_t key = ObjectDictionary::make_key(0x1018, 0x01);
    ASSERT_EQ(key, 0x101801);
}

// Test: Clear
TEST(clear) {
    ObjectDictionary od;

    // Add different entries
    for (uint8_t i = 0; i < 3; i++) {
        ObjectEntry entry;
        entry.index = 0x1000;
        entry.subindex = i;
        od.add_object(entry);
    }

    ASSERT_EQ(od.size(), 3u);
    od.clear();
    ASSERT_TRUE(od.empty());
}

// Test: Iteration
TEST(iteration) {
    ObjectDictionary od;

    for (int i = 0; i < 5; i++) {
        ObjectEntry entry;
        entry.index = 0x1000 + i;
        entry.subindex = 0x00;
        od.add_object(entry);
    }

    int count = 0;
    for (const auto& obj : od) {
        (void)obj;
        count++;
    }
    ASSERT_EQ(count, 5);
}

// Test: Default value initialization
TEST(default_value) {
    ObjectDictionary od;

    ObjectEntry entry;
    entry.index = 0x2000;
    entry.subindex = 0x00;
    entry.data_type = DataType::UNSIGNED32;
    entry.access = AccessType::RW;

    uint32_t default_val = 42;
    entry.default_value.assign(
        reinterpret_cast<uint8_t*>(&default_val),
        reinterpret_cast<uint8_t*>(&default_val) + sizeof(default_val)
    );

    od.add_object(entry);

    uint32_t read_val = 0;
    size_t size = sizeof(read_val);
    od.read(0x2000, 0x00, static_cast<void*>(&read_val), size);
    ASSERT_EQ(read_val, 42u);
}

// Main
int main() {
    std::cout << "Object Dictionary Tests\n";
    std::cout << "======================\n\n";

    RUN_TEST(create_object_dictionary);
    RUN_TEST(add_object);
    RUN_TEST(add_duplicate_object);
    RUN_TEST(get_object);
    RUN_TEST(get_nonexistent_object);
    RUN_TEST(object_exists);
    RUN_TEST(remove_object);
    RUN_TEST(read_write_value);
    RUN_TEST(write_readonly_fails);
    RUN_TEST(template_read_write);
    RUN_TEST(get_subentries);
    RUN_TEST(highest_subindex);
    RUN_TEST(make_key);
    RUN_TEST(clear);
    RUN_TEST(iteration);
    RUN_TEST(default_value);

    std::cout << "\n======================\n";
    std::cout << "Results: " << tests_passed << " passed, " << tests_failed << " failed\n";

    return tests_failed > 0 ? 1 : 0;
}
