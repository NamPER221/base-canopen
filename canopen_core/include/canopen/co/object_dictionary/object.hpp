/**
 * @file object.hpp
 * @brief Object Dictionary entry definitions
 */

#ifndef CANOPEN_CO_OBJECT_DICTIONARY_OBJECT_HPP
#define CANOPEN_CO_OBJECT_DICTIONARY_OBJECT_HPP

#include <cstdint>
#include <functional>
#include <string>
#include <variant>
#include <vector>

namespace canopen {

// ==================== Data Types ====================

/**
 * @brief CANopen data types
 */
enum class DataType : uint16_t {
    BOOLEAN = 0x01,
    INTEGER8 = 0x02,
    INTEGER16 = 0x03,
    INTEGER32 = 0x04,
    UNSIGNED8 = 0x05,
    UNSIGNED16 = 0x06,
    UNSIGNED32 = 0x07,
    REAL32 = 0x08,
    VISIBLE_STRING = 0x09,
    OCTET_STRING = 0x0A,
    UNICODE_STRING = 0x0B,
    TIME_OF_DAY = 0x0C,
    TIME_DIFFERENCE = 0x0D,
    DOMAIN = 0x0F,
    INTEGER64 = 0x10,
    UNSIGNED64 = 0x1B,
    REAL64 = 0x1C,
};

/**
 * @brief Access types for object dictionary entries
 */
enum class AccessType : uint8_t {
    CONST = 0,    // Read-only constant
    RO = 1,       // Read-only
    WO = 2,       // Write-only
    RW = 3,       // Read/Write
};

/**
 * @brief Get size of data type in bytes
 */
constexpr size_t get_data_type_size(DataType type) {
    switch (type) {
        case DataType::BOOLEAN: return 1;
        case DataType::INTEGER8: return 1;
        case DataType::INTEGER16: return 2;
        case DataType::INTEGER32: return 4;
        case DataType::UNSIGNED8: return 1;
        case DataType::UNSIGNED16: return 2;
        case DataType::UNSIGNED32: return 4;
        case DataType::REAL32: return 4;
        case DataType::TIME_OF_DAY: return 6;
        case DataType::TIME_DIFFERENCE: return 6;
        case DataType::INTEGER64: return 8;
        case DataType::UNSIGNED64: return 8;
        case DataType::REAL64: return 8;
        default: return 0;  // Variable size types
    }
}

// ==================== Object Entry ====================

/**
 * @brief Object Dictionary entry flags
 */
struct ObjectFlags {
    bool pdo_mappable : 1;
    bool pdo_able : 1;
    bool backup : 1;
    bool realtime : 1;
};

/**
 * @brief Object Dictionary entry
 */
struct ObjectEntry {
    uint16_t index{0};
    uint8_t subindex{0};
    DataType data_type{DataType::UNSIGNED32};
    AccessType access{AccessType::RW};
    uint8_t size{0};  // 0 = variable
    std::string name;

    ObjectFlags flags{};

    // Default value storage
    std::vector<uint8_t> default_value;
    std::vector<uint8_t> value;

    // Callbacks
    std::function<int(const void*, size_t)> on_write;
    std::function<int(void*, size_t)> on_read;
    std::function<void()> on_change;
    std::function<int()> validate;

    // Low-level access callbacks (for complex types)
    std::function<int(ObjectEntry&, const void*, size_t)> on_write_raw;
    std::function<int(const ObjectEntry&, void*, size_t)> on_read_raw;
};

/**
 * @brief Object Dictionary entry builder
 */
class ObjectEntryBuilder {
public:
    ObjectEntryBuilder& set_index(uint16_t index) {
        entry_.index = index;
        return *this;
    }

    ObjectEntryBuilder& set_subindex(uint8_t subindex) {
        entry_.subindex = subindex;
        return *this;
    }

    ObjectEntryBuilder& set_data_type(DataType type) {
        entry_.data_type = type;
        entry_.size = get_data_type_size(type);
        return *this;
    }

    ObjectEntryBuilder& set_access(AccessType access) {
        entry_.access = access;
        return *this;
    }

    ObjectEntryBuilder& set_name(const std::string& name) {
        entry_.name = name;
        return *this;
    }

    ObjectEntryBuilder& set_size(uint8_t size) {
        entry_.size = size;
        return *this;
    }

    ObjectEntryBuilder& set_flags(bool pdo_mappable, bool pdo_able, bool backup) {
        entry_.flags.pdo_mappable = pdo_mappable;
        entry_.flags.pdo_able = pdo_able;
        entry_.flags.backup = backup;
        return *this;
    }

    template<typename T>
    ObjectEntryBuilder& set_default_value(const T& val) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&val);
        entry_.default_value.assign(ptr, ptr + sizeof(T));
        return *this;
    }

    ObjectEntryBuilder& set_default_value(const void* data, size_t size) {
        entry_.default_value.assign(
            static_cast<const uint8_t*>(data),
            static_cast<const uint8_t*>(data) + size);
        return *this;
    }

    ObjectEntryBuilder& set_on_write(std::function<int(const void*, size_t)> cb) {
        entry_.on_write = std::move(cb);
        return *this;
    }

    ObjectEntryBuilder& set_on_read(std::function<int(void*, size_t)> cb) {
        entry_.on_read = std::move(cb);
        return *this;
    }

    ObjectEntryBuilder& set_on_change(std::function<void()> cb) {
        entry_.on_change = std::move(cb);
        return *this;
    }

    ObjectEntryBuilder& set_validate(std::function<int()> cb) {
        entry_.validate = std::move(cb);
        return *this;
    }

    ObjectEntry build() {
        return entry_;
    }

private:
    ObjectEntry entry_;
};

// ==================== PDO Mapping ====================

/**
 * @brief PDO mapping entry structure
 */
struct PDOMappingEntryDef {
    uint16_t object_index;
    uint8_t subindex;
    uint8_t bit_length;
};

/**
 * @brief PDO mapping definition
 */
struct PDOMappingDef {
    std::vector<PDOMappingEntryDef> entries;
    size_t data_size() const;
};

inline size_t PDOMappingDef::data_size() const {
    size_t total = 0;
    for (const auto& e : entries) {
        total += (e.bit_length + 7) / 8;
    }
    return total;
}

} // namespace canopen

#endif // CANOPEN_CO_OBJECT_DICTIONARY_OBJECT_HPP
