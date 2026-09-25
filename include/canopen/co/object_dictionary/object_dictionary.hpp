/**
 * @file object_dictionary.hpp
 * @brief Object Dictionary implementation
 */

#ifndef CANOPEN_CO_OBJECT_DICTIONARY_OBJECT_DICTIONARY_HPP
#define CANOPEN_CO_OBJECT_DICTIONARY_OBJECT_DICTIONARY_HPP

#include <canopen/co/object_dictionary/object.hpp>
#include <map>
#include <vector>
#include <functional>
#include <optional>
#include <string>

namespace canopen {

// Forward declarations
class ObjectDictionary;

/**
 * @brief Object Dictionary iterator
 */
class ODIterator {
public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = ObjectEntry;
    using difference_type = std::ptrdiff_t;
    using pointer = const ObjectEntry*;
    using reference = const ObjectEntry&;

    ODIterator(const ObjectDictionary* od, bool end);
    reference operator*() const;
    pointer operator->() const;
    ODIterator& operator++();
    ODIterator operator++(int);
    ODIterator& operator--();
    ODIterator operator--(int);
    bool operator==(const ODIterator& other) const;
    bool operator!=(const ODIterator& other) const;

private:
    const ObjectDictionary* od_;
    std::map<uint32_t, ObjectEntry>::const_iterator it_;
};

/**
 * @brief Object Dictionary
 *
 * Core data structure for CANopen that holds all device parameters.
 */
class ObjectDictionary {
    friend class ODIterator;

public:
    /**
     * @brief Key type for object lookup
     */
    using Key = uint32_t;  // (index << 8) | subindex

    ObjectDictionary();
    explicit ObjectDictionary(uint8_t node_id);
    ~ObjectDictionary() = default;

    // ==================== Object Management ====================

    /**
     * @brief Add single object entry
     */
    int add_object(const ObjectEntry& entry);

    /**
     * @brief Add multiple objects from array
     */
    int add_objects(const ObjectEntry* entries, size_t count);

    /**
     * @brief Remove object
     */
    bool remove_object(uint16_t index, uint8_t subindex);

    /**
     * @brief Get mutable object
     */
    ObjectEntry* get_object(uint16_t index, uint8_t subindex);

    /**
     * @brief Get immutable object
     */
    const ObjectEntry* get_object(uint16_t index, uint8_t subindex) const;

    /**
     * @brief Check if object exists
     */
    bool exists(uint16_t index, uint8_t subindex) const;

    /**
     * @brief Get all subindex entries for an index
     */
    std::vector<ObjectEntry*> get_subentries(uint16_t index);

    /**
     * @brief Get highest subindex
     */
    uint8_t get_highest_subindex(uint16_t index) const;

    // ==================== Value Access ====================

    /**
     * @brief Read value from object
     */
    int read(uint16_t index, uint8_t subindex, void* data, size_t& size) const;

    /**
     * @brief Write value to object
     */
    int write(uint16_t index, uint8_t subindex, const void* data, size_t size);

    /**
     * @brief Template read
     */
    template<typename T>
    int read(uint16_t index, uint8_t subindex, T& value) const;

    /**
     * @brief Template write
     */
    template<typename T>
    int write(uint16_t index, uint8_t subindex, const T& value);

    // ==================== Iteration ====================

    ODIterator begin() const;
    ODIterator end() const;

    /**
     * @brief Iterate over all objects
     */
    void for_each(std::function<bool(const ObjectEntry&)> callback) const;

    /**
     * @brief Get all objects in an index range
     */
    std::vector<const ObjectEntry*> get_range(uint16_t start, uint16_t end) const;

    // ==================== PDO Support ====================

    /**
     * @brief Get all PDO-mappable objects
     */
    std::vector<const ObjectEntry*> get_pdo_mappable() const;

    /**
     * @brief Check if object is PDO-mappable
     */
    bool is_pdo_mappable(uint16_t index, uint8_t subindex) const;

    // ==================== EDS/DCF Support ====================

    /**
     * @brief Load from EDS file
     * @param node_id Node ID used to resolve $NODEID defaults
     *                (set BEFORE parsing; call set_node_id() first if
     *                the EDS lacks a NodeID= entry)
     */
    int load_eds(const std::string& filename, uint8_t node_id = 0);

    /**
     * @brief Load from DCF file
     */
    int load_dcf(const std::string& filename, uint8_t node_id = 0);

    /**
     * @brief Save to EDS file
     */
    int save_eds(const std::string& filename) const;

    // ==================== Node ID ====================

    /**
     * @brief Set node ID (updates dynamic COB-IDs)
     */
    void set_node_id(uint8_t node_id);

    /**
     * @brief Get node ID
     */
    uint8_t get_node_id() const { return node_id_; }

    // ==================== Utility ====================

    /**
     * @brief Get object count
     */
    size_t size() const { return objects_.size(); }

    /**
     * @brief Check if empty
     */
    bool empty() const { return objects_.empty(); }

    /**
     * @brief Clear all objects
     */
    void clear();

    /**
     * @brief Get object by key
     */
    ObjectEntry* get(Key key);
    const ObjectEntry* get(Key key) const;

    /**
     * @brief Make key from index/subindex
     */
    static constexpr Key make_key(uint16_t index, uint8_t subindex) {
        return (static_cast<Key>(index) << 8) | subindex;
    }

private:
    static Key make_key_internal(uint16_t index, uint8_t subindex);

    std::map<Key, ObjectEntry> objects_;
    uint8_t node_id_{0};
};

// ==================== Template Implementations ====================

template<typename T>
int ObjectDictionary::read(uint16_t index, uint8_t subindex, T& value) const {
    size_t size = sizeof(T);
    int result = read(index, subindex, &value, size);
    if (result == 0 && size != sizeof(T)) {
        return -EINVAL;  // Size mismatch
    }
    return result;
}

template<typename T>
int ObjectDictionary::write(uint16_t index, uint8_t subindex, const T& value) {
    return write(index, subindex, &value, sizeof(T));
}

} // namespace canopen

#endif // CANOPEN_CO_OBJECT_DICTIONARY_OBJECT_DICTIONARY_HPP
