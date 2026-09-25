/**
 * @file object_dictionary.cpp
 * @brief Object Dictionary implementation
 */

#include <canopen/co/object_dictionary/object_dictionary.hpp>
#include <cstring>
#include <fstream>
#include <algorithm>

namespace canopen {

ODIterator::ODIterator(const ObjectDictionary* od, bool end)
    : od_(od) {
    if (end) {
        it_ = od_->objects_.end();
    } else {
        it_ = od_->objects_.begin();
    }
}

ODIterator::reference ODIterator::operator*() const {
    return it_->second;
}

ODIterator::pointer ODIterator::operator->() const {
    return &it_->second;
}

ODIterator& ODIterator::operator++() {
    ++it_;
    return *this;
}

ODIterator ODIterator::operator++(int) {
    ODIterator tmp = *this;
    ++(*this);
    return tmp;
}

ODIterator& ODIterator::operator--() {
    --it_;
    return *this;
}

ODIterator ODIterator::operator--(int) {
    ODIterator tmp = *this;
    --(*this);
    return tmp;
}

bool ODIterator::operator==(const ODIterator& other) const {
    return it_ == other.it_;
}

bool ODIterator::operator!=(const ODIterator& other) const {
    return it_ != other.it_;
}

ObjectDictionary::ObjectDictionary() = default;

ObjectDictionary::ObjectDictionary(uint8_t node_id) : node_id_(node_id) {}

int ObjectDictionary::add_object(const ObjectEntry& entry) {
    Key key = make_key_internal(entry.index, entry.subindex);

    if (objects_.find(key) != objects_.end()) {
        return -EEXIST;  // Already exists
    }

    objects_[key] = entry;

    // Initialize value from default if present
    if (!entry.default_value.empty()) {
        objects_[key].value = entry.default_value;
    }

    return 0;
}

int ObjectDictionary::add_objects(const ObjectEntry* entries, size_t count) {
    int result = 0;
    for (size_t i = 0; i < count; ++i) {
        int r = add_object(entries[i]);
        if (r < 0) {
            result = r;
        }
    }
    return result;
}

bool ObjectDictionary::remove_object(uint16_t index, uint8_t subindex) {
    Key key = make_key_internal(index, subindex);
    return objects_.erase(key) > 0;
}

ObjectEntry* ObjectDictionary::get_object(uint16_t index, uint8_t subindex) {
    Key key = make_key_internal(index, subindex);
    auto it = objects_.find(key);
    if (it != objects_.end()) {
        return &it->second;
    }
    return nullptr;
}

const ObjectEntry* ObjectDictionary::get_object(uint16_t index, uint8_t subindex) const {
    Key key = make_key_internal(index, subindex);
    auto it = objects_.find(key);
    if (it != objects_.end()) {
        return &it->second;
    }
    return nullptr;
}

bool ObjectDictionary::exists(uint16_t index, uint8_t subindex) const {
    Key key = make_key_internal(index, subindex);
    return objects_.find(key) != objects_.end();
}

std::vector<ObjectEntry*> ObjectDictionary::get_subentries(uint16_t index) {
    std::vector<ObjectEntry*> result;
    for (auto& [key, entry] : objects_) {
        if ((key >> 8) == index) {
            result.push_back(&entry);
        }
    }
    std::sort(result.begin(), result.end(),
              [](const ObjectEntry* a, const ObjectEntry* b) {
                  return a->subindex < b->subindex;
              });
    return result;
}

uint8_t ObjectDictionary::get_highest_subindex(uint16_t index) const {
    uint8_t highest = 0;
    for (const auto& [key, entry] : objects_) {
        if ((key >> 8) == index && entry.subindex > highest) {
            highest = entry.subindex;
        }
    }
    return highest;
}

int ObjectDictionary::read(uint16_t index, uint8_t subindex, void* data, size_t& size) const {
    const ObjectEntry* entry = get_object(index, subindex);
    if (!entry) {
        return -ENOENT;
    }

    // Call on_read callback if present
    if (entry->on_read) {
        return entry->on_read(data, size);
    }

    // Use on_read_raw if present
    if (entry->on_read_raw) {
        return entry->on_read_raw(*entry, data, size);
    }

    // Read from value storage
    size_t read_size = std::min(size, entry->value.size());
    if (entry->value.empty() && !entry->default_value.empty()) {
        read_size = std::min(size, entry->default_value.size());
        std::memcpy(data, entry->default_value.data(), read_size);
    } else {
        std::memcpy(data, entry->value.data(), read_size);
    }
    size = read_size;

    return 0;
}

int ObjectDictionary::write(uint16_t index, uint8_t subindex, const void* data, size_t size) {
    ObjectEntry* entry = get_object(index, subindex);
    if (!entry) {
        return -ENOENT;
    }

    // Check access
    switch (entry->access) {
        case AccessType::CONST:
        case AccessType::RO:
            return -EACCES;
        default:
            break;
    }

    // Call on_write callback if present
    if (entry->on_write) {
        int result = entry->on_write(data, size);
        if (result < 0) return result;
    }

    // Use on_write_raw if present
    if (entry->on_write_raw) {
        int result = entry->on_write_raw(*entry, data, size);
        if (result < 0) return result;
    }

    // Validate size
    if (entry->size > 0 && size != entry->size) {
        return -EINVAL;
    }

    // Write to value storage
    entry->value.assign(static_cast<const uint8_t*>(data),
                        static_cast<const uint8_t*>(data) + size);

    // Trigger on_change callback
    if (entry->on_change) {
        entry->on_change();
    }

    return 0;
}

ODIterator ObjectDictionary::begin() const {
    return ODIterator(this, false);
}

ODIterator ObjectDictionary::end() const {
    return ODIterator(this, true);
}

void ObjectDictionary::for_each(std::function<bool(const ObjectEntry&)> callback) const {
    for (const auto& [key, entry] : objects_) {
        if (!callback(entry)) {
            break;
        }
    }
}

std::vector<const ObjectEntry*> ObjectDictionary::get_range(uint16_t start, uint16_t end) const {
    std::vector<const ObjectEntry*> result;
    for (const auto& [key, entry] : objects_) {
        uint16_t index = key >> 8;
        if (index >= start && index <= end) {
            result.push_back(&entry);
        }
    }
    return result;
}

std::vector<const ObjectEntry*> ObjectDictionary::get_pdo_mappable() const {
    std::vector<const ObjectEntry*> result;
    for (const auto& [key, entry] : objects_) {
        if (entry.flags.pdo_mappable) {
            result.push_back(&entry);
        }
    }
    return result;
}

bool ObjectDictionary::is_pdo_mappable(uint16_t index, uint8_t subindex) const {
    const ObjectEntry* entry = get_object(index, subindex);
    return entry && entry->flags.pdo_mappable;
}

// load_eds / load_dcf are implemented in eds_parser.cpp

int ObjectDictionary::save_eds(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return -ENOENT;
    }

    // Simple EDS output
    file << "[FileInfo]\n";
    file << "FileName=" << filename << "\n";
    file << "FileVersion=4.0\n\n";

    file << "[MandatoryObjects]\n";
    for (const auto& [key, entry] : objects_) {
        uint16_t index = key >> 8;
        uint8_t subindex = key & 0xFF;
        file << "0x" << std::hex << index << "=0x" << subindex
             << ",0x" << static_cast<int>(entry.size) << "\n";
    }

    return 0;
}

void ObjectDictionary::set_node_id(uint8_t node_id) {
    node_id_ = node_id;
    // Update dynamic COB-IDs
    // This would update entries like heartbeat, PDO COB-IDs, etc.
}

ObjectEntry* ObjectDictionary::get(Key key) {
    auto it = objects_.find(key);
    if (it != objects_.end()) {
        return &it->second;
    }
    return nullptr;
}

const ObjectEntry* ObjectDictionary::get(Key key) const {
    auto it = objects_.find(key);
    if (it != objects_.end()) {
        return &it->second;
    }
    return nullptr;
}

ObjectDictionary::Key ObjectDictionary::make_key_internal(uint16_t index, uint8_t subindex) {
    return (static_cast<Key>(index) << 8) | subindex;
}

void ObjectDictionary::clear() {
    objects_.clear();
}

} // namespace canopen
