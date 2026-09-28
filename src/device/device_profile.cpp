/**
 * @file device_profile.cpp
 * @brief Tự động dò vai trò object CiA 402 từ EDS
 */

#include <canopen/device/device_profile.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

namespace canopen {

namespace {

std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

bool contains(const std::string& haystack, const char* needle) {
    return to_lower(haystack).find(to_lower(needle)) != std::string::npos;
}

bool is_signed_type(DataType t) {
    return t == DataType::INTEGER8 || t == DataType::INTEGER16 ||
           t == DataType::INTEGER32;
}

bool is_numeric_type(DataType t) {
    return is_signed_type(t) || t == DataType::UNSIGNED8 ||
           t == DataType::UNSIGNED16 || t == DataType::UNSIGNED32 ||
           t == DataType::REAL32;
}

/**
 * @brief Subindex 0 của một mảng chỉ là header, không phải giá trị
 *
 * Theo CiA 301, object kiểu ARRAY/RECORD có 0x00 = HighestSubIndex (loại
 * UNSIGNED8, tên "HighestSubIndex"). Object đơn giá trị thì 0x00 CHÍNH LÀ
 * giá trị. Nhờ phân biệt được điều này, cùng một logic dò được cả drive
 * chuẩn một trục (0x60FF:00 = tốc độ) lẫn ZLAC nhiều trục
 * (0x60FF:00 = header, giá trị nằm ở :01/:02).
 */
bool is_array_header(const ObjectEntry& e) {
    if (e.data_type == DataType::UNSIGNED8 || e.data_type == DataType::DOMAIN) {
        return true;
    }
    return contains(e.name, "highestsubindex");
}

const char* role_name(ObjectRole r) {
    switch (r) {
        case ObjectRole::Controlword:             return "controlword";
        case ObjectRole::Statusword:              return "statusword";
        case ObjectRole::ModesOfOperation:        return "modes_of_operation";
        case ObjectRole::ModesOfOperationDisplay: return "modes_of_operation_display";
        case ObjectRole::TargetVelocity:          return "target_velocity";
        case ObjectRole::ActualVelocity:          return "actual_velocity";
        case ObjectRole::TargetPosition:          return "target_position";
        case ObjectRole::ActualPosition:          return "actual_position";
        case ObjectRole::TargetTorque:            return "target_torque";
        case ObjectRole::ActualTorque:            return "actual_torque";
        case ObjectRole::ProfileVelocity:         return "profile_velocity";
        case ObjectRole::ProfileAcceleration:     return "profile_acceleration";
        case ObjectRole::ProfileDeceleration:     return "profile_deceleration";
        case ObjectRole::QuickStopDeceleration:   return "quick_stop_deceleration";
        case ObjectRole::ErrorCode:               return "error_code";
        default:                                  return "unknown";
    }
}

} // namespace

// ==================== ResolvedObject ====================

namespace {

/** Đọc n byte theo thứ tự đã cấu hình, trả về giá trị unsigned */
uint64_t load_bytes(const uint8_t* p, size_t n, ByteOrder order) {
    uint64_t v = 0;
    switch (order) {
        case ByteOrder::BigEndian:
            for (size_t i = 0; i < n; ++i) {
                v = (v << 8) | p[i];
            }
            return v;
        case ByteOrder::ByteSwapped:
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (n - 1) - i;
                v |= static_cast<uint64_t>(p[j]) << (8 * i);
            }
            return v;
        case ByteOrder::LittleEndian:
        default:
            for (size_t i = 0; i < n; ++i) {
                v |= static_cast<uint64_t>(p[i]) << (8 * i);
            }
            return v;
    }
}

/** Ghi n byte theo thứ tự đã cấu hình */
void store_bytes(uint8_t* p, size_t n, uint64_t v, ByteOrder order) {
    switch (order) {
        case ByteOrder::BigEndian:
            for (size_t i = 0; i < n; ++i) {
                p[i] = static_cast<uint8_t>((v >> (8 * (n - 1 - i))) & 0xFF);
            }
            return;
        case ByteOrder::ByteSwapped:
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (n - 1) - i;
                p[j] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
            }
            return;
        case ByteOrder::LittleEndian:
        default:
            for (size_t i = 0; i < n; ++i) {
                p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
            }
            return;
    }
}

/** Sign-extend từ n byte lên 64 bit */
int64_t sign_extend(uint64_t v, size_t n) {
    if (n >= 8) return static_cast<int64_t>(v);
    const uint64_t sign_bit = 1ULL << (n * 8 - 1);
    if (v & sign_bit) {
        return static_cast<int64_t>(v | (~0ULL << (n * 8)));
    }
    return static_cast<int64_t>(v);
}

} // namespace

bool ResolvedObject::to_double(const uint8_t* data, size_t len, double& out) const {
    if (!data || len == 0) return false;
    // EDS có khi khai sai kích thước (vd ZLAC khai statusword 32-bit trong
    // khi object thật chỉ 16-bit). Dùng số byte THỰC TẾ thiết bị trả về
    // thay vì tin tuyệt đối vào EDS — thiết bị mới là nguồn sự thật.
    const size_t n = (len < size) ? len : size;
    if (data_type == DataType::REAL32) {
        // REAL32: byte order vẫn áp dụng theo quy ước float
        uint8_t buf[4];
        store_bytes(buf, 4, load_bytes(data, 4, byte_order), byte_order);
        float f = 0.0f;
        std::memcpy(&f, buf, sizeof(f));
        out = f;
        return true;
    }
    {
        const uint64_t raw = load_bytes(data, n, byte_order);
        if (is_signed) {
            out = static_cast<double>(sign_extend(raw, n));
        } else {
            out = static_cast<double>(raw);
        }
        return true;
    }
}

bool ResolvedObject::from_double(double value, uint8_t* out, size_t& len) const {
    if (data_type == DataType::REAL32) {
        const float f = static_cast<float>(value);
        uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(bits));
        store_bytes(out, 4, bits, byte_order);
        len = 4;
        return true;
    }
    if (is_signed) {
        const int64_t v = static_cast<int64_t>(value);
        store_bytes(out, size, static_cast<uint64_t>(v), byte_order);
        len = size;
        return true;
    }
    {
        const uint64_t v = static_cast<uint64_t>(value);
        store_bytes(out, size, v, byte_order);
        len = size;
        return true;
    }
}

// ==================== DeviceProfile ====================

void DeviceProfile::probe_index(ObjectRole role, uint16_t index) {
    const ObjectEntry* sub0 = od_.get_object(index, 0);
    const bool is_array = (sub0 == nullptr) || is_array_header(*sub0);

    if (!is_array) {
        // Object đơn giá trị: subindex 0 chính là dữ liệu (chuẩn CiA 402)
        if (!is_numeric_type(sub0->data_type)) return;
        ResolvedObject r;
        r.valid = true;
        r.index = index;
        r.subindex = 0;
        r.data_type = sub0->data_type;
        r.access = sub0->access;
        r.size = sub0->size ? sub0->size : get_data_type_size(sub0->data_type);
        r.is_signed = is_signed_type(sub0->data_type);
        r.byte_order = byte_order_;
        r.name = to_lower(sub0->name);
        objects_[role] = r;
        return;
    }

    // Object kiểu mảng: giá trị nằm ở subindex >= 1. Lấy sub đầu tiên có
    // kiểu số (bỏ qua header và các phần tử không phải số).
    const auto subs = od_.get_subentries(index);
    for (const ObjectEntry* e : subs) {
        if (e->subindex == 0) continue;
        if (!is_numeric_type(e->data_type)) continue;
        if (role == ObjectRole::ActualVelocity && e->access == AccessType::WO) continue;
        ResolvedObject r;
        r.valid = true;
        r.index = index;
        r.subindex = e->subindex;
        r.data_type = e->data_type;
        r.access = e->access;
        r.size = e->size ? e->size : get_data_type_size(e->data_type);
        r.is_signed = is_signed_type(e->data_type);
        r.byte_order = byte_order_;
        r.name = to_lower(e->name);
        objects_[role] = r;
        return;
    }
}

void DeviceProfile::probe_by_name(ObjectRole role, const char* name_fragment) {
    if (has(role)) return;  // đã tìm theo index chuẩn

    const std::string wanted = to_lower(name_fragment);
    ObjectEntry* best = nullptr;
    ObjectDictionary& self = const_cast<ObjectDictionary&>(od_);

    self.for_each([&](const ObjectEntry& e) {
        if (best) return true;
        if (e.subindex != 0) return true;                 // bỏ qua subindex
        if (!is_numeric_type(e.data_type)) return true;   // bỏ qua chuỗi/enum
        if (e.name.empty()) return true;
        if (to_lower(e.name).find(wanted) == std::string::npos) return true;

        // Ưu tiên object PDO-mappable và ghi được (cho lệnh) hoặc đọc được
        const bool writable = (e.access == AccessType::RW || e.access == AccessType::WO);
        const bool readable = (e.access == AccessType::RW || e.access == AccessType::RO);
        const bool needs_write = (role == ObjectRole::TargetVelocity ||
                                  role == ObjectRole::TargetPosition ||
                                  role == ObjectRole::TargetTorque);
        if (needs_write && !writable) return true;
        if (!needs_write && !readable) return true;

        best = const_cast<ObjectEntry*>(&e);
        return true;
    });

    if (!best) return;
    probe_index(role, best->index);
}

void DeviceProfile::apply_all_probes() {
    // Index chuẩn CiA 402 — thử trước vì đáng tin nhất
    probe_index(ObjectRole::Controlword, 0x6040);
    probe_index(ObjectRole::Statusword, 0x6041);
    probe_index(ObjectRole::ModesOfOperation, 0x6060);
    probe_index(ObjectRole::ModesOfOperationDisplay, 0x6061);
    probe_index(ObjectRole::ActualPosition, 0x6064);
    probe_index(ObjectRole::ActualVelocity, 0x606C);
    probe_index(ObjectRole::TargetTorque, 0x6071);
    probe_index(ObjectRole::ActualTorque, 0x6077);
    probe_index(ObjectRole::TargetPosition, 0x607A);
    probe_index(ObjectRole::TargetVelocity, 0x60FF);
    probe_index(ObjectRole::ProfileVelocity, 0x6081);
    probe_index(ObjectRole::ProfileAcceleration, 0x6083);
    probe_index(ObjectRole::ProfileDeceleration, 0x6084);
    probe_index(ObjectRole::QuickStopDeceleration, 0x6085);
    probe_index(ObjectRole::ErrorCode, 0x603F);

    // Một số hãng đặt object lệch chuẩn — dò bổ sung theo tên trong EDS
    probe_by_name(ObjectRole::TargetVelocity, "target_velocity");
    probe_by_name(ObjectRole::ActualVelocity, "velocity_actual");
    probe_by_name(ObjectRole::Controlword, "controlword");
    probe_by_name(ObjectRole::Statusword, "statusword");
    probe_by_name(ObjectRole::ModesOfOperation, "modes_of_operation");
    probe_by_name(ObjectRole::ActualPosition, "position_actual");
    probe_by_name(ObjectRole::TargetPosition, "target_position");
}

DeviceProfile DeviceProfile::from_eds(const std::string& eds_path, uint8_t node_id) {
    DeviceProfile p;
    p.eds_path_ = eds_path;
    p.node_id_ = node_id;
    if (p.od_.load_eds(eds_path, node_id) < 0) {
        return p;  // od_ rỗng → is_usable() = false
    }
    p.apply_all_probes();
    return p;
}

DeviceProfile DeviceProfile::from_dictionary(const ObjectDictionary& od, uint8_t node_id) {
    DeviceProfile p;
    p.node_id_ = node_id;

    // Copy các entry vào OD riêng để hồ sơ sống độc lập với OD gọi vào.
    std::vector<ObjectEntry> entries;
    od.for_each([&entries](const ObjectEntry& e) {
        entries.push_back(e);
        return true;
    });
    if (!entries.empty()) {
        p.od_.add_objects(entries.data(), entries.size());
    }
    p.apply_all_probes();
    return p;
}

void DeviceProfile::set_override(ObjectRole role, uint16_t index, uint8_t subindex) {
    const ObjectEntry* e = od_.get_object(index, subindex);
    ResolvedObject r;
    r.valid = true;
    r.index = index;
    r.subindex = subindex;
    if (e) {
        r.data_type = e->data_type;
        r.access = e->access;
        r.size = e->size ? e->size : get_data_type_size(e->data_type);
        r.is_signed = is_signed_type(e->data_type);
        r.name = to_lower(e->name);
    }
    r.byte_order = byte_order_;
    r.from_override = true;
    objects_[role] = r;
}

const ResolvedObject& DeviceProfile::resolve(ObjectRole role) const {
    const auto it = objects_.find(role);
    if (it == objects_.end()) return empty_;
    return it->second;
}

std::vector<ObjectRole> DeviceProfile::missing_required() const {
    std::vector<ObjectRole> missing;
    const ObjectRole required[] = {
        ObjectRole::Controlword, ObjectRole::Statusword,
        ObjectRole::ModesOfOperation, ObjectRole::TargetVelocity,
    };
    for (ObjectRole r : required) {
        if (!has(r)) missing.push_back(r);
    }
    return missing;
}

void DeviceProfile::set_byte_order(ByteOrder order) {
    byte_order_ = order;
    // Object đã dò từ EDS nắm byte_order tại thời điểm dò, nên phải cập
    // nhật lại — nếu không thay đổi sẽ không có tác dụng với chúng.
    for (auto& [role, obj] : objects_) {
        obj.byte_order = byte_order_;
    }
}

const char* DeviceProfile::byte_order_name() const {
    switch (byte_order_) {
        case ByteOrder::BigEndian:   return "big";
        case ByteOrder::ByteSwapped: return "swapped";
        case ByteOrder::LittleEndian:
        default:                     return "little";
    }
}

std::string DeviceProfile::describe() const {
    std::ostringstream os;
    os << "DeviceProfile[" << (device_name_.empty() ? "?" : device_name_)
       << " node=" << static_cast<int>(node_id_) << "]";
    if (!eds_path_.empty()) os << " eds=" << eds_path_;
    os << "\n";
    if (!is_usable()) {
        os << "  KHÔNG DÙNG ĐƯỢC — thiếu:";
        for (ObjectRole r : missing_required()) os << ' ' << role_name(r);
        os << "\n";
        return os.str();
    }
    const ObjectRole all[] = {
        ObjectRole::Controlword, ObjectRole::Statusword,
        ObjectRole::ModesOfOperation, ObjectRole::TargetVelocity,
        ObjectRole::ActualVelocity, ObjectRole::ProfileVelocity,
        ObjectRole::ProfileAcceleration, ObjectRole::ProfileDeceleration,
    };
    for (ObjectRole r : all) {
        const ResolvedObject& o = resolve(r);
        if (!o.valid) {
            os << "  " << role_name(r) << ": (không có)\n";
            continue;
        }
        os << "  " << role_name(r) << ": 0x" << std::hex << o.index << std::dec
           << ':' << static_cast<int>(o.subindex)
           << "  size=" << o.size
           << (o.is_signed ? " signed" : " unsigned")
           << "  order=" << byte_order_name()
           << (o.from_override ? "  [OVERRIDE]" : "")
           << "  \"" << o.name << "\"\n";
    }
    return os.str();
}

} // namespace canopen
