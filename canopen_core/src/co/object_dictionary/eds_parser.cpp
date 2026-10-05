/**
 * @file eds_parser.cpp
 * @brief EDS/DCF file parser for ObjectDictionary
 *
 * Implements ObjectDictionary::load_eds/load_dcf. Parses the standard
 * CiA 306 INI-like EDS format: [FileInfo], [DeviceInfo], [1000],
 * [1018sub1], ... including DefaultValue / AccessType / DataType /
 * PDOMapping / LowLimit / HighLimit.
 */

#include <canopen/co/object_dictionary/object_dictionary.hpp>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>

namespace canopen {

namespace {

// Trim whitespace both ends
std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Lowercase copy
std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

// Parse "0x1A", "-1000", "42" into uint64 (handles two's complement for negative)
bool parse_number(const std::string& str, uint64_t& out) {
    std::string s = trim(str);
    if (s.empty()) return false;

    std::string body = s;
    bool negative = false;
    if (body[0] == '-') {
        negative = true;
        body = body.substr(1);
    } else if (body[0] == '+') {
        body = body.substr(1);
    }

    uint64_t value = 0;
    try {
        if (body.size() > 2 && body[0] == '0' && (body[1] == 'x' || body[1] == 'X')) {
            value = static_cast<uint64_t>(std::stoull(body.substr(2), nullptr, 16));
        } else {
            value = static_cast<uint64_t>(std::stoull(body, nullptr, 0));
        }
    } catch (...) {
        return false;
    }

    out = negative ? (~value + 1) : value;
    return true;
}

// Parse standard EDS "$NODEID" expressions:
//   $NODEID, $NODEID+0x200, $NODEID-0x100
// Returns false if raw is not a $NODEID expression.
bool parse_nodeid_expr(const std::string& raw, uint8_t node_id, uint64_t& out) {
    const std::string s = trim(raw);
    if (s.rfind("$NODEID", 0) != 0 && s.rfind("$nodeid", 0) != 0) {
        return false;
    }

    std::string rest = trim(s.substr(7));
    if (rest.empty()) {
        out = node_id;
        return true;
    }

    uint64_t delta = 0;
    if (!parse_number(rest, delta)) return false;

    // delta may be a negative two's-complement value; unsigned wrap gives
    // the correct result ($NODEID-0x100 == node_id - 256).
    out = static_cast<uint64_t>(node_id) + delta;
    return true;
}

// Encode value little-endian with the size of the given data type
std::vector<uint8_t> encode_value(const std::string& raw, DataType type,
                                  uint8_t node_id = 0) {
    uint64_t num = 0;

    // Standard EDS $NODEID expression takes precedence
    if (!parse_nodeid_expr(raw, node_id, num)) {
        if (!parse_number(raw, num)) return {};
    }

    const size_t size = get_data_type_size(type);
    if (size == 0) {
        // String types: store as ASCII text
        std::string s = trim(raw);
        return std::vector<uint8_t>(s.begin(), s.end());
    }

    std::vector<uint8_t> bytes(size);
    for (size_t i = 0; i < size && i < 8; ++i) {
        bytes[i] = static_cast<uint8_t>((num >> (i * 8)) & 0xFF);
    }
    return bytes;
}

AccessType parse_access(const std::string& raw) {
    const std::string a = lower(trim(raw));
    if (a == "ro" || a == "const") return AccessType::RO;
    if (a == "wo") return AccessType::WO;
    if (a == "rw") return AccessType::RW;
    return AccessType::RW;
}

DataType parse_data_type(const std::string& raw) {
    uint64_t code = 0;
    if (!parse_number(raw, code)) return DataType::UNSIGNED32;

    switch (code) {
        case 0x01: return DataType::BOOLEAN;
        case 0x02: return DataType::INTEGER8;
        case 0x03: return DataType::INTEGER16;
        case 0x04: return DataType::INTEGER32;
        case 0x05: return DataType::UNSIGNED8;
        case 0x06: return DataType::UNSIGNED16;
        case 0x07: return DataType::UNSIGNED32;
        case 0x08: return DataType::REAL32;
        case 0x09: return DataType::VISIBLE_STRING;
        case 0x0A: return DataType::OCTET_STRING;
        case 0x0B: return DataType::UNICODE_STRING;
        case 0x0C: return DataType::TIME_OF_DAY;
        case 0x0D: return DataType::TIME_DIFFERENCE;
        case 0x0F: return DataType::DOMAIN;
        case 0x10: return DataType::INTEGER64;
        case 0x1B: return DataType::UNSIGNED64;
        case 0x1C: return DataType::REAL64;
        default: return DataType::UNSIGNED32;
    }
}

// Section header: [1000] / [1018sub1] / [FileInfo]
struct SectionRef {
    bool is_object{false};
    uint16_t index{0};
    uint8_t subindex{0};
};

bool parse_section(const std::string& header, SectionRef& ref) {
    const size_t open = header.find('[');
    const size_t close = header.find(']');
    if (open == std::string::npos || close == std::string::npos || close < open) {
        return false;
    }

    const std::string name = lower(trim(header.substr(open + 1, close - open - 1)));

    const size_t sub = name.find("sub");
    if (sub != std::string::npos) {
        try {
            ref.index = static_cast<uint16_t>(std::stoul(name.substr(0, sub), nullptr, 16));
            ref.subindex = static_cast<uint8_t>(std::stoul(name.substr(sub + 3), nullptr, 16));
        } catch (...) {
            return false;
        }
        ref.is_object = true;
        return true;
    }

    // Pure hex number = object index
    if (name.size() == 4 && std::all_of(name.begin(), name.end(), ::isxdigit)) {
        try {
            ref.index = static_cast<uint16_t>(std::stoul(name, nullptr, 16));
        } catch (...) {
            return false;
        }
        ref.is_object = true;
        ref.subindex = 0;
        return true;
    }

    ref.is_object = false;  // FileInfo / DeviceInfo / etc.
    return true;
}

} // anonymous namespace

// ==================== ObjectDictionary::load_eds ====================

int ObjectDictionary::load_eds(const std::string& filename, uint8_t node_id) {
    if (node_id != 0) {
        set_node_id(node_id);  // resolve $NODEID defaults with this node
    }

    std::ifstream file(filename);
    if (!file.is_open()) {
        return -ENOENT;
    }

    std::string line;
    std::string section_header;
    bool in_object_section = false;
    SectionRef section{};
    ObjectEntry entry{};
    bool has_entry = false;

    auto flush_entry = [&]() {
        if (has_entry && in_object_section) {
            // Overwrite semantics: EDS entries take precedence
            remove_object(section.index, section.subindex);
            add_object(entry);
        }
        has_entry = false;
        entry = ObjectEntry{};
    };

    while (std::getline(file, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == ';' || trimmed[0] == '#') {
            continue;  // blank / comment
        }

        if (trimmed[0] == '[') {
            flush_entry();
            section_header = trimmed;
            if (parse_section(section_header, section) && section.is_object) {
                in_object_section = true;
                entry.index = section.index;
                entry.subindex = section.subindex;
                has_entry = true;
            } else {
                in_object_section = false;
            }
            continue;
        }

        // key=value
        const size_t eq = trimmed.find('=');
        if (eq == std::string::npos) continue;

        const std::string key = lower(trim(trimmed.substr(0, eq)));
        const std::string value = trim(trimmed.substr(eq + 1));

        if (!in_object_section) {
            // [DeviceInfo]: capture node id if present ("NodeID=")
            if (key == "nodeid") {
                uint64_t node = 0;
                if (parse_number(value, node) && node >= 1 && node <= 127) {
                    set_node_id(static_cast<uint8_t>(node));
                }
            }
            continue;
        }

        if (key == "parametername") {
            entry.name = value;
        } else if (key == "datatype") {
            entry.data_type = parse_data_type(value);
            const size_t sz = get_data_type_size(entry.data_type);
            if (sz > 0) entry.size = static_cast<uint8_t>(sz);
        } else if (key == "accesstype") {
            entry.access = parse_access(value);
        } else if (key == "defaultvalue") {
            entry.default_value = encode_value(value, entry.data_type, node_id_);
        } else if (key == "pdomapping") {
            uint64_t mappable = 0;
            if (parse_number(value, mappable)) {
                entry.flags.pdo_mappable = (mappable != 0);
            }
        } else if (key == "objecttype") {
            uint64_t ot = 0;
            if (parse_number(value, ot) && ot == 9) {
                // ARRAY object — subentries will follow
            }
        }
        // LowLimit / HighLimit: bounds — kept for future validation
    }

    flush_entry();
    return 0;
}

int ObjectDictionary::load_dcf(const std::string& filename, uint8_t node_id) {
    return load_eds(filename, node_id);  // DCF is a superset of EDS format
}

} // namespace canopen
