/**
 * @file filter_manager.cpp
 * @brief CAN Filter Manager implementation
 */

#include <canopen/can/raw/filter_manager.hpp>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace canopen {

void FilterManager::attach(int sock_fd) {
    std::lock_guard<std::mutex> lock(mutex_);
    sock_fd_ = sock_fd;
    apply_filters();
}

void FilterManager::detach() {
    std::lock_guard<std::mutex> lock(mutex_);
    sock_fd_ = -1;
    filters_.clear();
}

int FilterManager::add_filter(uint32_t can_id, uint32_t can_mask) {
    std::lock_guard<std::mutex> lock(mutex_);

    struct can_filter f;
    f.can_id = can_id;
    f.can_mask = can_mask;

    filters_.push_back(f);

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

int FilterManager::remove_filter(uint32_t can_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = std::remove_if(filters_.begin(), filters_.end(),
        [can_id](const struct can_filter& f) {
            return (f.can_id & ~CAN_EFF_FLAG) == (can_id & ~CAN_EFF_FLAG);
        });

    filters_.erase(it, filters_.end());

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

int FilterManager::add_extended_filter(uint32_t can_id, uint32_t can_mask) {
    std::lock_guard<std::mutex> lock(mutex_);

    struct can_filter f;
    f.can_id = can_id | CAN_EFF_FLAG;
    f.can_mask = can_mask | CAN_EFF_FLAG;

    filters_.push_back(f);

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

int FilterManager::set_filters(const std::vector<struct can_filter>& filters) {
    std::lock_guard<std::mutex> lock(mutex_);
    filters_ = filters;

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

int FilterManager::clear_all_filters() {
    std::lock_guard<std::mutex> lock(mutex_);
    filters_.clear();

    if (sock_fd_ >= 0) {
        // Set single null filter to reject all
        struct can_filter f = {0, 0};
        filters_.push_back(f);
        return apply_filters();
    }
    return 0;
}

int FilterManager::set_nmt_filter() {
    return add_filter(0x000, 0x7FF);
}

int FilterManager::set_sync_filter() {
    return add_filter(0x080, 0x7FF);
}

int FilterManager::set_pdo_filter(uint8_t node_id, uint8_t pdo_mask) {
    std::lock_guard<std::mutex> lock(mutex_);

    uint32_t base = node_id & 0x7F;

    if (pdo_mask & 0x01) {
        filters_.push_back({static_cast<uint32_t>(0x180 + base), 0x7FF});  // TPDO1
    }
    if (pdo_mask & 0x02) {
        filters_.push_back({static_cast<uint32_t>(0x200 + base), 0x7FF});  // RPDO1
    }
    if (pdo_mask & 0x04) {
        filters_.push_back({static_cast<uint32_t>(0x280 + base), 0x7FF});  // TPDO2
    }
    if (pdo_mask & 0x08) {
        filters_.push_back({static_cast<uint32_t>(0x300 + base), 0x7FF});  // RPDO2
    }
    if (pdo_mask & 0x10) {
        filters_.push_back({static_cast<uint32_t>(0x380 + base), 0x7FF});  // TPDO3
    }
    if (pdo_mask & 0x20) {
        filters_.push_back({static_cast<uint32_t>(0x400 + base), 0x7FF});  // RPDO3
    }
    if (pdo_mask & 0x40) {
        filters_.push_back({static_cast<uint32_t>(0x480 + base), 0x7FF});  // TPDO4
    }
    if (pdo_mask & 0x80) {
        filters_.push_back({static_cast<uint32_t>(0x500 + base), 0x7FF});  // RPDO4
    }

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

int FilterManager::set_sdo_filter(uint8_t node_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    uint32_t base = node_id & 0x7F;

    filters_.push_back({static_cast<uint32_t>(0x580 + base), 0x7FF});  // TSDO
    filters_.push_back({static_cast<uint32_t>(0x600 + base), 0x7FF});  // RSDO

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

int FilterManager::set_heartbeat_filter() {
    std::lock_guard<std::mutex> lock(mutex_);

    filters_.push_back({0x700, 0x780});  // Match 0x700-0x77F

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

int FilterManager::set_emcy_filter(uint8_t node_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (node_id == 0xFF) {
        filters_.push_back({0x100, 0x700});  // Match 0x100-0x1FF
    } else {
        filters_.push_back({static_cast<uint32_t>(0x080 + node_id), 0x7FF});
    }

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

int FilterManager::set_all_nodes_filter() {
    std::lock_guard<std::mutex> lock(mutex_);

    filters_.push_back({0x000, 0x7FF});    // NMT
    filters_.push_back({0x080, 0x7FF});    // SYNC
    filters_.push_back({0x100, 0x700});    // EMCY
    filters_.push_back({0x180, 0xF80});    // TPDO1-4
    filters_.push_back({0x200, 0xF80});    // RPDO1-4
    filters_.push_back({0x380, 0xF80});    // TPDO3-4
    filters_.push_back({0x580, 0xF80});    // TSDO
    filters_.push_back({0x600, 0xF80});    // RSDO
    filters_.push_back({0x700, 0x780});    // Heartbeat
    filters_.push_back({0x7E4, 0x7F4});    // LSS

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

int FilterManager::set_master_filter() {
    std::lock_guard<std::mutex> lock(mutex_);

    filters_.push_back({0x580, 0xF80});    // TSDO
    filters_.push_back({0x700, 0x780});    // Heartbeat
    filters_.push_back({0x100, 0x700});    // EMCY

    if (sock_fd_ >= 0) {
        return apply_filters();
    }
    return 0;
}

std::vector<struct can_filter> FilterManager::get_active_filters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return filters_;
}

size_t FilterManager::get_filter_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return filters_.size();
}

bool FilterManager::has_filter(uint32_t can_id) const {
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& f : filters_) {
        if ((can_id & f.can_mask) == (f.can_id & f.can_mask)) {
            return true;
        }
    }
    return false;
}

int FilterManager::apply_filters() {
    if (sock_fd_ < 0) return 0;

    if (filters_.empty()) {
        return 0;
    }

    return setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_FILTER,
                       filters_.data(), filters_.size() * sizeof(struct can_filter));
}

uint32_t FilterManager::make_pdo_cobid(uint8_t node_id, uint8_t base_id) {
    return (base_id << 7) | (node_id & 0x7F);
}

} // namespace canopen
