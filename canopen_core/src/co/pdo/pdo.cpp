/**
 * @file pdo.cpp
 * @brief PDO Protocol implementation
 */

#include <canopen/co/pdo/pdo.hpp>
#include <cstring>

namespace canopen {

// ==================== PDO Configuration ====================

PDOConfiguration::PDOConfiguration()
    : cob_id_(0),
      transmission_type_(PDOTransmissionType::SYNCHRONOUS_ACYCLIC),
      inhibit_time_(0),
      event_timer_(0),
      sync_start_value_(0) {
}

// ==================== TPDO ====================

TPDO::TPDO(ObjectDictionary& od, uint8_t pdo_num)
    : od_(od), pdo_num_(pdo_num) {
}

void TPDO::configure(const PDOConfiguration& config) {
    config_ = config;
    update_data();
}

void TPDO::start() {
    if (active_.load()) return;

    active_.store(true);
    running_.store(true);

    // Start event timer if configured
    if (config_.get_event_timer() > 0) {
        event_thread_ = std::thread(&TPDO::event_timer_loop, this);
    }
}

void TPDO::stop() {
    running_.store(false);
    active_.store(false);

    if (event_thread_.joinable()) {
        event_thread_.join();
    }
    if (inhibit_thread_.joinable()) {
        inhibit_thread_.join();
    }
}

void TPDO::send() {
    if (!active_.load() || !bus_) return;

    // Check inhibit time
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        now - last_tx_time_).count();

    if (config_.get_inhibit_time() > 0) {
        uint64_t inhibit_us = config_.get_inhibit_time() * 100;  // 100us units
        if (elapsed < static_cast<long>(inhibit_us)) {
            return;
        }
    }

    build_frame();

    CANFrame frame;
    frame.set_id(config_.get_cob_id());
    frame.set_len(static_cast<uint8_t>(tx_data_len_));
    memcpy(frame.data(), tx_data_.data(), tx_data_len_);

    bus_->send(frame);

    last_tx_time_ = std::chrono::steady_clock::now();

    if (on_transmit) {
        on_transmit();
    }
}

void TPDO::send_sync(uint8_t sync_counter) {
    if (!active_.load()) return;

    PDOTransmissionType type = config_.get_transmission_type();

    // Check transmission type
    if (type == PDOTransmissionType::ASYNCHRONOUS ||
        type == PDOTransmissionType::ASYNCHRONOUS_SPECIFIC ||
        type == PDOTransmissionType::ASYNCHRONOUS_RTR_SYNC) {
        return;  // Not for sync transmission
    }

    // Synchronous cyclic (type value 1-240)
    if (static_cast<uint8_t>(type) >= 1 && static_cast<uint8_t>(type) <= 240) {
        uint8_t period = static_cast<uint8_t>(type);
        if (period == 0) period = 1;

        if (sync_counter % period == 0) {
            update_data();
            send();
        }
    }
    // Synchronous acyclic - send on every sync
    else if (type == PDOTransmissionType::SYNCHRONOUS_ACYCLIC) {
        update_data();
        send();
    }
}

void TPDO::update_from_od() {
    update_data();
}

void TPDO::handle_frame(const CANFrame& frame) {
    (void)frame;
    // TPDO receives, so this would be for RTR handling
}

void TPDO::update_data() {
    uint8_t offset = 0;

    for (const auto& entry : config_.get_mapping().entries) {
        ObjectEntry* obj = od_.get_object(entry.object_index, entry.subindex);
        if (!obj) continue;

        uint8_t buffer[8];
        size_t size = sizeof(buffer);

        od_.read(entry.object_index, entry.subindex, buffer, size);

        // Copy data (assuming byte-aligned for simplicity)
        uint8_t bytes = (entry.bit_length + 7) / 8;
        if (offset + bytes <= sizeof(tx_data_)) {
            memcpy(tx_data_.data() + offset, buffer, bytes);
            offset += bytes;
        }
    }

    tx_data_len_ = offset;
}

void TPDO::build_frame() {
    // Frame already built in update_data
}

void TPDO::inhibit_timer_loop() {
    while (running_.load()) {
        if (config_.get_inhibit_time() > 0) {
            inhibit_timer_active_.store(true);
            std::this_thread::sleep_for(std::chrono::microseconds(
                config_.get_inhibit_time() * 100));
            inhibit_timer_active_.store(false);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void TPDO::event_timer_loop() {
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(
            config_.get_event_timer()));

        if (running_.load() && active_.load()) {
            update_data();
            send();
        }
    }
}

// ==================== RPDO ====================

RPDO::RPDO(ObjectDictionary& od, uint8_t pdo_num)
    : od_(od), pdo_num_(pdo_num) {
}

void RPDO::configure(const PDOConfiguration& config) {
    config_ = config;
}

void RPDO::start() {
    active_.store(true);
}

void RPDO::stop() {
    active_.store(false);
}

void RPDO::handle_frame(const CANFrame& frame) {
    if (!active_.load()) return;

    if (frame.id() != config_.get_cob_id()) return;

    // Copy data
    rx_data_len_ = std::min(static_cast<size_t>(frame.len()), rx_data_.size());
    memcpy(rx_data_.data(), frame.data(), rx_data_len_);

    // Update object dictionary
    update_od();

    // Call callback
    if (on_receive) {
        on_receive(rx_data_.data(), rx_data_len_);
    }
}

void RPDO::update_od() {
    uint8_t offset = 0;

    for (const auto& entry : config_.get_mapping().entries) {
        ObjectEntry* obj = od_.get_object(entry.object_index, entry.subindex);
        if (!obj) continue;

        uint8_t bytes = (entry.bit_length + 7) / 8;
        if (offset + bytes <= rx_data_len_) {
            od_.write(entry.object_index, entry.subindex,
                      rx_data_.data() + offset, bytes);
            offset += bytes;
        }
    }
}

void RPDO::parse_frame() {
    // Already parsed in handle_frame
}

} // namespace canopen
