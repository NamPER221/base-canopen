/**
 * @file pdo.hpp
 * @brief PDO (Process Data Object) Protocol
 */

#ifndef CANOPEN_CO_PDO_PDO_HPP
#define CANOPEN_CO_PDO_PDO_HPP

#include <canopen/can/msg/message_factory.hpp>
#include <canopen/co/object_dictionary/object_dictionary.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <array>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>

namespace canopen {

// Forward declaration
class BusInterface;

/**
 * @brief PDO mapping configuration
 */
struct PDOMappingConfig {
    std::vector<PDOMappingEntry> entries;
    size_t data_size() const;
};

/**
 * @brief PDO Configuration
 */
class PDOConfiguration {
public:
    PDOConfiguration();

    // COB-ID
    uint32_t get_cob_id() const { return cob_id_; }
    void set_cob_id(uint32_t id) { cob_id_ = id; }

    // Transmission type
    PDOTransmissionType get_transmission_type() const { return transmission_type_; }
    void set_transmission_type(PDOTransmissionType type) { transmission_type_ = type; }

    // Inhibit time (100us units)
    uint16_t get_inhibit_time() const { return inhibit_time_; }
    void set_inhibit_time(uint16_t time_100us) { inhibit_time_ = time_100us; }

    // Event timer (ms)
    uint16_t get_event_timer() const { return event_timer_; }
    void set_event_timer(uint16_t time_ms) { event_timer_ = time_ms; }

    // Sync start value
    uint8_t get_sync_start_value() const { return sync_start_value_; }
    void set_sync_start_value(uint8_t value) { sync_start_value_ = value; }

    // Mapping
    const PDOMappingConfig& get_mapping() const { return mapping_; }
    PDOMappingConfig& get_mapping() { return mapping_; }
    void set_mapping(const PDOMappingConfig& mapping) { mapping_ = mapping; }
    void add_mapping_entry(uint16_t index, uint8_t subindex, uint8_t bit_length);
    void clear_mapping() { mapping_.entries.clear(); }

private:
    uint32_t cob_id_;
    PDOTransmissionType transmission_type_;
    uint16_t inhibit_time_;
    uint16_t event_timer_;
    uint8_t sync_start_value_;
    PDOMappingConfig mapping_;
};

/**
 * @brief TPDO (Transmit PDO)
 */
class TPDO {
public:
    /**
     * @brief Construct TPDO
     */
    TPDO(ObjectDictionary& od, uint8_t pdo_num);

    /**
     * @brief Configure PDO
     */
    void configure(const PDOConfiguration& config);

    /**
     * @brief Set bus interface (TPDO transmits — no RX route needed)
     */
    void set_bus(BusInterface* bus) { bus_ = bus; }
    void attach(BusInterface& bus) { bus_ = &bus; }
    void detach(BusInterface& ) { bus_ = nullptr; }

    /**
     * @brief Start PDO transmission
     */
    void start();

    /**
     * @brief Stop PDO transmission
     */
    void stop();

    /**
     * @brief Set transmission inhibited
     */
    bool is_transmission_inhibited() const { return inhibit_timer_active_; }

    /**
     * @brief Send PDO immediately
     */
    void send();

    /**
     * @brief Send PDO on SYNC
     */
    void send_sync(uint8_t sync_counter);

    /**
     * @brief Update mapped values from OD
     */
    void update_from_od();

    /**
     * @brief Set async send (event-driven)
     */
    void set_async(bool async) { async_send_ = async; }

    /**
     * @brief Check if active
     */
    bool is_active() const { return active_.load(); }

    /**
     * @brief Get PDO number
     */
    uint8_t get_pdo_num() const { return pdo_num_; }

    /**
     * @brief Handle received frame
     */
    void handle_frame(const CANFrame& frame);

    /**
     * @brief Callback on transmission
     */
    std::function<void()> on_transmit;

private:
    void build_frame();
    void inhibit_timer_loop();
    void event_timer_loop();
    void update_data();

    ObjectDictionary& od_;
    uint8_t pdo_num_;
    BusInterface* bus_{nullptr};

    PDOConfiguration config_;
    std::array<uint8_t, 64> tx_data_;
    size_t tx_data_len_{0};

    std::atomic<bool> active_{false};
    std::atomic<bool> async_send_{false};
    std::atomic<bool> inhibit_timer_active_{false};

    std::thread inhibit_thread_;
    std::thread event_thread_;
    std::atomic<bool> running_{false};

    uint8_t sync_counter_{0};
    bool pending_transmission_{false};

    std::chrono::steady_clock::time_point last_tx_time_;
};

/**
 * @brief RPDO (Receive PDO)
 */
class RPDO {
public:
    /**
     * @brief Construct RPDO
     */
    RPDO(ObjectDictionary& od, uint8_t pdo_num);

    /**
     * @brief Configure PDO
     */
    void configure(const PDOConfiguration& config);

    /**
     * @brief Set bus interface
     */
    void set_bus(BusInterface* bus) { bus_ = bus; }

    /**
     * @brief Register for automatic dispatch of RPDO frames on this bus
     */
    void attach(BusInterface& bus) {
        bus_ = &bus;
        if (config_.get_cob_id() != 0) {
            route_ = bus.add_route(config_.get_cob_id(), 0x7FF,
                                   [this](const CANFrame& f) { handle_frame(f); });
        }
    }

    /**
     * @brief Unregister from the bus
     */
    void detach(BusInterface& bus) {
        if (route_) bus.remove_route(route_);
        route_ = 0;
        bus_ = nullptr;
    }

    /**
     * @brief Start PDO reception
     */
    void start();

    /**
     * @brief Stop PDO reception
     */
    void stop();

    /**
     * @brief Handle received frame
     */
    void handle_frame(const CANFrame& frame);

    /**
     * @brief Update OD with received values
     */
    void update_od();

    /**
     * @brief Get last received data
     */
    std::span<const uint8_t> get_data() const { return {rx_data_.data(), rx_data_len_}; }

    /**
     * @brief Callback on reception
     */
    std::function<void(const void*, size_t)> on_receive;

    /**
     * @brief Check if active
     */
    bool is_active() const { return active_.load(); }

    /**
     * @brief Get PDO number
     */
    uint8_t get_pdo_num() const { return pdo_num_; }

private:
    void parse_frame();

    ObjectDictionary& od_;
    uint8_t pdo_num_;
    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_{0};

    PDOConfiguration config_;
    std::array<uint8_t, 64> rx_data_;
    size_t rx_data_len_{0};

    std::atomic<bool> active_{false};
};

// ==================== Inline Implementations ====================

inline size_t PDOMappingConfig::data_size() const {
    size_t total = 0;
    for (const auto& e : entries) {
        total += (e.bit_length + 7) / 8;
    }
    return total;
}

inline void PDOConfiguration::add_mapping_entry(uint16_t index, uint8_t subindex, uint8_t bit_length) {
    mapping_.entries.push_back({index, subindex, bit_length});
}

} // namespace canopen

#endif // CANOPEN_CO_PDO_PDO_HPP
