/**
 * @file nmt.hpp
 * @brief NMT (Network Management) Service
 */

#ifndef CANOPEN_CO_NMT_NMT_HPP
#define CANOPEN_CO_NMT_NMT_HPP

#include <canopen/can/msg/message_factory.hpp>
#include <canopen/co/object_dictionary/object_dictionary.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <map>
#include <chrono>

namespace canopen {

// Forward declaration
class BusInterface;

/**
 * @brief Node state info
 */
struct NodeStateInfo {
    NMTState state{NMTState::INITIALISING};
    std::chrono::steady_clock::time_point last_heartbeat;
    bool bootup_received{false};
    bool heartbeat_timeout{false};
    uint8_t consecutive_timeouts{0};
};

/**
 * @brief NMT Service for Master/Slave
 */
class NMTService {
public:
    /**
     * @brief Construct NMT Service
     * @param od Object Dictionary reference
     * @param bus Bus interface (may be set later via set_bus/attach)
     * @param node_id Local node ID (for heartbeat producer)
     */
    explicit NMTService(ObjectDictionary& od, BusInterface* bus = nullptr,
                        uint8_t node_id = 0);

    ~NMTService();

    // ==================== Configuration ====================

    void set_bus(BusInterface* bus) { bus_ = bus; }
    void set_node_id(uint8_t id) { node_id_ = id; }
    uint8_t get_node_id() const { return node_id_; }

    /**
     * @brief Register for automatic dispatch (NMT commands + heartbeats)
     */
    void attach(BusInterface& bus);

    /**
     * @brief Unregister from the bus
     */
    void detach(BusInterface& bus);

    // ==================== Lifecycle ====================

    void start();
    void stop();

    // ==================== Master Operations ====================

    /**
     * @brief Send NMT command to node
     * @param node_id Target node ID (0 = all)
     * @param cmd NMT command
     */
    void send_command(uint8_t node_id, NMTCommand cmd);

    /**
     * @brief Start heartbeat producer
     * @param interval_ms Heartbeat interval in milliseconds
     */
    void start_heartbeat_producer(uint16_t interval_ms);

    /**
     * @brief Stop heartbeat producer
     */
    void stop_heartbeat_producer();

    /**
     * @brief Add heartbeat consumer for node
     */
    void add_heartbeat_consumer(uint8_t node_id, uint16_t producer_time_ms);

    /**
     * @brief Remove heartbeat consumer
     */
    void remove_heartbeat_consumer(uint8_t node_id);

    /**
     * @brief Set heartbeat timeout
     */
    void set_heartbeat_timeout(uint8_t node_id, uint16_t timeout_ms);

    // ==================== State Management ====================

    /**
     * @brief Get current state
     */
    NMTState get_state() const { return state_.load(); }

    /**
     * @brief Set local state
     */
    void set_state(NMTState state);

    /**
     * @brief Get node state
     */
    NodeStateInfo get_node_state(uint8_t node_id) const;

    /**
     * @brief Get all node states
     */
    std::map<uint8_t, NodeStateInfo> get_all_node_states() const;

    // ==================== Callbacks ====================

    std::function<void(NMTState old_state, NMTState new_state)> on_state_change;
    std::function<void(uint8_t node_id, NMTState state)> on_node_state_change;
    std::function<void(uint8_t node_id)> on_bootup;
    std::function<void(uint8_t node_id)> on_heartbeat_timeout;
    std::function<void()> on_heartbeat_produced;

    // ==================== Frame Handling ====================

    void handle_frame(const CANFrame& frame);

private:
    void send_heartbeat();
    void handle_nmt_command(const CANFrame& frame);
    void handle_heartbeat(uint8_t node_id, NMTState state);
    void heartbeat_consumer_loop();

    ObjectDictionary& od_;
    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_nmt_{0};
    BusInterface::RouteHandle route_hb_{0};
    uint8_t node_id_{0};

    std::atomic<NMTState> state_{NMTState::INITIALISING};
    std::atomic<bool> running_{false};

    // Heartbeat producer
    std::thread heartbeat_producer_thread_;
    std::atomic<bool> heartbeat_producer_active_{false};
    uint16_t heartbeat_interval_ms_{0};

    // Heartbeat consumers
    std::map<uint8_t, NodeStateInfo> node_states_;
    mutable std::mutex node_states_mutex_;
    std::thread heartbeat_consumer_thread_;
    std::atomic<bool> consumer_running_{false};

    // Configuration
    uint16_t consumer_timeout_ms_{500};
};

} // namespace canopen

#endif // CANOPEN_CO_NMT_NMT_HPP
