/**
 * @file heartbeat.hpp
 * @brief Heartbeat Producer/Consumer
 */

#ifndef CANOPEN_CO_HEARTBEAT_HEARTBEAT_HPP
#define CANOPEN_CO_HEARTBEAT_HEARTBEAT_HPP

#include <canopen/can/msg/message_factory.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <map>
#include <functional>

namespace canopen {

// Forward declaration
class BusInterface;

/**
 * @brief Heartbeat Producer
 */
class HeartbeatProducer {
public:
    HeartbeatProducer();

    void set_bus(BusInterface* bus) { bus_ = bus; }
    void attach(BusInterface& bus) { bus_ = &bus; }
    void detach(BusInterface& ) { bus_ = nullptr; }
    void set_node_id(uint8_t id);
    void set_state(NMTState state);

    /**
     * @brief Start producing heartbeat
     * @param interval_ms Heartbeat interval in milliseconds
     */
    void start(uint16_t interval_ms);

    /**
     * @brief Stop producing heartbeat
     */
    void stop();

    /**
     * @brief Check if running
     */
    bool is_running() const { return running_.load(); }

private:
    void producer_loop();

    BusInterface* bus_{nullptr};
    uint8_t node_id_{0};
    NMTState state_{NMTState::INITIALISING};
    uint16_t interval_ms_{0};

    std::atomic<bool> running_{false};
    std::thread producer_thread_;
};

/**
 * @brief Heartbeat Consumer
 */
class HeartbeatConsumer {
public:
    HeartbeatConsumer();

    void set_bus(BusInterface* bus) { bus_ = bus; }
    void attach(BusInterface& bus) {
        bus_ = &bus;
        route_ = routes::heartbeat_all(bus, [this](const CANFrame& f) {
            handle_frame(f);
        });
    }
    void detach(BusInterface& bus) {
        if (route_) bus.remove_route(route_);
        route_ = 0;
        bus_ = nullptr;
    }

    /**
     * @brief Add producer to monitor
     */
    void add_producer(uint8_t node_id, uint16_t heartbeat_time);

    /**
     * @brief Remove producer
     */
    void remove_producer(uint8_t node_id);

    /**
     * @brief Set consumer heartbeat time
     */
    void set_consumer_heartbeat_time(uint16_t time_ms);

    /**
     * @brief Handle heartbeat frame
     */
    void handle_frame(const CANFrame& frame);

    /**
     * @brief Get producer status
     */
    struct ProducerStatus {
        NMTState state;
        std::chrono::steady_clock::time_point last_heartbeat;
        bool in_bootup;
        bool lost;
    };

    ProducerStatus get_producer_status(uint8_t node_id) const;

    /**
     * @brief Check if producer is alive
     */
    bool is_producer_alive(uint8_t node_id) const;

    /**
     * @brief Callback on producer state change
     */
    std::function<void(uint8_t node_id, NMTState state)> on_producer_state;

    /**
     * @brief Callback on producer timeout
     */
    std::function<void(uint8_t node_id)> on_producer_timeout;

private:
    void consumer_loop();
    void check_timeouts();

    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_{0};

    struct ProducerInfo {
        uint16_t heartbeat_time;
        NMTState state{NMTState::INITIALISING};
        std::chrono::steady_clock::time_point last_heartbeat;
        bool in_bootup{true};
        bool lost{false};
    };

    std::map<uint8_t, ProducerInfo> producers_;
    uint16_t consumer_time_ms_{500};

    std::thread consumer_thread_;
    std::atomic<bool> running_{false};
};

} // namespace canopen

#endif // CANOPEN_CO_HEARTBEAT_HEARTBEAT_HPP
