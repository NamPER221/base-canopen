/**
 * @file sync.hpp
 * @brief SYNC Producer/Consumer
 */

#ifndef CANOPEN_CO_SYNC_SYNC_HPP
#define CANOPEN_CO_SYNC_SYNC_HPP

#include <canopen/can/msg/message_factory.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>

namespace canopen {

// Forward declaration
class BusInterface;

/**
 * @brief SYNC Service
 */
class SYNCService {
public:
    SYNCService();

    void set_bus(BusInterface* bus) { bus_ = bus; }
    void attach(BusInterface& bus) {
        bus_ = &bus;
        route_ = routes::sync(bus, [this](const CANFrame& f) {
            handle_frame(f);
        });
    }
    void detach(BusInterface& bus) {
        if (route_) bus.remove_route(route_);
        route_ = 0;
        bus_ = nullptr;
    }

    /**
     * @brief Start producer mode
     * @param cycle_period_us SYNC cycle period in microseconds
     */
    void start_producer(uint16_t cycle_period_us);

    /**
     * @brief Stop producer
     */
    void stop_producer();

    /**
     * @brief Start consumer mode
     */
    void start_consumer();

    /**
     * @brief Stop consumer
     */
    void stop_consumer();

    /**
     * @brief Get current counter
     */
    uint8_t get_counter() const { return counter_; }

    /**
     * @brief Set counter (for consumer)
     */
    void set_counter(uint8_t counter) { counter_ = counter; }

    /**
     * @brief Handle SYNC frame
     */
    void handle_frame(const CANFrame& frame);

    /**
     * @brief Check if producer is running
     */
    bool is_producer_running() const { return producer_running_.load(); }

    /**
     * @brief Check if consumer is running
     */
    bool is_consumer_running() const { return consumer_running_.load(); }

    /**
     * @brief Set consumer timeout
     */
    void set_consumer_timeout(uint32_t timeout_ms);

    /**
     * @brief Callback on SYNC received
     */
    std::function<void(uint8_t counter)> on_sync;

    /**
     * @brief Callback on SYNC lost
     */
    std::function<void()> on_sync_lost;

private:
    void producer_loop();
    void consumer_loop();

    void send_sync();
    void handle_sync_frame(const CANFrame& frame);

    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_{0};

    // Producer
    bool is_producer_{false};
    uint16_t cycle_period_us_{10000};  // 10ms default
    std::atomic<bool> producer_running_{false};
    std::thread producer_thread_;

    // Consumer
    bool is_consumer_{false};
    std::atomic<uint8_t> counter_{0};
    std::atomic<uint8_t> expected_counter_{0};
    bool counter_overflow_{false};

    std::atomic<bool> consumer_running_{false};
    std::thread consumer_thread_;

    std::chrono::steady_clock::time_point last_sync_time_;
    uint32_t consumer_timeout_ms_{100};

    std::atomic<bool> sync_received_{false};
};

} // namespace canopen

#endif // CANOPEN_CO_SYNC_SYNC_HPP
