/**
 * @file bus_monitor.hpp
 * @brief CAN Bus Monitor and Error Handler
 */

#ifndef CANOPEN_CAN_RAW_BUS_MONITOR_HPP
#define CANOPEN_CAN_RAW_BUS_MONITOR_HPP

#include <canopen/can/raw/socket_can.hpp>
#include <chrono>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>
#include <vector>
#include <ctime>
#include <string>

namespace canopen {

/**
 * @brief Bus metrics structure
 */
struct BusMetrics {
    uint64_t tx_frames{0};
    uint64_t rx_frames{0};
    uint64_t tx_errors{0};
    uint64_t rx_errors{0};
    uint64_t bus_errors{0};
    uint64_t arbitration_lost{0};
    uint8_t tx_error_counter{0};
    uint8_t rx_error_counter{0};
    BusState state{BusState::UNKNOWN};
    struct timespec last_error_time{0, 0};
    struct timespec last_tx_time{0, 0};
    struct timespec last_rx_time{0, 0};
};

/**
 * @brief Bus observer interface
 */
class BusObserver {
public:
    virtual ~BusObserver() = default;
    virtual void on_error(const CANErrorFrame& error) = 0;
    virtual void on_state_change(BusState old_state, BusState new_state) = 0;
    virtual void on_recovery_attempt(int attempt_number) = 0;
};

/**
 * @brief CAN Bus Monitor
 *
 * Monitors bus state, error counters, and provides callbacks
 * for state changes and error conditions.
 */
class BusMonitor {
public:
    BusMonitor();
    ~BusMonitor();

    // Non-copyable
    BusMonitor(const BusMonitor&) = delete;
    BusMonitor& operator=(const BusMonitor&) = delete;

    // ==================== Lifecycle ====================

    /**
     * @brief Start monitoring on interface
     * @param interface CAN interface name
     */
    void start(const std::string& interface);

    /**
     * @brief Stop monitoring
     */
    void stop();

    /**
     * @brief Check if monitoring is active
     */
    bool is_active() const { return active_.load(); }

    // ==================== Metrics ====================

    /**
     * @brief Get current bus metrics
     */
    BusMetrics get_metrics() const;

    /**
     * @brief Reset all metrics
     */
    void reset_metrics();

    /**
     * @brief Get time since last frame
     */
    std::chrono::milliseconds time_since_last_frame() const;

    // ==================== Callbacks ====================

    /**
     * @brief Set error callback
     */
    void set_error_callback(std::function<void(const CANErrorFrame&)> callback);

    /**
     * @brief Set state change callback
     */
    void set_state_change_callback(std::function<void(BusState, BusState)> callback);

    /**
     * @brief Add observer
     */
    void add_observer(BusObserver* observer);

    /**
     * @brief Remove observer
     */
    void remove_observer(BusObserver* observer);

    // ==================== Error Thresholds ====================

    /**
     * @brief Set error warning threshold
     * @param tx_threshold TX error counter threshold
     * @param rx_threshold RX error counter threshold
     */
    void set_warning_threshold(uint8_t tx_threshold, uint8_t rx_threshold);

    /**
     * @brief Set error passive threshold (typically 128)
     */
    void set_error_passive_threshold(uint8_t threshold);

    /**
     * @brief Check if in error warning state
     */
    bool is_error_warning() const;

    /**
     * @brief Check if in error passive state
     */
    bool is_error_passive() const;

    /**
     * @brief Check if in bus-off state
     */
    bool is_bus_off() const;

    /**
     * @brief Check if recovering from bus-off
     */
    bool is_recovering() const { return recovering_.load(); }

    // ==================== Statistics ====================

    /**
     * @brief Get frames per second (TX)
     */
    double get_tx_rate() const;

    /**
     * @brief Get frames per second (RX)
     */
    double get_rx_rate() const;

    /**
     * @brief Get bus load estimate (0-100%)
     */
    double get_bus_load() const;

    /**
     * @brief Get uptime
     */
    std::chrono::seconds get_uptime() const;

    // Frame counting
    void increment_tx_frames() {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.tx_frames++;
        clock_gettime(CLOCK_MONOTONIC, &metrics_.last_tx_time);
    }

    void increment_rx_frames() {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.rx_frames++;
        clock_gettime(CLOCK_MONOTONIC, &metrics_.last_rx_time);
    }

private:
    void monitoring_loop();
    void update_metrics(const CANErrorFrame& error);
    void notify_observers(const CANErrorFrame& error);
    void notify_state_change(BusState old_state, BusState new_state);

    std::string interface_;
    std::atomic<bool> active_{false};
    std::atomic<bool> recovering_{false};
    std::thread monitor_thread_;

    BusMetrics metrics_;
    mutable std::mutex metrics_mutex_;

    uint8_t warning_tx_threshold_{96};
    uint8_t warning_rx_threshold_{96};
    uint8_t passive_threshold_{128};

    std::function<void(const CANErrorFrame&)> error_callback_;
    std::function<void(BusState, BusState)> state_change_callback_;
    std::vector<BusObserver*> observers_;
    mutable std::mutex observers_mutex_;

    struct timespec start_time_{0, 0};
    std::atomic<BusState> last_state_{BusState::UNKNOWN};
};

} // namespace canopen

#endif // CANOPEN_CAN_RAW_BUS_MONITOR_HPP
