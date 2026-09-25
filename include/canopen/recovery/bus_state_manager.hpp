/**
 * @file bus_state_manager.hpp
 * @brief CAN Bus State Manager with auto-recovery
 */

#ifndef CANOPEN_RECOVERY_BUS_STATE_MANAGER_HPP
#define CANOPEN_RECOVERY_BUS_STATE_MANAGER_HPP

#include <canopen/can/raw/socket_can.hpp>
#include <canopen/can/raw/bus_monitor.hpp>
#include <canopen/can/frame/frame.hpp>
#include <atomic>
#include <thread>
#include <functional>
#include <chrono>
#include <ctime>
#include <map>
#include <vector>
#include <optional>
#include <string>

namespace canopen {

// Forward declarations
enum class NMTState : uint8_t;

/**
 * @brief Bus State Manager with self-recovery
 */
class BusStateManager : public BusObserver {
public:
    /**
     * @brief Construct BusStateManager
     */
    BusStateManager(SocketCAN& can, BusMonitor& monitor);

    ~BusStateManager();

    // ==================== State Monitoring ====================

    /**
     * @brief Get current bus state
     */
    BusState get_state() const { return state_.load(); }

    /**
     * @brief Check if bus is operational
     */
    bool is_operational() const;

    /**
     * @brief Check if recovering
     */
    bool is_recovering() const { return recovering_.load(); }

    // ==================== Recovery Control ====================

    /**
     * @brief Enable/disable auto-recovery
     */
    void set_auto_recovery(bool enable, uint32_t restart_delay_ms = 1000);

    /**
     * @brief Manual recovery trigger
     */
    void recovery_now();

    /**
     * @brief Stop recovery
     */
    void stop_recovery();

    // ==================== State Preservation ====================

    /**
     * @brief Save current state
     */
    void save_state();

    /**
     * @brief Restore saved state
     */
    void restore_state();

    // ==================== Statistics ====================

    struct RecoveryStats {
        uint32_t total_recoveries{0};
        uint32_t successful_recoveries{0};
        uint32_t failed_recoveries{0};
        uint64_t total_downtime_ms{0};
        struct timespec last_recovery_time{0, 0};
        struct timespec last_bus_off_time{0, 0};
    };

    RecoveryStats get_stats() const;

    // ==================== Callbacks ====================

    std::function<void(BusState old_state, BusState new_state)> on_state_changed;
    std::function<void()> on_recovery_start;
    std::function<void(bool success)> on_recovery_complete;
    std::function<void()> on_permanent_failure;

    // BusObserver interface
    void on_error(const CANErrorFrame& error) override;
    void on_state_change(BusState old_state, BusState new_state) override;
    void on_recovery_attempt(int attempt_number) override;

private:
    void monitor_loop();
    void perform_recovery();
    void check_recovery_timeout();

    SocketCAN& can_;
    BusMonitor& monitor_;

    std::atomic<BusState> state_{BusState::UNKNOWN};
    std::atomic<bool> recovering_{false};
    std::atomic<bool> auto_recovery_{true};

    uint32_t restart_delay_ms_{1000};
    std::thread monitor_thread_;
    std::atomic<bool> running_{false};

    RecoveryStats stats_{};
    mutable std::mutex stats_mutex_;

    struct SavedState {
        std::chrono::steady_clock::time_point saved_at;
        // Would save: NMT state, OD values, pending TX queue
    };
    std::optional<SavedState> saved_state_;
    mutable std::mutex state_mutex_;
};

/**
 * @brief TX Queue with priority ordering
 */
class TXQueueManager {
public:
    TXQueueManager(size_t max_queue_size = 1024);

    /**
     * @brief Enqueue frame
     */
    bool enqueue(const CANFrame& frame, bool high_priority = false);

    /**
     * @brief Dequeue frame
     */
    std::optional<CANFrame> dequeue();

    /**
     * @brief Clear queue
     */
    void clear();

    /**
     * @brief Pause queuing
     */
    void pause();

    /**
     * @brief Resume queuing
     */
    void resume();

    /**
     * @brief Get pending frames for state preservation
     */
    struct PendingTXFrame {
        CANFrame frame;
        timespec timestamp;
        uint8_t retry_count;
        uint32_t can_id;
    };

    std::vector<PendingTXFrame> get_pending_frames() const;
    size_t restore_pending_frames(const std::vector<PendingTXFrame>& frames);

    /**
     * @brief Queue status
     */
    size_t size() const;
    bool is_empty() const;
    bool is_paused() const { return paused_.load(); }

private:
    size_t get_priority(const CANFrame& frame) const;

    std::vector<PendingTXFrame> queue_;
    std::vector<PendingTXFrame> high_priority_queue_;
    mutable std::mutex mutex_;
    std::atomic<bool> paused_{false};
    size_t max_size_;
};

/**
 * @brief Node Health Monitor
 */
class NodeHealthMonitor {
public:
    NodeHealthMonitor(void* bus);

    ~NodeHealthMonitor();

    /**
     * @brief Register node for monitoring
     */
    void register_node(uint8_t node_id, uint16_t heartbeat_time);

    /**
     * @brief Unregister node
     */
    void unregister_node(uint8_t node_id);

    /**
     * @brief Set expected NMT state for node
     */
    void set_expected_state(uint8_t node_id, NMTState state);

    /**
     * @brief Get node health info
     */
    struct NodeHealth {
        uint8_t node_id;
        NMTState current_state;
        NMTState expected_state;
        uint16_t heartbeat_time;
        bool timeout_occurred;
        uint8_t consecutive_timeouts;
        bool configuration_complete;
    };

    NodeHealth get_health(uint8_t node_id) const;

    /**
     * @brief Get all unhealthy nodes
     */
    std::vector<uint8_t> get_unhealthy_nodes() const;

    /**
     * @brief Reset node
     */
    void reset_node(uint8_t node_id);

    /**
     * @brief Callback on node error
     */
    std::function<void(uint8_t node_id, const std::string& error)> on_node_error;

    /**
     * @brief Callback on node recovery
     */
    std::function<void(uint8_t node_id)> on_node_recovered;

    /**
     * @brief Callback on state change
     */
    std::function<void(uint8_t node_id, NMTState old_state, NMTState new_state)> on_state_change;

private:
    void heartbeat_check_loop();

    void* bus_;
    std::map<uint8_t, NodeHealth> nodes_;
    mutable std::mutex nodes_mutex_;

    std::thread check_thread_;
    std::atomic<bool> running_{false};
};

/**
 * @brief Recovery Orchestrator
 */
class RecoveryOrchestrator {
public:
    /**
     * @brief Recovery Policy
     */
    struct RecoveryPolicy {
        bool auto_recover_bus{true};
        uint32_t bus_recovery_delay_ms{1000};
        uint32_t max_recovery_attempts{5};
        uint32_t node_timeout_ms{5000};
        bool preserve_tx_queue{true};
        bool restore_nmt_state{true};
        bool use_exponential_backoff{true};
        uint32_t initial_backoff_ms{1000};
        uint32_t max_backoff_ms{30000};
    };

    RecoveryOrchestrator(BusStateManager& bus_manager, TXQueueManager& tx_queue, void* nmt);

    /**
     * @brief Set recovery policy
     */
    void set_policy(const RecoveryPolicy& policy);
    RecoveryPolicy get_policy() const { return policy_; }

    /**
     * @brief Enable auto-recovery
     */
    void enable_auto_recovery();

    /**
     * @brief Disable auto-recovery
     */
    void disable_auto_recovery();

    /**
     * @brief Manual recovery
     */
    struct RecoveryResult {
        bool success;
        std::string error_message;
        std::chrono::milliseconds duration;
        std::vector<uint8_t> recovered_nodes;
    };

    RecoveryResult recover();
    RecoveryResult recover_node(uint8_t node_id);

    /**
     * @brief Status
     */
    bool is_recovering() const;
    enum class RecoveryPhase { IDLE, DETECTING, PRESERVING, WAITING, RECONNECTING, RESTORING, VERIFYING, COMPLETE };
    RecoveryPhase get_current_phase() const { return phase_.load(); }

    /**
     * @brief Callbacks
     */
    std::function<void(RecoveryPhase phase, float progress)> on_phase_change;
    std::function<void(const RecoveryResult& result)> on_recovery_complete;

private:
    RecoveryPolicy policy_;
    BusStateManager& bus_manager_;
    TXQueueManager& tx_queue_;
    void* nmt_;

    std::atomic<RecoveryPhase> phase_{RecoveryPhase::IDLE};
    std::atomic<float> progress_{0.0f};
    std::atomic<bool> recovery_active_{false};
    std::thread recovery_thread_;
    std::atomic<uint32_t> recovery_attempts_{0};
    uint32_t backoff_ms_{1000};
};

} // namespace canopen

#endif // CANOPEN_RECOVERY_BUS_STATE_MANAGER_HPP
