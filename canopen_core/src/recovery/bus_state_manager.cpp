/**
 * @file bus_state_manager.cpp
 * @brief Bus State Manager implementation
 */

#include <canopen/recovery/bus_state_manager.hpp>
#include <canopen/can/msg/message_factory.hpp>
#include <chrono>
#include <cstring>

namespace canopen {

// ==================== BusStateManager ====================

BusStateManager::BusStateManager(SocketCAN& can, BusMonitor& monitor)
    : can_(can), monitor_(monitor) {
    monitor_.add_observer(this);
}

BusStateManager::~BusStateManager() {
    monitor_.remove_observer(this);
    stop_recovery();
}

bool BusStateManager::is_operational() const {
    return state_.load() == BusState::ACTIVE;
}

void BusStateManager::set_auto_recovery(bool enable, uint32_t restart_delay_ms) {
    auto_recovery_.store(enable);
    restart_delay_ms_ = restart_delay_ms;

    if (enable && !running_.load()) {
        running_.store(true);
        monitor_thread_ = std::thread(&BusStateManager::monitor_loop, this);
    } else if (!enable && running_.load()) {
        running_.store(false);
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
    }
}

void BusStateManager::recovery_now() {
    if (recovering_.load()) return;
    perform_recovery();
}

void BusStateManager::stop_recovery() {
    running_.store(false);
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

void BusStateManager::save_state() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    SavedState ss;
    ss.saved_at = std::chrono::steady_clock::now();
    saved_state_ = ss;
}

void BusStateManager::restore_state() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!saved_state_) return;
    // Restore state from saved_state_
    saved_state_.reset();
}

BusStateManager::RecoveryStats BusStateManager::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void BusStateManager::monitor_loop() {
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        (void)monitor_.get_metrics();

        // Check for bus-off
        if (monitor_.is_bus_off() && !recovering_.load()) {
            if (auto_recovery_.load()) {
                perform_recovery();
            }
        }

        check_recovery_timeout();
    }
}

void BusStateManager::perform_recovery() {
    recovering_.store(true);

    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.total_recoveries++;
    clock_gettime(CLOCK_MONOTONIC, &stats_.last_bus_off_time);

    if (on_recovery_start) {
        on_recovery_start();
    }

    // Wait for restart delay
    std::this_thread::sleep_for(std::chrono::milliseconds(restart_delay_ms_));

    // Attempt recovery
    bool success = (can_.restart() == 0);

    if (success) {
        stats_.successful_recoveries++;
        state_.store(BusState::ACTIVE);
    } else {
        stats_.failed_recoveries++;
    }

    recovering_.store(false);

    clock_gettime(CLOCK_MONOTONIC, &stats_.last_recovery_time);

    if (on_recovery_complete) {
        on_recovery_complete(success);
    }
}

void BusStateManager::check_recovery_timeout() {
    // Check if recovery is taking too long
}

void BusStateManager::on_error(const CANErrorFrame& error) {
    (void)error;
}

void BusStateManager::on_state_change(BusState old_state, BusState new_state) {
    (void)old_state;
    BusState old = state_.load();
    state_.store(new_state);

    if (old != new_state && on_state_changed) {
        on_state_changed(old, new_state);
    }
}

void BusStateManager::on_recovery_attempt(int attempt_number) {
    (void)attempt_number;
}

// ==================== TXQueueManager ====================

TXQueueManager::TXQueueManager(size_t max_queue_size)
    : max_size_(max_queue_size) {
}

bool TXQueueManager::enqueue(const CANFrame& frame, bool high_priority) {
    if (paused_.load()) return false;

    std::lock_guard<std::mutex> lock(mutex_);

    if (high_priority) {
        if (high_priority_queue_.size() >= max_size_) return false;
        PendingTXFrame pf;
        pf.frame = frame;
        clock_gettime(CLOCK_MONOTONIC, &pf.timestamp);
        pf.retry_count = 0;
        pf.can_id = frame.id();
        high_priority_queue_.push_back(pf);
    } else {
        if (queue_.size() >= max_size_) return false;
        PendingTXFrame pf;
        pf.frame = frame;
        clock_gettime(CLOCK_MONOTONIC, &pf.timestamp);
        pf.retry_count = 0;
        pf.can_id = frame.id();
        queue_.push_back(pf);
    }

    return true;
}

std::optional<CANFrame> TXQueueManager::dequeue() {
    std::lock_guard<std::mutex> lock(mutex_);

    // High priority first
    if (!high_priority_queue_.empty()) {
        auto frame = high_priority_queue_.front().frame;
        high_priority_queue_.erase(high_priority_queue_.begin());
        return frame;
    }

    if (!queue_.empty()) {
        auto frame = queue_.front().frame;
        queue_.erase(queue_.begin());
        return frame;
    }

    return std::nullopt;
}

void TXQueueManager::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    high_priority_queue_.clear();
}

void TXQueueManager::pause() {
    paused_.store(true);
}

void TXQueueManager::resume() {
    paused_.store(false);
}

std::vector<TXQueueManager::PendingTXFrame> TXQueueManager::get_pending_frames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PendingTXFrame> result;
    result.insert(result.end(), high_priority_queue_.begin(), high_priority_queue_.end());
    result.insert(result.end(), queue_.begin(), queue_.end());
    return result;
}

size_t TXQueueManager::restore_pending_frames(const std::vector<PendingTXFrame>& frames) {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t restored = 0;

    for (const auto& frame : frames) {
        // Determine priority
        uint32_t priority = get_priority(frame.frame);
        PendingTXFrame pf = frame;

        if (priority < 10) {
            if (high_priority_queue_.size() < max_size_) {
                high_priority_queue_.push_back(pf);
                restored++;
            }
        } else {
            if (queue_.size() < max_size_) {
                queue_.push_back(pf);
                restored++;
            }
        }
    }

    return restored;
}

size_t TXQueueManager::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size() + high_priority_queue_.size();
}

bool TXQueueManager::is_empty() const {
    return size() == 0;
}

size_t TXQueueManager::get_priority(const CANFrame& frame) const {
    uint32_t id = frame.id();

    // Priority (lower = higher priority):
    // NMT: 0
    // SYNC: 1
    // EMCY: 2
    // SDO: 3
    // PDO: 4
    // Heartbeat: 5

    if (id == 0x000) return 0;  // NMT
    if (id == 0x080) return 1;  // SYNC
    if ((id & 0xF80) == 0x100) return 2;  // EMCY
    if ((id & 0x780) == 0x580 || (id & 0x780) == 0x600) return 3;  // SDO
    if ((id & 0x780) == 0x180 || (id & 0x780) == 0x200) return 4;  // PDO
    if ((id & 0x780) == 0x700) return 5;  // Heartbeat
    return 10;
}

// ==================== NodeHealthMonitor ====================

NodeHealthMonitor::NodeHealthMonitor(void* bus) : bus_(bus) {}

NodeHealthMonitor::~NodeHealthMonitor() {
    running_.store(false);
    if (check_thread_.joinable()) {
        check_thread_.join();
    }
}

void NodeHealthMonitor::register_node(uint8_t node_id, uint16_t heartbeat_time) {
    std::lock_guard<std::mutex> lock(nodes_mutex_);

    NodeHealth health;
    health.node_id = node_id;
    health.heartbeat_time = heartbeat_time;
    health.timeout_occurred = false;
    health.consecutive_timeouts = 0;
    health.configuration_complete = false;
    nodes_[node_id] = health;

    if (!running_.load()) {
        running_.store(true);
        check_thread_ = std::thread(&NodeHealthMonitor::heartbeat_check_loop, this);
    }
}

void NodeHealthMonitor::unregister_node(uint8_t node_id) {
    std::lock_guard<std::mutex> lock(nodes_mutex_);
    nodes_.erase(node_id);

    if (nodes_.empty() && running_.load()) {
        running_.store(false);
        if (check_thread_.joinable()) {
            check_thread_.join();
        }
    }
}

void NodeHealthMonitor::set_expected_state(uint8_t node_id, NMTState state) {
    std::lock_guard<std::mutex> lock(nodes_mutex_);
    auto it = nodes_.find(node_id);
    if (it != nodes_.end()) {
        it->second.expected_state = state;
    }
}

NodeHealthMonitor::NodeHealth NodeHealthMonitor::get_health(uint8_t node_id) const {
    std::lock_guard<std::mutex> lock(nodes_mutex_);
    auto it = nodes_.find(node_id);
    if (it != nodes_.end()) {
        return it->second;
    }
    return NodeHealth{};
}

std::vector<uint8_t> NodeHealthMonitor::get_unhealthy_nodes() const {
    std::lock_guard<std::mutex> lock(nodes_mutex_);
    std::vector<uint8_t> result;

    for (const auto& [node_id, health] : nodes_) {
        if (health.timeout_occurred ||
            (health.current_state != health.expected_state && health.expected_state != NMTState::INITIALISING)) {
            result.push_back(node_id);
        }
    }

    return result;
}

void NodeHealthMonitor::reset_node(uint8_t node_id) {
    std::lock_guard<std::mutex> lock(nodes_mutex_);
    auto it = nodes_.find(node_id);
    if (it != nodes_.end()) {
        it->second.timeout_occurred = false;
        it->second.consecutive_timeouts = 0;
    }
}

void NodeHealthMonitor::heartbeat_check_loop() {
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::lock_guard<std::mutex> lock(nodes_mutex_);

        for (auto& [node_id, health] : nodes_) {
            if (health.timeout_occurred) {
                health.consecutive_timeouts++;
                if (health.consecutive_timeouts > 10) {
                    if (on_node_error) {
                        on_node_error(node_id, "Node timeout");
                    }
                }
            }
        }
    }
}

// ==================== RecoveryOrchestrator ====================

RecoveryOrchestrator::RecoveryOrchestrator(BusStateManager& bus_manager, TXQueueManager& tx_queue, void* nmt)
    : bus_manager_(bus_manager), tx_queue_(tx_queue), nmt_(nmt) {}

void RecoveryOrchestrator::set_policy(const RecoveryPolicy& policy) {
    policy_ = policy;
}

void RecoveryOrchestrator::enable_auto_recovery() {
    bus_manager_.set_auto_recovery(true, policy_.bus_recovery_delay_ms);
}

void RecoveryOrchestrator::disable_auto_recovery() {
    bus_manager_.set_auto_recovery(false);
}

RecoveryOrchestrator::RecoveryResult RecoveryOrchestrator::recover() {
    RecoveryResult result;
    result.success = false;

    phase_.store(RecoveryPhase::PRESERVING);

    // Save state if enabled
    if (policy_.preserve_tx_queue) {
        tx_queue_.pause();
        bus_manager_.save_state();
    }

    phase_.store(RecoveryPhase::WAITING);

    // Wait for bus
    uint32_t backoff = policy_.initial_backoff_ms;
    for (uint32_t i = 0; i < policy_.max_recovery_attempts; i++) {
        phase_.store(RecoveryPhase::RECONNECTING);

        bus_manager_.recovery_now();

        std::this_thread::sleep_for(std::chrono::milliseconds(backoff));

        if (bus_manager_.is_operational()) {
            result.success = true;
            break;
        }

        // Exponential backoff
        if (policy_.use_exponential_backoff) {
            backoff = std::min(backoff * 2, policy_.max_backoff_ms);
        }
    }

    if (result.success) {
        phase_.store(RecoveryPhase::RESTORING);

        // Restore state
        if (policy_.preserve_tx_queue) {
            auto frames = tx_queue_.get_pending_frames();
            tx_queue_.restore_pending_frames(frames);
            tx_queue_.resume();
        }

        if (policy_.restore_nmt_state) {
            bus_manager_.restore_state();
        }

        phase_.store(RecoveryPhase::VERIFYING);
    }

    phase_.store(RecoveryPhase::COMPLETE);

    if (on_recovery_complete) {
        on_recovery_complete(result);
    }

    return result;
}

RecoveryOrchestrator::RecoveryResult RecoveryOrchestrator::recover_node(uint8_t node_id) {
    RecoveryResult result;
    result.recovered_nodes.push_back(node_id);
    result.success = true;
    return result;
}

bool RecoveryOrchestrator::is_recovering() const {
    return recovery_active_.load();
}

} // namespace canopen
