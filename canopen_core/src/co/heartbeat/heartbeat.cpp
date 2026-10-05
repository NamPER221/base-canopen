/**
 * @file heartbeat.cpp
 * @brief Heartbeat implementation
 */

#include <canopen/co/heartbeat/heartbeat.hpp>
#include <chrono>

namespace canopen {

// ==================== Heartbeat Producer ====================

HeartbeatProducer::HeartbeatProducer() = default;

void HeartbeatProducer::set_node_id(uint8_t id) {
    node_id_ = id;
}

void HeartbeatProducer::set_state(NMTState state) {
    state_ = state;
}

void HeartbeatProducer::start(uint16_t interval_ms) {
    if (running_.load()) {
        stop();
    }

    interval_ms_ = interval_ms;
    running_.store(true);

    producer_thread_ = std::thread(&HeartbeatProducer::producer_loop, this);
}

void HeartbeatProducer::stop() {
    running_.store(false);

    if (producer_thread_.joinable()) {
        producer_thread_.join();
    }
}

void HeartbeatProducer::producer_loop() {
    while (running_.load()) {
        if (bus_) {
            CANFrame frame = MessageFactory::create_heartbeat(node_id_, state_);
            bus_->send(frame);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    }
}

// ==================== Heartbeat Consumer ====================

HeartbeatConsumer::HeartbeatConsumer() = default;

void HeartbeatConsumer::add_producer(uint8_t node_id, uint16_t heartbeat_time) {
    ProducerInfo info;
    info.heartbeat_time = heartbeat_time;
    info.last_heartbeat = std::chrono::steady_clock::now();
    producers_[node_id] = info;

    if (!running_.load()) {
        running_.store(true);
        consumer_thread_ = std::thread(&HeartbeatConsumer::consumer_loop, this);
    }
}

void HeartbeatConsumer::remove_producer(uint8_t node_id) {
    producers_.erase(node_id);

    if (producers_.empty() && running_.load()) {
        running_.store(false);
        if (consumer_thread_.joinable()) {
            consumer_thread_.join();
        }
    }
}

void HeartbeatConsumer::set_consumer_heartbeat_time(uint16_t time_ms) {
    consumer_time_ms_ = time_ms;
}

void HeartbeatConsumer::handle_frame(const CANFrame& frame) {
    uint8_t node_id;
    NMTState state;
    if (!MessageFactory::parse_heartbeat(frame, node_id, state)) {
        return;
    }

    auto it = producers_.find(node_id);
    if (it == producers_.end()) {
        // Auto-add new producer
        add_producer(node_id, 1000);
        it = producers_.find(node_id);
    }

    bool old_bootup = it->second.in_bootup;
    it->second.last_heartbeat = std::chrono::steady_clock::now();
    it->second.lost = false;

    // Check if bootup (initial state)
    if (state == NMTState::INITIALISING || state == static_cast<NMTState>(0)) {
        it->second.in_bootup = true;
    } else {
        if (old_bootup && !it->second.in_bootup) {
            // Was bootup, now operational
        }
        it->second.in_bootup = false;
    }

    // State change callback
    if (it->second.state != state && on_producer_state) {
        on_producer_state(node_id, state);
    }

    it->second.state = state;
}

HeartbeatConsumer::ProducerStatus HeartbeatConsumer::get_producer_status(uint8_t node_id) const {
    ProducerStatus status;
    auto it = producers_.find(node_id);
    if (it != producers_.end()) {
        status.state = it->second.state;
        status.last_heartbeat = it->second.last_heartbeat;
        status.in_bootup = it->second.in_bootup;
        status.lost = it->second.lost;
    }
    return status;
}

bool HeartbeatConsumer::is_producer_alive(uint8_t node_id) const {
    auto it = producers_.find(node_id);
    if (it == producers_.end()) return false;
    return !it->second.lost;
}

void HeartbeatConsumer::consumer_loop() {
    while (running_.load()) {
        check_timeouts();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void HeartbeatConsumer::check_timeouts() {
    auto now = std::chrono::steady_clock::now();

    for (auto& [node_id, info] : producers_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - info.last_heartbeat).count();

        uint32_t timeout = std::max(info.heartbeat_time, static_cast<uint16_t>(consumer_time_ms_));

        if (elapsed > timeout && !info.lost) {
            info.lost = true;

            if (on_producer_timeout) {
                on_producer_timeout(node_id);
            }
        }
    }
}

} // namespace canopen
