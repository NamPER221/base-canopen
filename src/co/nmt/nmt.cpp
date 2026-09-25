/**
 * @file nmt.cpp
 * @brief NMT Service implementation
 */

#include <canopen/co/nmt/nmt.hpp>
#include <cstring>
#include <chrono>

namespace canopen {

NMTService::NMTService(ObjectDictionary& od, BusInterface* bus, uint8_t node_id)
    : od_(od), bus_(bus), node_id_(node_id) {
}

NMTService::~NMTService() {
    stop();
}

void NMTService::attach(BusInterface& bus) {
    bus_ = &bus;
    route_nmt_ = routes::nmt(bus, [this](const CANFrame& f) {
        handle_frame(f);
    });
    route_hb_ = routes::heartbeat_all(bus, [this](const CANFrame& f) {
        handle_frame(f);
    });
}

void NMTService::detach(BusInterface& bus) {
    if (route_nmt_) bus.remove_route(route_nmt_);
    if (route_hb_) bus.remove_route(route_hb_);
    route_nmt_ = 0;
    route_hb_ = 0;
    bus_ = nullptr;
}

void NMTService::start() {
    if (running_.load()) return;

    running_.store(true);

    // Start heartbeat consumer thread
    consumer_running_.store(true);
    heartbeat_consumer_thread_ = std::thread(&NMTService::heartbeat_consumer_loop, this);
}

void NMTService::stop() {
    running_.store(false);

    stop_heartbeat_producer();

    consumer_running_.store(false);
    if (heartbeat_consumer_thread_.joinable()) {
        heartbeat_consumer_thread_.join();
    }
}

void NMTService::send_command(uint8_t node_id, NMTCommand cmd) {
    if (!bus_) return;

    CANFrame frame = MessageFactory::create_nmt(node_id, cmd);
    bus_->send(frame);
}

void NMTService::start_heartbeat_producer(uint16_t interval_ms) {
    if (heartbeat_producer_active_.load()) {
        stop_heartbeat_producer();
    }

    heartbeat_interval_ms_ = interval_ms;
    heartbeat_producer_active_.store(true);

    heartbeat_producer_thread_ = std::thread([this]() {
        while (heartbeat_producer_active_.load()) {
            send_heartbeat();
            std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms_));
        }
    });
}

void NMTService::stop_heartbeat_producer() {
    heartbeat_producer_active_.store(false);
    if (heartbeat_producer_thread_.joinable()) {
        heartbeat_producer_thread_.join();
    }
}

void NMTService::add_heartbeat_consumer(uint8_t node_id, uint16_t producer_time_ms) {
    std::lock_guard<std::mutex> lock(node_states_mutex_);

    NodeStateInfo info;
    info.last_heartbeat = std::chrono::steady_clock::now();
    node_states_[node_id] = info;
    (void)producer_time_ms;
}

void NMTService::remove_heartbeat_consumer(uint8_t node_id) {
    std::lock_guard<std::mutex> lock(node_states_mutex_);
    node_states_.erase(node_id);
}

void NMTService::set_heartbeat_timeout(uint8_t node_id, uint16_t timeout_ms) {
    std::lock_guard<std::mutex> lock(node_states_mutex_);
    if (node_states_.find(node_id) != node_states_.end()) {
        consumer_timeout_ms_ = timeout_ms;
    }
    (void)node_id;
}

void NMTService::set_state(NMTState state) {
    NMTState old = state_.load();
    state_.store(state);

    if (old != state && on_state_change) {
        on_state_change(old, state);
    }
}

NodeStateInfo NMTService::get_node_state(uint8_t node_id) const {
    std::lock_guard<std::mutex> lock(node_states_mutex_);
    auto it = node_states_.find(node_id);
    if (it != node_states_.end()) {
        return it->second;
    }
    return NodeStateInfo{};
}

std::map<uint8_t, NodeStateInfo> NMTService::get_all_node_states() const {
    std::lock_guard<std::mutex> lock(node_states_mutex_);
    return node_states_;
}

void NMTService::send_heartbeat() {
    if (!bus_) return;

    // Bootup: first heartbeat with state INITIALISING (0x00)
    CANFrame frame = MessageFactory::create_heartbeat(node_id_, state_.load());
    bus_->send(frame);

    if (on_heartbeat_produced) {
        on_heartbeat_produced();
    }
}

void NMTService::handle_frame(const CANFrame& frame) {
    uint32_t cob_id = frame.id() & 0x7FF;

    // Check for NMT command (0x000)
    if (cob_id == 0x000 && frame.len() == 2) {
        handle_nmt_command(frame);
        return;
    }

    // Check for heartbeat (0x700 + node_id)
    if ((cob_id & 0x700) == 0x700 && frame.len() == 1) {
        uint8_t node_id = cob_id & 0x7F;
        if (node_id == node_id_) return;  // ignore own heartbeat

        NMTState state = static_cast<NMTState>(frame.get_u8(0));

        // Check for bootup message (state == 0)
        if (frame.get_u8(0) == 0) {
            handle_heartbeat(node_id, NMTState::INITIALISING);
        } else {
            handle_heartbeat(node_id, state);
        }
    }
}

void NMTService::handle_nmt_command(const CANFrame& frame) {
    uint8_t cmd_byte = frame.get_u8(0);
    uint8_t node_id = frame.get_u8(1);

    NMTCommand cmd = static_cast<NMTCommand>(cmd_byte);

    // Handle commands for this node (or broadcast)
    if (node_id == 0 || node_id == node_id_) {
        NMTState old_state = state_.load();
        NMTState new_state = old_state;

        switch (cmd) {
            case NMTCommand::OPERATIONAL:
                new_state = NMTState::OPERATIONAL;
                break;
            case NMTCommand::STOP:
                new_state = NMTState::STOPPED;
                break;
            case NMTCommand::PREOPERATIONAL:
                new_state = NMTState::PREOPERATIONAL;
                break;
            case NMTCommand::RESET_NODE:
            case NMTCommand::RESET_COMMUNICATION:
                // Trigger reset
                new_state = NMTState::INITIALISING;
                break;
        }

        if (old_state != new_state) {
            set_state(new_state);
        }
    }
}

void NMTService::handle_heartbeat(uint8_t node_id, NMTState state) {
    std::lock_guard<std::mutex> lock(node_states_mutex_);

    auto it = node_states_.find(node_id);
    if (it == node_states_.end()) {
        // New node detected, add it
        NodeStateInfo info;
        info.state = state;
        info.last_heartbeat = std::chrono::steady_clock::now();
        info.bootup_received = (state == NMTState::INITIALISING);
        node_states_[node_id] = info;

        if (on_bootup) {
            on_bootup(node_id);
        }
    } else {
        bool state_changed = (it->second.state != state);
        it->second.state = state;
        it->second.last_heartbeat = std::chrono::steady_clock::now();

        if (state == NMTState::INITIALISING && !it->second.bootup_received) {
            it->second.bootup_received = true;
            if (on_bootup) {
                on_bootup(node_id);
            }
        } else if (state_changed && on_node_state_change) {
            on_node_state_change(node_id, state);
        }
    }
}

void NMTService::heartbeat_consumer_loop() {
    while (consumer_running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto now = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(node_states_mutex_);
        for (auto& [node_id, info] : node_states_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - info.last_heartbeat).count();

            if (elapsed > consumer_timeout_ms_ && !info.heartbeat_timeout) {
                info.heartbeat_timeout = true;
                info.consecutive_timeouts++;

                if (on_heartbeat_timeout) {
                    on_heartbeat_timeout(node_id);
                }
            }
        }
    }
}

} // namespace canopen
