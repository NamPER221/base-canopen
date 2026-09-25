/**
 * @file emcy.cpp
 * @brief EMCY implementation
 */

#include <canopen/co/emcy/emcy.hpp>
#include <cstring>

namespace canopen {

EMCYService::EMCYService(ObjectDictionary& od) : od_(od) {}

void EMCYService::set_node_id(uint8_t id) {
    node_id_ = id;
}

void EMCYService::handle_frame(const CANFrame& frame) {
    auto opt_msg = MessageFactory::parse_emcy(frame);
    if (!opt_msg) return;

    const auto& msg = *opt_msg;

    EmergencyError error;
    error.error_code = msg.error_code;
    error.error_register = msg.error_register;
    std::memcpy(error.manufacturer_specific, msg.manufacturer_specific, 5);

    add_to_history(error);

    if (on_emergency) {
        on_emergency(msg.node_id, error);
    }
}

void EMCYService::send(uint16_t error_code, uint8_t error_register,
                       const uint8_t* manufacturer) {
    if (!bus_) return;
    CANFrame frame = MessageFactory::create_emcy(node_id_, error_code, error_register, manufacturer);
    bus_->send(frame);
}

void EMCYService::clear_error_register() {
    uint8_t old = error_register_;
    error_register_ = 0;

    if (old != 0 && on_error_register_change) {
        on_error_register_change(error_register_);
    }
}

void EMCYService::update_error_register() {
    uint8_t old = error_register_;
    // Update from OD 0x1001 if needed
    (void)old;
}

void EMCYService::add_to_history(const EmergencyError& error) {
    error_history_[error_history_index_] = error;
    error_history_index_ = (error_history_index_ + 1) % 8;
}

EMCYConsumer::EMCYConsumer() {}

void EMCYConsumer::add_producer(uint8_t node_id) {
    producers_[node_id] = ProducerInfo{};
}

void EMCYConsumer::remove_producer(uint8_t node_id) {
    producers_.erase(node_id);
}

void EMCYConsumer::handle_frame(const CANFrame& frame) {
    auto opt_msg = MessageFactory::parse_emcy(frame);
    if (!opt_msg) return;

    const auto& msg = *opt_msg;
    auto it = producers_.find(msg.node_id);
    if (it == producers_.end()) return;

    it->second.last_error = {msg.error_code, msg.error_register, {}};
    std::memcpy(it->second.last_error.manufacturer_specific, msg.manufacturer_specific, 5);
    it->second.has_error = true;

    if (on_emergency) {
        on_emergency(msg.node_id, it->second.last_error);
    }
}

const EmergencyError* EMCYConsumer::get_error(uint8_t node_id) const {
    auto it = producers_.find(node_id);
    if (it != producers_.end() && it->second.has_error) {
        return &it->second.last_error;
    }
    return nullptr;
}

void EMCYConsumer::clear_error(uint8_t node_id) {
    auto it = producers_.find(node_id);
    if (it != producers_.end()) {
        it->second.has_error = false;
    }
}

} // namespace canopen
