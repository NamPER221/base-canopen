/**
 * @file lss.cpp
 * @brief LSS implementation
 */

#include <canopen/co/lss/lss.hpp>
#include <cstring>

namespace canopen {

// ==================== LSS Master ====================

LSSMaster::LSSMaster() = default;



int LSSMaster::switch_to_config(uint8_t node_id) {
    selected_node_ = node_id;

    uint8_t data[5] = {
        0x05,  // Switch mode selective
        static_cast<uint8_t>(node_id),
        0, 0, 0  // LSS address
    };

    CANFrame frame = MessageFactory::create_lss_inquiry(
        LSSCommand::SWITCH_MODE_SELECTIVE, data, sizeof(data));
    if (bus_) bus_->send(frame);

    return 0;
}

int LSSMaster::switch_to_config_global() {
    uint8_t data[2] = {0x04, 0x00};  // Switch mode global, config
    CANFrame frame = MessageFactory::create_lss_inquiry(
        LSSCommand::SWITCH_MODE_GLOBAL, data, sizeof(data));
    if (bus_) bus_->send(frame);

    state_ = LSSState::WAITING_FOR_CONFIG;
    return 0;
}

int LSSMaster::switch_to_operational() {
    uint8_t data[2] = {0x04, 0x01};  // Switch mode global, operational
    CANFrame frame = MessageFactory::create_lss_inquiry(
        LSSCommand::SWITCH_MODE_GLOBAL, data, sizeof(data));
    if (bus_) bus_->send(frame);

    state_ = LSSState::IDLE;
    return 0;
}

int LSSMaster::set_node_id(uint8_t node_id) {
    uint8_t data[2] = {0x11, node_id};
    CANFrame frame = MessageFactory::create_lss_inquiry(
        LSSCommand::CONFIGURE_NODE_ID, data, sizeof(data));
    if (bus_) bus_->send(frame);

    return 0;
}

int LSSMaster::set_bit_timing(uint8_t bit_timing_index) {
    uint8_t data[2] = {0x15, bit_timing_index};
    CANFrame frame = MessageFactory::create_lss_inquiry(
        LSSCommand::CONFIGURE_BIT_TIMING, data, sizeof(data));
    if (bus_) bus_->send(frame);

    return 0;
}

int LSSMaster::store_configuration() {
    uint8_t data[1] = {0x17};
    CANFrame frame = MessageFactory::create_lss_inquiry(
        LSSCommand::STORE_CONFIGURATION, data, sizeof(data));
    if (bus_) bus_->send(frame);

    return 0;
}

int LSSMaster::inquire_lss_address(LSSAddress& address) {
    (void)address;
    // Inquire vendor ID
    CANFrame frame1 = MessageFactory::create_lss_inquiry(LSSCommand::INQUIRE_VENDOR_ID);
    if (bus_) bus_->send(frame1);

    return 0;
}

int LSSMaster::inquire_node_id(uint8_t& node_id) {
    (void)node_id;
    CANFrame frame = MessageFactory::create_lss_inquiry(LSSCommand::INQUIRE_NODE_ID);
    if (bus_) bus_->send(frame);

    return 0;
}

void LSSMaster::handle_frame(const CANFrame& frame) {
    auto opt_msg = MessageFactory::parse_lss(frame);
    if (!opt_msg) return;

    if (opt_msg->is_response) {
        handle_switch_response(frame);
        handle_config_response(frame);
        handle_inquiry_response(frame);
    }
}

void LSSMaster::handle_switch_response(const CANFrame& frame) {
    // Handle LSS switch response
    (void)frame;
}

void LSSMaster::handle_config_response(const CANFrame& frame) {
    (void)frame;
    // Handle LSS config response
    if (on_config_complete) {
        on_config_complete(true);
    }
}

void LSSMaster::handle_inquiry_response(const CANFrame& frame) {
    // Handle LSS inquiry response
    (void)frame;
}

// ==================== LSS Slave ====================

LSSSlave::LSSSlave() = default;



void LSSSlave::set_lss_address(const LSSAddress& address) {
    address_ = address;
}

void LSSSlave::set_node_id(uint8_t id) {
    uint8_t old_id = node_id_;
    node_id_ = id;

    if (old_id != id && on_node_id_change) {
        on_node_id_change(old_id, id);
    }
}

void LSSSlave::handle_frame(const CANFrame& frame) {
    auto opt_msg = MessageFactory::parse_lss(frame);
    if (!opt_msg || opt_msg->is_response) return;

    switch (opt_msg->command) {
        case LSSCommand::SWITCH_MODE_GLOBAL:
            // Global switch
            if (opt_msg->data.size() >= 2) {
                if (opt_msg->data[1] == 0x00) {
                    state_ = LSSState::WAITING_FOR_CONFIG;
                    if (on_mode_change) on_mode_change(true);
                } else {
                    state_ = LSSState::IDLE;
                    if (on_mode_change) on_mode_change(false);
                }
            }
            break;

        case LSSCommand::SWITCH_MODE_SELECTIVE:
            handle_switch_mode_selective(frame);
            break;

        case LSSCommand::CONFIGURE_NODE_ID:
            handle_configure_command(frame);
            break;

        case LSSCommand::STORE_CONFIGURATION:
            handle_store_command(frame);
            break;

        case LSSCommand::INQUIRE_VENDOR_ID:
        case LSSCommand::INQUIRE_PRODUCT_CODE:
        case LSSCommand::INQUIRE_REVISION:
        case LSSCommand::INQUIRE_SERIAL:
        case LSSCommand::INQUIRE_NODE_ID:
            handle_inquiry_command(frame);
            break;

        case LSSCommand::ACTIVATE_BIT_TIMING:
            handle_activate_bit_timing(frame);
            break;

        default:
            break;
    }
}

void LSSSlave::handle_switch_mode_selective(const CANFrame& frame) {
    (void)frame;
    // Parse selective switch - LSS address in frame
    // For now, just switch to config mode
    state_ = LSSState::WAITING_FOR_CONFIG;
    if (on_mode_change) {
        on_mode_change(true);
    }
}

void LSSSlave::handle_configure_command(const CANFrame& frame) {
    if (state_ != LSSState::WAITING_FOR_CONFIG) return;

    if (frame.len() >= 2 && frame.get_u8(1) == 0x11) {
        uint8_t new_id = frame.get_u8(2);
        if (new_id >= 1 && new_id <= 127) {
            set_node_id(new_id);
        }
    }

    send_response(0x90);  // Inhibit time
}

void LSSSlave::handle_store_command(const CANFrame& frame) {
    (void)frame;
    if (state_ != LSSState::WAITING_FOR_CONFIG) return;
    // Store configuration - would persist to non-volatile storage
    send_response(0x92);
}

void LSSSlave::handle_inquiry_command(const CANFrame& frame) {
    if (frame.len() < 2) return;

    uint8_t cmd = frame.get_u8(1);
    uint32_t reply_data[2] = {0};

    switch (cmd) {
        case 0x0C:  // Vendor ID
            reply_data[0] = address_.vendor_id;
            break;
        case 0x0D:  // Product code
            reply_data[0] = address_.product_code;
            break;
        case 0x0E:  // Revision
            reply_data[0] = address_.revision;
            break;
        case 0x0F:  // Serial
            reply_data[0] = address_.serial;
            break;
        case 0x10:  // Node ID
            reply_data[0] = node_id_;
            break;
    }

    send_response(0x91, reinterpret_cast<uint8_t*>(reply_data), 4);
}

void LSSSlave::handle_activate_bit_timing(const CANFrame& frame) {
    if (frame.len() >= 4) {
        // Parse delay (2 bytes big-endian)
        // uint16_t delay = (frame.get_u8(2) << 8) | frame.get_u8(3);
        awaiting_activate_bit_timing_ = true;
    }
}

void LSSSlave::send_response(uint8_t specifier, const uint8_t* data, size_t len) {
    CANFrame frame;
    frame.set_id(0x7E5);  // LSS slave
    frame.set_len(static_cast<uint8_t>(2 + len));
    frame.set_u8(0, specifier);
    frame.set_u8(1, node_id_);

    if (data && len > 0) {
        std::memcpy(frame.data() + 2, data, len);
    }

    if (bus_) bus_->send(frame);
}

} // namespace canopen
