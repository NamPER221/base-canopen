/**
 * @file message_factory.cpp
 * @brief CANopen Message Factory implementation
 */

#include <canopen/can/msg/message_factory.hpp>
#include <cstring>

namespace canopen {

// ==================== NMT ====================

CANFrame MessageFactory::create_nmt(uint8_t node_id, NMTCommand cmd) {
    CANFrame frame;
    frame.set_id(COBID::NMT);
    frame.set_len(2);
    frame.set_u8(0, static_cast<uint8_t>(cmd));
    frame.set_u8(1, node_id);
    return frame;
}

bool MessageFactory::parse_nmt(const CANFrame& frame, uint8_t& node_id, NMTCommand& cmd) {
    if (frame.id() != COBID::NMT || frame.len() != 2) {
        return false;
    }

    cmd = static_cast<NMTCommand>(frame.get_u8(0));
    node_id = frame.get_u8(1);
    return true;
}

// ==================== PDO ====================

CANFrame MessageFactory::create_pdo_tx(uint8_t node_id, uint8_t pdo_num,
                                       const PDOMapping& mapping) {
    CANFrame frame;
    frame.set_id(COBID::tpdo_tx(node_id, pdo_num));
    frame.set_len(static_cast<uint8_t>(mapping.data_size()));

    uint8_t offset = 0;
    // Note: Actual data would be populated from Object Dictionary
    // This is just the structure creation
    (void)offset;  // Suppress unused warning

    return frame;
}

bool MessageFactory::parse_pdo_tx(const CANFrame& frame, PDOMapping& mapping) {
    (void)frame;
    (void)mapping;
    // Parse based on mapping configuration
    return true;
}

CANFrame MessageFactory::create_pdo_rx(uint8_t node_id, uint8_t pdo_num,
                                       const PDOMapping& mapping) {
    CANFrame frame;
    frame.set_id(COBID::tpdo_rx(node_id, pdo_num));
    frame.set_len(static_cast<uint8_t>(mapping.data_size()));
    return frame;
}

bool MessageFactory::parse_pdo_rx(const CANFrame& frame, PDOMapping& mapping) {
    (void)frame;
    (void)mapping;
    return true;
}

// ==================== SDO ====================

CANFrame MessageFactory::create_sdo_download_request(uint8_t node_id, uint16_t index,
                                                     uint8_t subindex, const void* data,
                                                     size_t data_size) {
    CANFrame frame;
    frame.set_id(COBID::sdo_rx(node_id));

    uint8_t cmd = static_cast<uint8_t>(SDOCommand::DOWNLOAD_INITIATE);

    // Command byte determines data size encoding
    if (data_size <= 4) {
        // Expedited transfer: e=1, s=1, n = 4 - size
        cmd |= (4 - data_size) << 2;
        cmd |= 0x03;  // e=1 (bit 1) + s=1 (bit 0, size indicator)

        frame.set_len(8);  // cmd + index(2) + subindex + data(4)
        frame.set_u8(0, cmd);
        frame.set_u16_le(1, index);
        frame.set_u8(3, subindex);
        std::memcpy(frame.data() + 4, data, data_size);
    } else {
        // Normal transfer
        cmd = 0x21;  // Download initiate, normal transfer (e=0, s=1)
        frame.set_len(8);
        frame.set_u8(0, cmd);
        frame.set_u16_le(1, index);
        frame.set_u8(3, subindex);
        frame.set_u32_le(4, static_cast<uint32_t>(data_size));
    }

    return frame;
}

CANFrame MessageFactory::create_sdo_upload_request(uint8_t node_id, uint16_t index,
                                                   uint8_t subindex) {
    CANFrame frame;
    frame.set_id(COBID::sdo_rx(node_id));
    // ZLAC8015D (CiA 306) requires SDO messages to ALWAYS be 8 bytes
    frame.set_len(8);
    frame.set_u8(0, static_cast<uint8_t>(SDOCommand::UPLOAD_INITIATE));
    frame.set_u16_le(1, index);
    frame.set_u8(3, subindex);
    // bytes 4-7 remain zero
    return frame;
}

CANFrame MessageFactory::create_sdo_download_response(uint8_t node_id, uint16_t index,
                                                      uint8_t subindex, const void* data,
                                                      size_t data_size) {
    (void)data;  // download response carries no payload
    CANFrame frame;
    frame.set_id(COBID::sdo_tx(node_id));

    if (data_size <= 4) {
        // Expedited transfer response
        frame.set_len(4);
        frame.set_u8(0, 0x60);  // Download initiate response
        frame.set_u16_le(1, index);
        frame.set_u8(3, subindex);
    } else {
        // Normal transfer response
        frame.set_len(8);
        frame.set_u8(0, 0x20);  // Download initiate response, normal transfer
        frame.set_u16_le(1, index);
        frame.set_u8(3, subindex);
        frame.set_u32_le(4, static_cast<uint32_t>(data_size));
    }

    return frame;
}

CANFrame MessageFactory::create_sdo_upload_response(uint8_t node_id, uint16_t index,
                                                   uint8_t subindex, const void* data,
                                                   size_t data_size) {
    CANFrame frame;
    frame.set_id(COBID::sdo_tx(node_id));

    if (data_size <= 4) {
        // Expedited transfer
        uint8_t cmd = 0x43 | ((4 - data_size) << 2);  // Upload initiate response
        frame.set_len(static_cast<uint8_t>(data_size + 4));
        frame.set_u8(0, cmd);
        frame.set_u16_le(1, index);
        frame.set_u8(3, subindex);
        std::memcpy(frame.data() + 4, data, data_size);
    } else {
        // Normal transfer
        frame.set_len(8);
        frame.set_u8(0, 0x41);  // Upload initiate response
        frame.set_u16_le(1, index);
        frame.set_u8(3, subindex);
        frame.set_u32_le(4, static_cast<uint32_t>(data_size));
    }

    return frame;
}

CANFrame MessageFactory::create_sdo_abort(uint8_t node_id, uint16_t index,
                                         uint8_t subindex, uint32_t abort_code) {
    CANFrame frame;
    frame.set_id(COBID::sdo_tx(node_id));
    frame.set_len(8);
    frame.set_u8(0, static_cast<uint8_t>(SDOCommand::ABORT));
    frame.set_u16_le(1, index);
    frame.set_u8(3, subindex);
    frame.set_u32_le(4, abort_code);
    return frame;
}

std::optional<MessageFactory::SDOMessage> MessageFactory::parse_sdo(const CANFrame& frame) {
    SDOMessage msg;
    uint32_t cob_id = frame.id();

    // Determine node ID from COB-ID
    if ((cob_id & 0x780) == 0x580) {
        msg.node_id = cob_id & 0x7F;
    } else if ((cob_id & 0x780) == 0x600) {
        msg.node_id = cob_id & 0x7F;
    } else {
        return std::nullopt;
    }

    if (frame.len() < 4) {
        return std::nullopt;
    }

    uint8_t cmd = frame.get_u8(0);
    msg.index = frame.get_u16_le(1);
    msg.subindex = frame.get_u8(3);

    // Determine command type
    if ((cmd & 0xE0) == 0x20) {
        msg.command = SDOCommand::DOWNLOAD_INITIATE;
        if (frame.len() > 4) {
            msg.data.resize(frame.len() - 4);
            std::memcpy(msg.data.data(), frame.data() + 4, frame.len() - 4);
        }
    } else if ((cmd & 0xE0) == 0x40) {
        msg.command = SDOCommand::UPLOAD_INITIATE;
    } else if ((cmd & 0xE0) == 0x60) {
        msg.command = SDOCommand::DOWNLOAD_INITIATE;  // Response
    } else if ((cmd & 0xE0) == 0x43 || (cmd & 0xE0) == 0x41) {
        msg.command = SDOCommand::UPLOAD_INITIATE;  // Response
        if (frame.len() > 4) {
            msg.data.resize(frame.len() - 4);
            std::memcpy(msg.data.data(), frame.data() + 4, frame.len() - 4);
        }
    } else if (cmd == 0x80) {
        msg.command = SDOCommand::ABORT;
        msg.abort_code = frame.get_u32_le(4);
    } else {
        return std::nullopt;
    }

    return msg;
}

// ==================== SYNC ====================

CANFrame MessageFactory::create_sync(uint8_t counter) {
    CANFrame frame;
    frame.set_id(COBID::SYNC);
    if (counter != 0) {
        frame.set_len(1);
        frame.set_u8(0, counter);
    } else {
        frame.set_len(0);
    }
    return frame;
}

bool MessageFactory::parse_sync(const CANFrame& frame, uint8_t& counter) {
    if (frame.id() != COBID::SYNC) {
        return false;
    }

    if (frame.len() == 1) {
        counter = frame.get_u8(0);
    } else {
        counter = 0;
    }
    return true;
}

// ==================== Heartbeat ====================

CANFrame MessageFactory::create_heartbeat(NMTState state) {
    return create_heartbeat(0, state);
}

CANFrame MessageFactory::create_heartbeat(uint8_t node_id, NMTState state) {
    CANFrame frame;
    frame.set_id(COBID::heartbeat(node_id));
    frame.set_len(1);
    frame.set_u8(0, static_cast<uint8_t>(state));
    return frame;
}

bool MessageFactory::parse_heartbeat(const CANFrame& frame, uint8_t& node_id, NMTState& state) {
    uint32_t cob_id = frame.id();

    if ((cob_id & 0x700) != 0x700) {
        return false;
    }

    if (frame.len() != 1) {
        return false;
    }

    node_id = cob_id & 0x7F;
    state = static_cast<NMTState>(frame.get_u8(0));
    return true;
}

// ==================== EMCY ====================

CANFrame MessageFactory::create_emcy(uint8_t node_id, uint16_t error_code,
                                     uint8_t error_register,
                                     const uint8_t* manufacturer) {
    CANFrame frame;
    frame.set_id(COBID::emcy(node_id));
    frame.set_len(8);
    frame.set_u16_le(0, error_code);
    frame.set_u8(2, error_register);
    frame.set_u8(3, 0);  // Reserved

    if (manufacturer) {
        std::memcpy(frame.data() + 4, manufacturer, 5);
    } else {
        std::memset(frame.data() + 4, 0, 5);
    }

    return frame;
}

std::optional<MessageFactory::EMCYMessage> MessageFactory::parse_emcy(const CANFrame& frame) {
    uint32_t cob_id = frame.id();

    if ((cob_id & 0xF80) != 0x100) {
        return std::nullopt;
    }

    if (frame.len() != 8) {
        return std::nullopt;
    }

    EMCYMessage msg;
    msg.node_id = cob_id & 0x7F;
    msg.error_code = frame.get_u16_le(0);
    msg.error_register = frame.get_u8(2);
    std::memcpy(msg.manufacturer_specific, frame.data() + 4, 5);

    return msg;
}

// ==================== LSS ====================

CANFrame MessageFactory::create_lss_inquiry(LSSCommand cmd, const uint8_t* data,
                                            size_t data_len) {
    CANFrame frame;
    frame.set_id(COBID::LSS_MASTER);
    frame.set_len(static_cast<uint8_t>(2 + data_len));
    frame.set_u8(0, 0x04);  // LSS switch mode
    frame.set_u8(1, static_cast<uint8_t>(cmd));

    if (data && data_len > 0) {
        std::memcpy(frame.data() + 2, data, data_len);
    }

    return frame;
}

CANFrame MessageFactory::create_lss_response(const uint8_t* data, size_t data_len) {
    CANFrame frame;
    frame.set_id(COBID::LSS_SLAVE);
    frame.set_len(static_cast<uint8_t>(data_len));

    if (data && data_len > 0) {
        std::memcpy(frame.data(), data, data_len);
    }

    return frame;
}

std::optional<MessageFactory::LSSMessage> MessageFactory::parse_lss(const CANFrame& frame) {
    uint32_t cob_id = frame.id();

    LSSMessage msg;

    if (cob_id == COBID::LSS_MASTER) {
        msg.is_response = false;
    } else if (cob_id == COBID::LSS_SLAVE) {
        msg.is_response = true;
    } else {
        return std::nullopt;
    }

    if (frame.len() < 2) {
        return std::nullopt;
    }

    msg.command = static_cast<LSSCommand>(frame.get_u8(1));
    msg.data.resize(frame.len() - 2);
    std::memcpy(msg.data.data(), frame.data() + 2, frame.len() - 2);

    return msg;
}

// ==================== Utility ====================

std::string MessageFactory::get_message_type_name(uint32_t cob_id) {
    if (cob_id == COBID::NMT) return "NMT";
    if (cob_id == COBID::SYNC) return "SYNC";
    if ((cob_id & 0x780) == 0x180) return "TPDO1";
    if ((cob_id & 0x780) == 0x200) return "RPDO1";
    if ((cob_id & 0x780) == 0x280) return "TPDO2";
    if ((cob_id & 0x780) == 0x300) return "RPDO2";
    if ((cob_id & 0x780) == 0x380) return "TPDO3";
    if ((cob_id & 0x780) == 0x400) return "RPDO3";
    if ((cob_id & 0x780) == 0x480) return "TPDO4";
    if ((cob_id & 0x780) == 0x500) return "RPDO4";
    if ((cob_id & 0x780) == 0x580) return "TSDO";
    if ((cob_id & 0x780) == 0x600) return "RSDO";
    if ((cob_id & 0xF80) == 0x100) return "EMCY";
    if ((cob_id & 0x700) == 0x700) return "HEARTBEAT";
    if (cob_id == COBID::LSS_MASTER) return "LSS_MASTER";
    if (cob_id == COBID::LSS_SLAVE) return "LSS_SLAVE";
    return "UNKNOWN";
}

bool MessageFactory::is_canopen_cobid(uint32_t cob_id) {
    // Valid ranges
    if (cob_id <= 0x7FF) {
        // NMT, SYNC, EMCY base, PDO range, SDO range, Heartbeat range, LSS
        if (cob_id == 0) return true;  // NMT
        if (cob_id == 0x80) return true;  // SYNC
        if ((cob_id & 0xF80) == 0x100) return true;  // EMCY
        if ((cob_id & 0x780) == 0x180) return true;  // TPDO1
        if ((cob_id & 0x780) == 0x200) return true;  // RPDO1
        if ((cob_id & 0x780) == 0x280) return true;  // TPDO2
        if ((cob_id & 0x780) == 0x300) return true;  // RPDO2
        if ((cob_id & 0x780) == 0x380) return true;  // TPDO3
        if ((cob_id & 0x780) == 0x400) return true;  // RPDO3
        if ((cob_id & 0x780) == 0x480) return true;  // TPDO4
        if ((cob_id & 0x780) == 0x500) return true;  // RPDO4
        if ((cob_id & 0x780) == 0x580) return true;  // TSDO
        if ((cob_id & 0x780) == 0x600) return true;  // RSDO
        if ((cob_id & 0x700) == 0x700) return true;  // Heartbeat
        if (cob_id == 0x7E4 || cob_id == 0x7E5) return true;  // LSS
    }
    return false;
}

} // namespace canopen
