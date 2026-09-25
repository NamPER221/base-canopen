/**
 * @file message_factory.hpp
 * @brief CANopen Message Factory for creating/parsing CANopen frames
 */

#ifndef CANOPEN_CAN_MSG_MESSAGE_FACTORY_HPP
#define CANOPEN_CAN_MSG_MESSAGE_FACTORY_HPP

#include <canopen/can/frame/frame.hpp>
#include <linux/can.h>
#include <cstdint>
#include <string>
#include <vector>
#include <functional>

namespace canopen {

// ==================== NMT Types ====================

/**
 * @brief NMT State enumeration
 */
enum class NMTState : uint8_t {
    INITIALISING = 0x00,
    PREOPERATIONAL = 0x7F,
    OPERATIONAL = 0x05,
    STOPPED = 0x04,
};

/**
 * @brief NMT Command enumeration
 */
enum class NMTCommand : uint8_t {
    OPERATIONAL = 0x01,
    STOP = 0x02,
    PREOPERATIONAL = 0x80,
    RESET_NODE = 0x81,
    RESET_COMMUNICATION = 0x82,
};

// ==================== PDO Types ====================

/**
 * @brief PDO transmission type
 */
enum class PDOTransmissionType : uint8_t {
    SYNCHRONOUS_CYCLIC = 0,      // 0 = acyclic, 1-240 = cyclic
    SYNCHRONOUS_ACYCLIC = 254,
    ASYNCHRONOUS_SPECIFIC = 252,
    ASYNCHRONOUS_RTR_SYNC = 253,
    ASYNCHRONOUS = 255,
};

/**
 * @brief PDO Mapping entry
 */
struct PDOMappingEntry {
    uint16_t object_index;
    uint8_t subindex;
    uint8_t bit_length;
};

/**
 * @brief PDO Mapping
 */
struct PDOMapping {
    std::vector<PDOMappingEntry> entries;
    size_t data_size() const;
};

// ==================== SDO Types ====================

/**
 * @brief SDO Command Specifier
 */
enum class SDOCommand : uint8_t {
    DOWNLOAD_INITIATE = 0x20,
    DOWNLOAD_SEGMENT = 0x00,
    UPLOAD_INITIATE = 0x40,
    UPLOAD_SEGMENT = 0x60,
    ABORT = 0x80,
};

/**
 * @brief SDO abort codes
 */
enum class SDOAbortCode : uint32_t {
    TOGGLE_NOT_ALTERNATED = 0x05030000,
    TIMEOUT = 0x05040000,
    COMMAND_SPECIFIER_INVALID = 0x05040001,
    BLOCK_SIZE_INVALID = 0x05040002,
    BLOCK_SEQUENCE_INVALID = 0x05040003,
    DATA_TYPE_MISMATCH = 0x06030000,
    OBJECT_LENGTH_MISMATCH = 0x06070010,
    ACCESS_FAILED = 0x06010000,
    OBJECT_NOT_EXIST = 0x06020000,
    OBJECT_PDO_MAPPED = 0x06040005,
    PARAMETER_INCOMPATIBLE = 0x06040041,
};

/**
 * @brief PDO Communication Parameter
 */
struct PDOCommParam {
    uint32_t cob_id;
    uint8_t transmission_type;
    uint16_t inhibit_time;
    uint16_t reserved;
    uint16_t event_timer;
    uint8_t sync_start_value;
};

// ==================== LSS Types ====================

/**
 * @brief LSS Command
 */
enum class LSSCommand : uint8_t {
    SWITCH_MODE_GLOBAL = 0x04,
    SWITCH_MODE_SELECTIVE = 0x05,
    INQUIRE_VENDOR_ID = 0x0C,
    INQUIRE_PRODUCT_CODE = 0x0D,
    INQUIRE_REVISION = 0x0E,
    INQUIRE_SERIAL = 0x0F,
    INQUIRE_NODE_ID = 0x10,
    CONFIGURE_NODE_ID = 0x11,
    CONFIGURE_BIT_TIMING = 0x15,
    ACTIVATE_BIT_TIMING = 0x16,
    STORE_CONFIGURATION = 0x17,
};

/**
 * @brief LSS Address
 */
struct LSSAddress {
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    uint32_t serial;
};

/**
 * @brief CANopen COB-ID Calculator
 */
class COBID {
public:
    // NMT
    static constexpr uint32_t NMT = 0x000;

    // SYNC
    static constexpr uint32_t SYNC = 0x080;
    static constexpr uint32_t SYNC_PRODUCER = 0x080;

    // Emergency
    static constexpr uint32_t EMCY_BASE = 0x080;
    static uint32_t emcy(uint8_t node_id);

    // PDO
    static constexpr uint32_t TPDO1_BASE = 0x180;
    static constexpr uint32_t RPDO1_BASE = 0x200;
    static constexpr uint32_t TPDO2_BASE = 0x280;
    static constexpr uint32_t RPDO2_BASE = 0x300;
    static constexpr uint32_t TPDO3_BASE = 0x380;
    static constexpr uint32_t RPDO3_BASE = 0x400;
    static constexpr uint32_t TPDO4_BASE = 0x480;
    static constexpr uint32_t RPDO4_BASE = 0x500;

    static uint32_t tpdo_tx(uint8_t node_id, uint8_t pdo_num);
    static uint32_t tpdo_rx(uint8_t node_id, uint8_t pdo_num);

    // SDO
    static constexpr uint32_t TSDO_BASE = 0x580;
    static constexpr uint32_t RSDO_BASE = 0x600;

    static uint32_t sdo_tx(uint8_t node_id);
    static uint32_t sdo_rx(uint8_t node_id);

    // Heartbeat
    static constexpr uint32_t HEARTBEAT_BASE = 0x700;

    static uint32_t heartbeat(uint8_t node_id);
    static uint32_t heartbeat_consumer(uint8_t consumer_num);

    // LSS
    static constexpr uint32_t LSS_MASTER = 0x7E4;
    static constexpr uint32_t LSS_SLAVE = 0x7E5;

    // Helper: Check if COB-ID is valid
    static bool is_valid(uint32_t cob_id);
    static bool is_valid_node_id(uint8_t node_id);
};

// ==================== Message Factory ====================

/**
 * @brief Factory for creating and parsing CANopen messages
 */
class MessageFactory {
public:
    MessageFactory() = default;

    // ==================== NMT ====================

    /**
     * @brief Create NMT command frame
     * @param node_id Target node ID (0 = all nodes)
     * @param cmd NMT command
     * @return CANFrame containing NMT command
     */
    static CANFrame create_nmt(uint8_t node_id, NMTCommand cmd);

    /**
     * @brief Parse NMT command from frame
     * @param frame Input frame
     * @param node_id Output node ID
     * @param cmd Output command
     * @return true if valid NMT frame
     */
    static bool parse_nmt(const CANFrame& frame, uint8_t& node_id, NMTCommand& cmd);

    // ==================== PDO ====================

    /**
     * @brief Create TPDO transmission frame
     */
    static CANFrame create_pdo_tx(uint8_t node_id, uint8_t pdo_num,
                                   const PDOMapping& mapping);

    /**
     * @brief Parse TPDO frame data
     */
    static bool parse_pdo_tx(const CANFrame& frame, PDOMapping& mapping);

    /**
     * @brief Create RPDO reception frame
     */
    static CANFrame create_pdo_rx(uint8_t node_id, uint8_t pdo_num,
                                   const PDOMapping& mapping);

    /**
     * @brief Parse RPDO frame data
     */
    static bool parse_pdo_rx(const CANFrame& frame, PDOMapping& mapping);

    // ==================== SDO ====================

    /**
     * @brief Create SDO download initiate request
     */
    static CANFrame create_sdo_download_request(uint8_t node_id, uint16_t index,
                                                 uint8_t subindex, const void* data,
                                                 size_t data_size);

    /**
     * @brief Create SDO upload initiate request
     */
    static CANFrame create_sdo_upload_request(uint8_t node_id, uint16_t index,
                                               uint8_t subindex);

    /**
     * @brief Create SDO download response
     */
    static CANFrame create_sdo_download_response(uint8_t node_id, uint16_t index,
                                                  uint8_t subindex, const void* data,
                                                  size_t data_size);

    /**
     * @brief Create SDO upload response
     */
    static CANFrame create_sdo_upload_response(uint8_t node_id, uint16_t index,
                                                uint8_t subindex, const void* data,
                                                size_t data_size);

    /**
     * @brief Create SDO abort response
     */
    static CANFrame create_sdo_abort(uint8_t node_id, uint16_t index,
                                     uint8_t subindex, uint32_t abort_code);

    /**
     * @brief Parse SDO message
     */
    struct SDOMessage {
        uint8_t node_id{0};
        uint16_t index{0};
        uint8_t subindex{0};
        SDOCommand command{SDOCommand::UPLOAD_INITIATE};
        std::vector<uint8_t> data;
        uint32_t abort_code{0};
    };

    static std::optional<SDOMessage> parse_sdo(const CANFrame& frame);

    // ==================== SYNC ====================

    /**
     * @brief Create SYNC frame
     * @param counter Counter value (0-255)
     */
    static CANFrame create_sync(uint8_t counter = 0);

    /**
     * @brief Parse SYNC frame
     * @param frame Input frame
     * @param counter Output counter
     * @return true if valid SYNC frame
     */
    static bool parse_sync(const CANFrame& frame, uint8_t& counter);

    // ==================== Heartbeat ====================

    /**
     * @brief Create Heartbeat frame (master, node 0)
     * @param state NMT state
     */
    static CANFrame create_heartbeat(NMTState state);

    /**
     * @brief Create Heartbeat frame for a specific node
     * @param node_id Node ID (bootup = state 0x00)
     * @param state NMT state
     */
    static CANFrame create_heartbeat(uint8_t node_id, NMTState state);

    /**
     * @brief Parse Heartbeat frame
     * @param frame Input frame
     * @param node_id Output node ID
     * @param state Output NMT state
     * @return true if valid heartbeat frame
     */
    static bool parse_heartbeat(const CANFrame& frame, uint8_t& node_id, NMTState& state);

    // ==================== EMCY ====================

    /**
     * @brief Create EMCY frame
     */
    static CANFrame create_emcy(uint8_t node_id, uint16_t error_code,
                                 uint8_t error_register,
                                 const uint8_t* manufacturer = nullptr);

    /**
     * @brief Parse EMCY frame
     */
    struct EMCYMessage {
        uint8_t node_id;
        uint16_t error_code;
        uint8_t error_register;
        uint8_t manufacturer_specific[5];
    };

    static std::optional<EMCYMessage> parse_emcy(const CANFrame& frame);

    // ==================== LSS ====================

    /**
     * @brief Create LSS inquiry frame
     */
    static CANFrame create_lss_inquiry(LSSCommand cmd, const uint8_t* data = nullptr,
                                        size_t data_len = 0);

    /**
     * @brief Create LSS response frame
     */
    static CANFrame create_lss_response(const uint8_t* data, size_t data_len);

    /**
     * @brief Parse LSS message
     */
    struct LSSMessage {
        LSSCommand command;
        std::vector<uint8_t> data;
        bool is_response;
    };

    static std::optional<LSSMessage> parse_lss(const CANFrame& frame);

    // ==================== Utility ====================

    /**
     * @brief Get message type from COB-ID
     */
    static std::string get_message_type_name(uint32_t cob_id);

    /**
     * @brief Check if COB-ID is valid CANopen frame
     */
    static bool is_canopen_cobid(uint32_t cob_id);
};

// ==================== Inline Implementations ====================

inline uint32_t COBID::emcy(uint8_t node_id) {
    return EMCY_BASE + (node_id & 0x7F);
}

inline uint32_t COBID::tpdo_tx(uint8_t node_id, uint8_t pdo_num) {
    switch (pdo_num) {
        case 1: return TPDO1_BASE + (node_id & 0x7F);
        case 2: return TPDO2_BASE + (node_id & 0x7F);
        case 3: return TPDO3_BASE + (node_id & 0x7F);
        case 4: return TPDO4_BASE + (node_id & 0x7F);
        default: return 0;
    }
}

inline uint32_t COBID::tpdo_rx(uint8_t node_id, uint8_t pdo_num) {
    switch (pdo_num) {
        case 1: return RPDO1_BASE + (node_id & 0x7F);
        case 2: return RPDO2_BASE + (node_id & 0x7F);
        case 3: return RPDO3_BASE + (node_id & 0x7F);
        case 4: return RPDO4_BASE + (node_id & 0x7F);
        default: return 0;
    }
}

inline uint32_t COBID::sdo_tx(uint8_t node_id) {
    return TSDO_BASE + (node_id & 0x7F);
}

inline uint32_t COBID::sdo_rx(uint8_t node_id) {
    return RSDO_BASE + (node_id & 0x7F);
}

inline uint32_t COBID::heartbeat(uint8_t node_id) {
    return HEARTBEAT_BASE + (node_id & 0x7F);
}

inline bool COBID::is_valid(uint32_t cob_id) {
    return (cob_id & ~0x7FF) == 0;
}

inline bool COBID::is_valid_node_id(uint8_t node_id) {
    return node_id >= 1 && node_id <= 127;
}

inline size_t PDOMapping::data_size() const {
    size_t total = 0;
    for (const auto& entry : entries) {
        total += (entry.bit_length + 7) / 8;
    }
    return total;
}

} // namespace canopen

#endif // CANOPEN_CAN_MSG_MESSAGE_FACTORY_HPP
