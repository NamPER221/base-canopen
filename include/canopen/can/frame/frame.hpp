/**
 * @file frame.hpp
 * @brief CAN Frame Builder/Parser for CANopen
 */

#ifndef CANOPEN_CAN_FRAME_FRAME_HPP
#define CANOPEN_CAN_FRAME_FRAME_HPP

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>
#include <cstdint>
#include <array>
#include <vector>
#include <optional>
#include <span>

namespace canopen {

/**
 * @brief CAN Frame class with CANopen support
 *
 * Provides high-level CAN frame manipulation with support for:
 * - CAN 2.0 (Standard and Extended)
 * - CAN FD
 * - RTR (Remote Transmission Request)
 * - Error frames
 */
class CANFrame {
public:
    // ==================== Constructors ====================

    /**
     * @brief Default constructor
     */
    CANFrame() = default;

    /**
     * @brief Construct from CAN ID, data, and length
     */
    CANFrame(uint32_t can_id, const uint8_t* data, uint8_t len);

    /**
     * @brief Construct from native can_frame
     */
    explicit CANFrame(const can_frame& frame);

    /**
     * @brief Construct CAN-FD frame
     */
    explicit CANFrame(const canfd_frame& frame);

    // ==================== Accessors ====================

    /**
     * @brief Get CAN ID
     */
    uint32_t id() const { return can_id_; }

    /**
     * @brief Set CAN ID
     */
    void set_id(uint32_t id) { can_id_ = id; }

    /**
     * @brief Check if extended ID
     */
    bool is_extended_id() const { return can_id_ & CAN_EFF_FLAG; }

    /**
     * @brief Set extended ID flag
     */
    void set_extended_id(bool enable) {
        if (enable) can_id_ |= CAN_EFF_FLAG;
        else can_id_ &= ~CAN_EFF_FLAG;
    }

    /**
     * @brief Check if RTR (Remote Transmission Request)
     */
    bool is_rtr() const { return can_id_ & CAN_RTR_FLAG; }

    /**
     * @brief Set RTR flag
     */
    void set_rtr(bool enable) {
        if (enable) can_id_ |= CAN_RTR_FLAG;
        else can_id_ &= ~CAN_RTR_FLAG;
    }

    /**
     * @brief Check if error frame
     */
    bool is_error() const { return can_id_ & CAN_ERR_FLAG; }

    /**
     * @brief Get data length (0-8 for CAN 2.0, 0-64 for CAN-FD)
     */
    uint8_t len() const { return len_; }

    /**
     * @brief Set data length
     */
    void set_len(uint8_t len) { len_ = len; }

    /**
     * @brief Get data pointer
     */
    const uint8_t* data() const { return data_.data(); }

    /**
     * @brief Get mutable data pointer
     */
    uint8_t* data() { return data_.data(); }

    /**
     * @brief Get payload as span
     */
    std::span<const uint8_t> payload() const { return {data_.data(), len_}; }

    /**
     * @brief Check if CAN-FD frame
     */
    bool is_fd() const { return flags_ & CANFD_FDF; }

    /**
     * @brief Set CAN-FD flag
     */
    void set_fd(bool enable) {
        if (enable) flags_ |= CANFD_FDF;
        else flags_ &= ~CANFD_FDF;
    }

    /**
     * @brief Check BRS (Bit Rate Switch)
     */
    bool has_brs() const { return flags_ & CANFD_BRS; }

    /**
     * @brief Set BRS flag
     */
    void set_brs(bool enable) {
        if (enable) flags_ |= CANFD_BRS;
        else flags_ &= ~CANFD_BRS;
    }

    /**
     * @brief Check ESI (Error State Indicator)
     */
    bool has_esi() const { return flags_ & CANFD_ESI; }

    // ==================== Data Access Helpers ====================

    /**
     * @brief Read 8-bit value at offset
     */
    uint8_t get_u8(size_t offset) const;

    /**
     * @brief Read 16-bit value (little-endian) at offset
     */
    uint16_t get_u16_le(size_t offset) const;

    /**
     * @brief Read 16-bit value (big-endian) at offset
     */
    uint16_t get_u16_be(size_t offset) const;

    /**
     * @brief Read 32-bit value (little-endian) at offset
     */
    uint32_t get_u32_le(size_t offset) const;

    /**
     * @brief Read 32-bit value (big-endian) at offset
     */
    uint32_t get_u32_be(size_t offset) const;

    /**
     * @brief Read 64-bit value (little-endian) at offset
     */
    uint64_t get_u64_le(size_t offset) const;

    /**
     * @brief Write 8-bit value at offset
     */
    void set_u8(size_t offset, uint8_t value);

    /**
     * @brief Write 16-bit value (little-endian) at offset
     */
    void set_u16_le(size_t offset, uint16_t value);

    /**
     * @brief Write 16-bit value (big-endian) at offset
     */
    void set_u16_be(size_t offset, uint16_t value);

    /**
     * @brief Write 32-bit value (little-endian) at offset
     */
    void set_u32_le(size_t offset, uint32_t value);

    /**
     * @brief Write 32-bit value (big-endian) at offset
     */
    void set_u32_be(size_t offset, uint32_t value);

    /**
     * @brief Write 64-bit value (little-endian) at offset
     */
    void set_u64_le(size_t offset, uint64_t value);

    // ==================== Conversion ====================

    /**
     * @brief Get native can_frame reference
     */
    const can_frame& to_native() const;

    /**
     * @brief Get native canfd_frame reference
     */
    const canfd_frame& to_native_fd() const;

    /**
     * @brief Create CANFrame from native can_frame
     */
    static CANFrame from_native(const can_frame& frame);

    /**
     * @brief Create CANFrame from native canfd_frame
     */
    static CANFrame from_native(const canfd_frame& frame);

    /**
     * @brief Create CANFrame from raw CAN ID and data
     */
    static CANFrame create(uint32_t can_id, std::span<const uint8_t> data);

    /**
     * @brief Create RTR frame
     */
    static CANFrame create_rtr(uint32_t can_id, uint8_t dlc);

private:
    uint32_t can_id_{0};
    uint8_t len_{0};
    uint8_t flags_{0};
    std::array<uint8_t, 64> data_{};
    mutable can_frame native_frame_{};
    mutable canfd_frame native_fd_frame_{};
    mutable bool native_valid_{false};
    mutable bool native_fd_valid_{false};
};

// ==================== Inline Implementations ====================

inline uint8_t CANFrame::get_u8(size_t offset) const {
    if (offset >= len_) return 0;
    return data_[offset];
}

inline uint16_t CANFrame::get_u16_le(size_t offset) const {
    if (offset + 2 > len_) return 0;
    return static_cast<uint16_t>(data_[offset]) |
           (static_cast<uint16_t>(data_[offset + 1]) << 8);
}

inline uint16_t CANFrame::get_u16_be(size_t offset) const {
    if (offset + 2 > len_) return 0;
    return (static_cast<uint16_t>(data_[offset]) << 8) |
           static_cast<uint16_t>(data_[offset + 1]);
}

inline uint32_t CANFrame::get_u32_le(size_t offset) const {
    if (offset + 4 > len_) return 0;
    return static_cast<uint32_t>(data_[offset]) |
           (static_cast<uint32_t>(data_[offset + 1]) << 8) |
           (static_cast<uint32_t>(data_[offset + 2]) << 16) |
           (static_cast<uint32_t>(data_[offset + 3]) << 24);
}

inline uint32_t CANFrame::get_u32_be(size_t offset) const {
    if (offset + 4 > len_) return 0;
    return (static_cast<uint32_t>(data_[offset]) << 24) |
           (static_cast<uint32_t>(data_[offset + 1]) << 16) |
           (static_cast<uint32_t>(data_[offset + 2]) << 8) |
           static_cast<uint32_t>(data_[offset + 3]);
}

inline uint64_t CANFrame::get_u64_le(size_t offset) const {
    if (offset + 8 > len_) return 0;
    uint64_t val = 0;
    for (int i = 0; i < 8; ++i) {
        val |= (static_cast<uint64_t>(data_[offset + i]) << (i * 8));
    }
    return val;
}

inline void CANFrame::set_u8(size_t offset, uint8_t value) {
    if (offset >= len_) return;
    data_[offset] = value;
    native_valid_ = false;
    native_fd_valid_ = false;
}

inline void CANFrame::set_u16_le(size_t offset, uint16_t value) {
    if (offset + 2 > len_) return;
    data_[offset] = static_cast<uint8_t>(value & 0xFF);
    data_[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    native_valid_ = false;
    native_fd_valid_ = false;
}

inline void CANFrame::set_u16_be(size_t offset, uint16_t value) {
    if (offset + 2 > len_) return;
    data_[offset] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data_[offset + 1] = static_cast<uint8_t>(value & 0xFF);
    native_valid_ = false;
    native_fd_valid_ = false;
}

inline void CANFrame::set_u32_le(size_t offset, uint32_t value) {
    if (offset + 4 > len_) return;
    for (int i = 0; i < 4; ++i) {
        data_[offset + i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
    native_valid_ = false;
    native_fd_valid_ = false;
}

inline void CANFrame::set_u32_be(size_t offset, uint32_t value) {
    if (offset + 4 > len_) return;
    for (int i = 0; i < 4; ++i) {
        data_[offset + 3 - i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
    native_valid_ = false;
    native_fd_valid_ = false;
}

inline void CANFrame::set_u64_le(size_t offset, uint64_t value) {
    if (offset + 8 > len_) return;
    for (int i = 0; i < 8; ++i) {
        data_[offset + i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
    native_valid_ = false;
    native_fd_valid_ = false;
}

} // namespace canopen

#endif // CANOPEN_CAN_FRAME_FRAME_HPP
