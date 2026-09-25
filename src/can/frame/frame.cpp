/**
 * @file frame.cpp
 * @brief CAN Frame implementation
 */

#include <canopen/can/frame/frame.hpp>
#include <cstring>

namespace canopen {

CANFrame::CANFrame(uint32_t can_id, const uint8_t* data, uint8_t len)
    : can_id_(can_id), len_(std::min(len, static_cast<uint8_t>(64))) {
    if (data && len_ > 0) {
        std::memcpy(data_.data(), data, len_);
    }
    native_valid_ = false;
    native_fd_valid_ = false;
}

CANFrame::CANFrame(const can_frame& frame)
    : can_id_(frame.can_id), len_(frame.can_dlc) {
    std::memcpy(data_.data(), frame.data, frame.can_dlc);
    native_valid_ = false;
    native_fd_valid_ = false;
}

CANFrame::CANFrame(const canfd_frame& frame)
    : can_id_(frame.can_id), len_(frame.len), flags_(frame.flags) {
    std::memcpy(data_.data(), frame.data, frame.len);
    native_valid_ = false;
    native_fd_valid_ = false;
}

const can_frame& CANFrame::to_native() const {
    if (!native_valid_) {
        std::memset(&native_frame_, 0, sizeof(native_frame_));
        native_frame_.can_id = can_id_;
        native_frame_.can_dlc = len_;
        std::memcpy(native_frame_.data, data_.data(), len_);
        native_valid_ = true;
    }
    return native_frame_;
}

const canfd_frame& CANFrame::to_native_fd() const {
    if (!native_fd_valid_) {
        std::memset(&native_fd_frame_, 0, sizeof(native_fd_frame_));
        native_fd_frame_.can_id = can_id_;
        native_fd_frame_.len = len_;
        native_fd_frame_.flags = flags_;
        std::memcpy(native_fd_frame_.data, data_.data(), len_);
        native_fd_valid_ = true;
    }
    return native_fd_frame_;
}

CANFrame CANFrame::from_native(const can_frame& frame) {
    return CANFrame(frame);
}

CANFrame CANFrame::from_native(const canfd_frame& frame) {
    return CANFrame(frame);
}

CANFrame CANFrame::create(uint32_t can_id, std::span<const uint8_t> data) {
    return CANFrame(can_id, data.data(), static_cast<uint8_t>(data.size()));
}

CANFrame CANFrame::create_rtr(uint32_t can_id, uint8_t dlc) {
    CANFrame frame;
    frame.set_id(can_id | CAN_RTR_FLAG);
    frame.set_len(dlc);
    return frame;
}

} // namespace canopen
