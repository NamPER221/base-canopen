/**
 * @file fake_bus.hpp
 * @brief In-memory fake CAN bus for unit tests (no hardware required)
 *
 * - `send()` records the frame and echoes it back to routes
 *   (simulating loopback), so SDO/NMT/PDO exchanges can be tested
 *   end-to-end.
 * - `pop_sent()` lets tests inspect transmitted frames.
 */

#ifndef CANOPEN_TEST_MOCK_FAKE_BUS_HPP
#define CANOPEN_TEST_MOCK_FAKE_BUS_HPP

#include <canopen/can/raw/bus_interface.hpp>
#include <deque>
#include <mutex>
#include <vector>

namespace canopen {
namespace test {

class FakeBus : public BusInterface {
public:
    bool send(const CANFrame& frame) override {
        {
            std::lock_guard<std::mutex> lock(sent_mutex_);
            sent_frames_.push_back(frame);
            send_count_++;
        }
        // Echo to routes (simulates loopback) unless echo disabled
        if (echo_enabled_) {
            dispatch(frame);
        }
        return true;
    }

    bool is_up() const override { return up_; }

    // ==================== Test Helpers ====================

    void set_up(bool up) { up_ = up; }

    /// Enable/disable echo on send (default on)
    void set_echo(bool enable) { echo_enabled_ = enable; }

    /// Last sent frame (empty if none)
    CANFrame last_sent() {
        std::lock_guard<std::mutex> lock(sent_mutex_);
        if (sent_frames_.empty()) return CANFrame{};
        return sent_frames_.back();
    }

    /// All sent frames
    std::vector<CANFrame> sent() {
        std::lock_guard<std::mutex> lock(sent_mutex_);
        return std::vector<CANFrame>(sent_frames_.begin(), sent_frames_.end());
    }

    /// Total number of sent frames
    size_t send_count() {
        std::lock_guard<std::mutex> lock(sent_mutex_);
        return send_count_;
    }

    /// Clear recorded frames
    void clear_sent() {
        std::lock_guard<std::mutex> lock(sent_mutex_);
        sent_frames_.clear();
        send_count_ = 0;
    }

private:
    std::deque<CANFrame> sent_frames_;
    std::mutex sent_mutex_;
    size_t send_count_{0};
    bool up_{true};
    bool echo_enabled_{true};
};

} // namespace test
} // namespace canopen

#endif // CANOPEN_TEST_MOCK_FAKE_BUS_HPP
