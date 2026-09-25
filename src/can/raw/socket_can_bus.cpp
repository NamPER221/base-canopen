/**
 * @file socket_can_bus.cpp
 * @brief SocketCAN implementation of BusInterface
 */

#include <canopen/can/raw/socket_can_bus.hpp>
#include <cstring>
#include <cerrno>

namespace canopen {

SocketCanBus::SocketCanBus(const std::string& interface, bool loopback,
                           bool error_frames)
    : socket_(interface) {
    if (socket_.open(loopback) < 0) {
        return;
    }
    if (error_frames) {
        socket_.bind_error_frame();
        socket_.start_monitoring([this](const CANErrorFrame& err) {
            handle_error(err);
        });
    }
}

SocketCanBus::~SocketCanBus() {
    close();
}

int SocketCanBus::open() {
    if (!socket_.is_open()) {
        const int rc = socket_.open(false);
        if (rc < 0) {
            return rc;
        }
    }

    // Start RX thread (idempotent — constructor may have opened the socket)
    if (!running_.load()) {
        running_.store(true);
        rx_thread_ = std::thread(&SocketCanBus::rx_loop, this);
    }
    return 0;
}

void SocketCanBus::close() {
    running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    socket_.stop_monitoring();
    socket_.close();
}

bool SocketCanBus::send(const CANFrame& frame) {
    if (!socket_.is_open()) {
        return false;
    }
    can_frame native = frame.to_native();
    ssize_t n = socket_.write(native);
    return n > 0;
}

bool SocketCanBus::is_up() const {
    if (!socket_.is_open()) {
        return false;
    }
    BusState state;
    if (const_cast<SocketCAN&>(socket_).get_bus_state(state) != 0) {
        // Fallback: socket open means we assume usable
        return true;
    }
    return state != BusState::BUS_OFF && state != BusState::STOPPED;
}

void SocketCanBus::rx_loop() {
    while (running_.load()) {
        can_frame native{};
        struct timespec timeout{0, 100 * 1000 * 1000};  // 100 ms poll

        ssize_t n = socket_.read(native, &timeout);
        if (n <= 0) {
            continue;  // timeout or transient error
        }

        if (native.can_id & CAN_ERR_FLAG) {
            CANErrorFrame err{};
            err.error_code = native.can_id & CAN_ERR_MASK;
            err.tx_error_counter = native.data[0];
            err.rx_error_counter = native.data[1];
            err.error_type = native.data[2];
            handle_error(err);
            continue;
        }

        dispatch(CANFrame::from_native(native));
    }
}

void SocketCanBus::handle_error(const CANErrorFrame& err) {
    last_error_.store(err.error_code);
    if (on_error_frame) {
        on_error_frame(err);
    }
}

} // namespace canopen
