/**
 * @file socket_can_bus.hpp
 * @brief SocketCAN implementation of BusInterface
 *
 * Wraps SocketCAN and runs an RX thread that dispatches received frames
 * to the registered routes. Also captures CAN error frames and exposes
 * bus health.
 */

#ifndef CANOPEN_CAN_RAW_SOCKET_CAN_BUS_HPP
#define CANOPEN_CAN_RAW_SOCKET_CAN_BUS_HPP

#include <canopen/can/raw/bus_interface.hpp>
#include <canopen/can/raw/socket_can.hpp>
#include <atomic>
#include <string>
#include <thread>

namespace canopen {

class SocketCanBus : public BusInterface {
public:
    /**
     * @brief Construct SocketCAN bus
     * @param interface CAN interface name (e.g., "can0", "vcan0")
     * @param loopback Enable loopback (frames sent are also received)
     * @param error_frames Bind error frame reception
     */
    explicit SocketCanBus(const std::string& interface, bool loopback = false,
                          bool error_frames = false);
    ~SocketCanBus() override;

    // Non-copyable
    SocketCanBus(const SocketCanBus&) = delete;
    SocketCanBus& operator=(const SocketCanBus&) = delete;

    /**
     * @brief Open the CAN interface and start the RX thread
     * @return 0 on success, negative errno on error
     */
    int open();

    /**
     * @brief Close the interface and stop the RX thread
     */
    void close();

    // ==================== BusInterface ====================

    bool send(const CANFrame& frame) override;
    bool is_up() const override;

    // ==================== Access ====================

    /**
     * @brief Direct access to the underlying SocketCAN (filters, stats, ...)
     */
    SocketCAN& socket() { return socket_; }
    const SocketCAN& socket() const { return socket_; }

    const std::string& interface_name() const { return socket_.interface(); }

    /**
     * @brief Last CAN error code observed (0 = none)
     */
    uint32_t last_error() const { return last_error_.load(); }

    /**
     * @brief Callback invoked on CAN error frames (bus-off, ack error, ...)
     */
    std::function<void(const CANErrorFrame&)> on_error_frame;

private:
    void rx_loop();
    void handle_error(const CANErrorFrame& err);

    SocketCAN socket_;
    std::thread rx_thread_;
    std::atomic<bool> running_{false};
    std::atomic<uint32_t> last_error_{0};
};

} // namespace canopen

#endif // CANOPEN_CAN_RAW_SOCKET_CAN_BUS_HPP
