/**
 * @file socket_can.hpp
 * @brief SocketCAN abstraction layer for Linux CAN interfaces
 *
 * This provides deep CAN_RAW and CAN_ADMIN intervention for:
 * - CAN frame read/write operations
 * - CAN filter management
 * - CAN bus state control and monitoring
 * - Error frame handling
 */

#ifndef CANOPEN_CAN_RAW_SOCKET_CAN_HPP
#define CANOPEN_CAN_RAW_SOCKET_CAN_HPP

#include <cstdint>
#include <cstddef>
#include <string>
#include <functional>
#include <chrono>
#include <optional>
#include <atomic>
#include <thread>
#include <mutex>
#include <array>

// Linux CAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

namespace canopen {

/**
 * @brief CAN bus state enumeration
 */
enum class BusState : uint8_t {
    UNKNOWN = 0,
    ACTIVE = 1,
    WARNING = 2,
    PASSIVE = 3,
    BUS_OFF = 4,
    STOPPED = 5
};

/**
 * @brief CAN error types
 */
enum class CANError : uint32_t {
    NONE            = 0,
    TX_TIMEOUT      = CAN_ERR_TX_TIMEOUT,
    LOST_ARB        = CAN_ERR_LOSTARB,
    CTRL_ERROR      = CAN_ERR_CRTL,
    PROTOCOL_ERROR  = CAN_ERR_PROT,
    TRANSCEIVER     = CAN_ERR_TRX,
    NO_ACK          = CAN_ERR_ACK,
    BUS_OFF         = CAN_ERR_BUSOFF,
    BUS_ERROR       = CAN_ERR_BUSERROR,
    RESTARTED       = CAN_ERR_RESTARTED,
};

/**
 * @brief CAN statistics
 */
struct CANStats {
    uint64_t tx_frames;
    uint64_t rx_frames;
    uint64_t tx_errors;
    uint64_t rx_errors;
    uint64_t bus_errors;
    uint32_t arbitration_lost;
    uint8_t tx_error_counter;
    uint8_t rx_error_counter;
    BusState bus_state;
};

/**
 * @brief CAN filter structure
 */
struct CANFilter {
    uint32_t can_id;
    uint32_t can_mask;

    bool is_extended() const { return can_id & CAN_EFF_FLAG; }
    bool is_remote() const { return can_id & CAN_RTR_FLAG; }
};

/**
 * @brief CAN frame with timestamp
 */
struct CANFrameWithTime {
    can_frame frame;
    timespec timestamp;
};

/**
 * @brief Error frame data
 */
struct CANErrorFrame {
    uint32_t error_code;
    uint8_t tx_error_counter;
    uint8_t rx_error_counter;
    uint8_t error_type;
    uint8_t padding[5];
};

/**
 * @brief SocketCAN abstraction class
 *
 * Provides a C++ interface to Linux SocketCAN with deep intervention
 * into CAN_RAW and CAN_ADMIN sockets.
 */
class SocketCAN {
public:
    /**
     * @brief Construct SocketCAN with interface name
     * @param interface CAN interface name (e.g., "can0")
     */
    explicit SocketCAN(const std::string& interface);

    /**
     * @brief Destructor - closes socket if open
     */
    ~SocketCAN();

    // Non-copyable
    SocketCAN(const SocketCAN&) = delete;
    SocketCAN& operator=(const SocketCAN&) = delete;

    // Movable
    SocketCAN(SocketCAN&& other) noexcept;
    SocketCAN& operator=(SocketCAN&& other) noexcept;

    // ==================== Lifecycle ====================

    /**
     * @brief Open CAN interface
     * @param loopback Enable loopback mode (for testing)
     * @return 0 on success, negative errno on error
     */
    int open(bool loopback = false);

    /**
     * @brief Open CAN-FD interface
     * @param loopback Enable loopback mode
     * @return 0 on success, negative errno on error
     */
    int open_fd(bool loopback = false);

    /**
     * @brief Close CAN interface
     */
    void close();

    /**
     * @brief Check if interface is open
     */
    bool is_open() const { return sock_fd_.load() >= 0; }

    /**
     * @brief Get interface name
     */
    const std::string& interface() const { return interface_; }

    // ==================== CAN_RAW Operations ====================

    /**
     * @brief Set CAN filters
     * @param filters Array of filters
     * @param count Number of filters
     * @return 0 on success, negative errno on error
     */
    int set_filter(const CANFilter* filters, size_t count);

    /**
     * @brief Set single filter (accept all with mask)
     * @param can_mask CAN mask (e.g., 0x7FF for standard)
     * @return 0 on success, negative errno on error
     */
    int set_mask(uint32_t can_mask);

    /**
     * @brief Clear all filters
     * @return 0 on success, negative errno on error
     */
    int clear_filters();

    /**
     * @brief Enable own CAN ID filter (ignore frames from self)
     * @param enable Enable/Disable
     * @return 0 on success, negative errno on error
     */
    int enable_own_filter(bool enable);

    /**
     * @brief Bind error frame filter
     * @return 0 on success, negative errno on error
     */
    int bind_error_frame();

    // ==================== Read/Write ====================

    /**
     * @brief Read CAN frame with timeout
     * @param frame Output frame buffer
     * @param timeout Timeout (nullptr = blocking)
     * @return bytes read, 0 on timeout, negative on error
     */
    ssize_t read(can_frame& frame, struct timespec* timeout = nullptr);

    /**
     * @brief Read CAN-FD frame with timeout
     * @param frame Output FD frame buffer
     * @param timeout Timeout (nullptr = blocking)
     * @return bytes read, 0 on timeout, negative on error
     */
    ssize_t read_fd(canfd_frame& frame, struct timespec* timeout = nullptr);

    /**
     * @brief Write CAN frame
     * @param frame Frame to write
     * @return bytes written, negative on error
     */
    ssize_t write(const can_frame& frame);

    /**
     * @brief Write CAN-FD frame
     * @param frame FD frame to write
     * @return bytes written, negative on error
     */
    ssize_t write_fd(const canfd_frame& frame);

    /**
     * @brief Write raw data (for PDO burst)
     * @param data Pointer to data
     * @param len Length in bytes
     * @return bytes written, negative on error
     */
    ssize_t write_raw(const void* data, size_t len);

    // ==================== CAN_ADMIN Operations ====================

    /**
     * @brief Set bus state (up/down)
     * @param state Desired state
     * @return 0 on success, negative errno on error
     */
    int set_bus_state(BusState state);

    /**
     * @brief Get current bus state
     * @param state Output state
     * @return 0 on success, negative errno on error
     */
    int get_bus_state(BusState& state);

    /**
     * @brief Get CAN statistics
     * @param stats Output statistics
     * @return 0 on success, negative errno on error
     */
    int get_stats(CANStats& stats);

    /**
     * @brief Get auto-restart delay (ms)
     * @param ms Output delay in milliseconds
     * @return 0 on success, negative errno on error
     */
    int get_restart_ms(uint32_t& ms);

    /**
     * @brief Set auto-restart delay (ms)
     * @param ms Delay in milliseconds (0 = disabled)
     * @return 0 on success, negative errno on error
     */
    int set_restart_ms(uint32_t ms);

    /**
     * @brief Manual bus restart
     * @return 0 on success, negative errno on error
     */
    int restart();

    /**
     * @brief Get error frame mask
     * @return error mask (0 = no error frames)
     */
    int get_error_mask() const;

    // ==================== File Descriptor ====================

    /**
     * @brief Get raw socket file descriptor
     * For use with select/poll/epoll
     */
    int get_fd() const { return sock_fd_.load(); }

    /**
     * @brief Get error fd (for error frame reception)
     */
    int get_error_fd() const { return err_fd_.load(); }

    // ==================== Monitoring ====================

    /**
     * @brief Start error monitoring thread
     * @param callback Called on error frames
     */
    void start_monitoring(std::function<void(const CANErrorFrame&)> callback);

    /**
     * @brief Stop error monitoring
     */
    void stop_monitoring();

    /**
     * @brief Check if bus is operational
     */
    bool is_operational() const;

    /**
     * @brief Check if bus is in bus-off state
     */
    bool is_bus_off() const;

    /**
     * @brief Check for error passive state
     */
    bool is_error_passive() const;

    /**
     * @brief Reset error counters
     */
    void reset_error_counters();

    // ==================== Utility ====================

    /**
     * @brief Get last error string
     */
    const char* get_error_str() const;

    /**
     * @brief Get current error number
     */
    int get_error_num() const { return errno_; }

private:
    int open_common(int type, bool loopback);
    int apply_filters();
    void update_error_counters();
    int handle_netlink_error(const char* operation);

    std::string interface_;
    std::atomic<int> sock_fd_{-1};
    std::atomic<int> err_fd_{-1};
    std::atomic<BusState> state_{BusState::UNKNOWN};

    std::vector<CANFilter> filters_;
    mutable std::mutex filter_mutex_;

    std::atomic<uint8_t> tx_error_counter_{0};
    std::atomic<uint8_t> rx_error_counter_{0};

    std::thread monitor_thread_;
    std::atomic<bool> monitoring_{false};
    std::function<void(const CANErrorFrame&)> error_callback_;

    int errno_{0};

    CANStats stats_{};
    mutable std::mutex stats_mutex_;
};

/**
 * @brief Create SocketCAN with RAII semantics
 */
using SocketCANPtr = std::unique_ptr<SocketCAN>;

} // namespace canopen

#endif // CANOPEN_CAN_RAW_SOCKET_CAN_HPP
