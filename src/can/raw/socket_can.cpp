/**
 * @file socket_can.cpp
 * @brief SocketCAN implementation for Linux CAN interfaces
 */

#include <canopen/can/raw/socket_can.hpp>
#include <algorithm>
#include <cstring>
#include <unistd.h>
#include <errno.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>

namespace canopen {

SocketCAN::SocketCAN(const std::string& interface)
    : interface_(interface) {
}

SocketCAN::~SocketCAN() {
    close();
}

SocketCAN::SocketCAN(SocketCAN&& other) noexcept
    : interface_(std::move(other.interface_))
    , sock_fd_(other.sock_fd_.load())
    , err_fd_(other.err_fd_.load())
    , state_(other.state_.load())
    , filters_(std::move(other.filters_))
    , tx_error_counter_(other.tx_error_counter_.load())
    , rx_error_counter_(other.rx_error_counter_.load()) {
    other.sock_fd_.store(-1);
    other.err_fd_.store(-1);
}

SocketCAN& SocketCAN::operator=(SocketCAN&& other) noexcept {
    if (this != &other) {
        close();
        interface_ = std::move(other.interface_);
        sock_fd_.store(other.sock_fd_.load());
        err_fd_.store(other.err_fd_.load());
        state_.store(other.state_.load());
        filters_ = std::move(other.filters_);
        tx_error_counter_.store(other.tx_error_counter_.load());
        rx_error_counter_.store(other.rx_error_counter_.load());
        other.sock_fd_.store(-1);
        other.err_fd_.store(-1);
    }
    return *this;
}

int SocketCAN::open_common(int type, bool loopback) {
    close();

    // Create socket (protocol MUST be CAN_RAW, not 0 — protocol 0
    // is unregistered under PF_CAN and fails with EPROTONOSUPPORT)
    int fd = ::socket(PF_CAN, type, CAN_RAW);
    if (fd < 0) {
        errno_ = errno;
        return -1;
    }

    // Set loopback mode
    if (loopback) {
        int loop = 1;
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loop, sizeof(loop)) < 0) {
            errno_ = errno;
            ::close(fd);
            return -1;
        }
    }

    // Get interface index
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        errno_ = errno;
        ::close(fd);
        return -1;
    }

    // Bind socket
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        errno_ = errno;
        ::close(fd);
        return -1;
    }

    sock_fd_.store(fd);
    state_.store(BusState::ACTIVE);

    // Apply filters if any
    apply_filters();

    return 0;
}

int SocketCAN::open(bool loopback) {
    return open_common(SOCK_RAW, loopback);
}

int SocketCAN::open_fd(bool loopback) {
    return open_common(SOCK_RAW, loopback);
}

void SocketCAN::close() {
    int fd = sock_fd_.load();
    if (fd >= 0) {
        ::close(fd);
        sock_fd_.store(-1);
    }

    fd = err_fd_.load();
    if (fd >= 0) {
        ::close(fd);
        err_fd_.store(-1);
    }

    stop_monitoring();
    state_.store(BusState::UNKNOWN);
}

int SocketCAN::set_filter(const CANFilter* filters, size_t count) {
    if (!is_open()) return -ENOTCONN;

    std::lock_guard<std::mutex> lock(filter_mutex_);
    filters_.clear();
    for (size_t i = 0; i < count; ++i) {
        filters_.push_back(filters[i]);
    }

    return apply_filters();
}

int SocketCAN::set_mask(uint32_t can_mask) {
    CANFilter filter;
    filter.can_id = 0;
    filter.can_mask = can_mask;
    return set_filter(&filter, 1);
}

int SocketCAN::clear_filters() {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    filters_.clear();

    if (!is_open()) return 0;

    // Set filter to accept all
    CANFilter filter = {0, 0};
    return set_filter(&filter, 1);
}

int SocketCAN::enable_own_filter(bool enable) {
    if (!is_open()) return -ENOTCONN;

    int loop = enable ? 0 : 1;
    // CAN_RAW_RECV_OWN_MSGS controls receiving own messages
    return setsockopt(sock_fd_.load(), SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loop, sizeof(loop));
}

int SocketCAN::bind_error_frame() {
    int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        errno_ = errno;
        return -1;
    }

    // Get interface index
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        errno_ = errno;
        ::close(fd);
        return -1;
    }

    // Bind to error frame filter
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        errno_ = errno;
        ::close(fd);
        return -1;
    }

    err_fd_.store(fd);
    return 0;
}

int SocketCAN::apply_filters() {
    if (!is_open()) return -ENOTCONN;

    std::lock_guard<std::mutex> lock(filter_mutex_);

    if (filters_.empty()) {
        return 0;  // No filters to apply
    }

    std::vector<struct can_filter> native_filters;
    for (const auto& f : filters_) {
        struct can_filter nf;
        nf.can_id = f.can_id;
        nf.can_mask = f.can_mask;
        native_filters.push_back(nf);
    }

    return setsockopt(sock_fd_.load(), SOL_CAN_RAW, CAN_RAW_FILTER,
                      native_filters.data(), native_filters.size() * sizeof(struct can_filter));
}

ssize_t SocketCAN::read(can_frame& frame, struct timespec* timeout) {
    int fd = sock_fd_.load();
    if (fd < 0) {
        errno_ = ENOTCONN;
        return -1;
    }

    if (timeout) {
        fd_set rdfds;
        FD_ZERO(&rdfds);
        FD_SET(fd, &rdfds);

        struct timespec tv = *timeout;
        int ret = pselect(fd + 1, &rdfds, nullptr, nullptr, &tv, nullptr);

        if (ret < 0) {
            errno_ = errno;
            return -1;
        }
        if (ret == 0) {
            return 0;  // Timeout
        }
    }

    ssize_t n = ::read(fd, &frame, sizeof(frame));
    if (n < 0) {
        errno_ = errno;
        update_error_counters();
    }

    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (n > 0) {
        stats_.rx_frames++;
    }

    return n;
}

ssize_t SocketCAN::read_fd(canfd_frame& frame, struct timespec* timeout) {
    int fd = sock_fd_.load();
    if (fd < 0) {
        errno_ = ENOTCONN;
        return -1;
    }

    if (timeout) {
        fd_set rdfds;
        FD_ZERO(&rdfds);
        FD_SET(fd, &rdfds);

        struct timespec tv = *timeout;
        int ret = pselect(fd + 1, &rdfds, nullptr, nullptr, &tv, nullptr);

        if (ret < 0) {
            errno_ = errno;
            return -1;
        }
        if (ret == 0) {
            return 0;
        }
    }

    ssize_t n = ::read(fd, &frame, sizeof(frame));
    if (n < 0) {
        errno_ = errno;
    }

    return n;
}

ssize_t SocketCAN::write(const can_frame& frame) {
    int fd = sock_fd_.load();
    if (fd < 0) {
        errno_ = ENOTCONN;
        return -1;
    }

    ssize_t n = ::write(fd, &frame, sizeof(frame));
    if (n < 0) {
        errno_ = errno;
        update_error_counters();

        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.tx_errors++;
    } else {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.tx_frames++;
    }

    return n;
}

ssize_t SocketCAN::write_fd(const canfd_frame& frame) {
    int fd = sock_fd_.load();
    if (fd < 0) {
        errno_ = ENOTCONN;
        return -1;
    }

    ssize_t n = ::write(fd, &frame, sizeof(frame));
    if (n < 0) {
        errno_ = errno;
    }

    return n;
}

ssize_t SocketCAN::write_raw(const void* data, size_t len) {
    int fd = sock_fd_.load();
    if (fd < 0) {
        errno_ = ENOTCONN;
        return -1;
    }

    struct iovec iov[2];
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    iov[0].iov_base = &frame;
    iov[0].iov_len = sizeof(frame);
    iov[1].iov_base = const_cast<void*>(data);
    iov[1].iov_len = len;

    ssize_t n = writev(fd, iov, 2);
    if (n < 0) {
        errno_ = errno;
    }

    return n;
}

int SocketCAN::set_bus_state(BusState state) {
    // Use netlink to set bus state
    int sock = ::socket(PF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (sock < 0) {
        errno_ = errno;
        return -1;
    }

    struct {
        nlmsghdr nlh;
        ifinfomsg ifi;
    } req;

    std::memset(&req, 0, sizeof(req));
    req.nlh.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
    req.nlh.nlmsg_type = RTM_NEWLINK;
    req.nlh.nlmsg_flags = NLM_F_REQUEST;
    req.nlh.nlmsg_seq = 1;

    req.ifi.ifi_family = AF_UNSPEC;
    req.ifi.ifi_index = if_nametoindex(interface_.c_str());
    if (req.ifi.ifi_index == 0) {
        errno_ = ENODEV;
        ::close(sock);
        return -1;
    }

    switch (state) {
        case BusState::ACTIVE:
        case BusState::STOPPED:
            req.ifi.ifi_change = IFF_UP;
            req.ifi.ifi_flags = (state == BusState::ACTIVE) ? (IFF_UP | IFF_RUNNING) : 0;
            break;
        default:
            ::close(sock);
            return 0;
    }

    if (send(sock, &req, req.nlh.nlmsg_len, 0) < 0) {
        errno_ = errno;
        ::close(sock);
        return -1;
    }

    ::close(sock);

    state_.store(state);
    return 0;
}

int SocketCAN::get_bus_state(BusState& state) {
    // Read from interface flags
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);

    int sock = ::socket(PF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        errno_ = errno;
        return -1;
    }

    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
        errno_ = errno;
        ::close(sock);
        return -1;
    }

    ::close(sock);

    if (ifr.ifr_flags & IFF_RUNNING) {
        state = BusState::ACTIVE;
    } else if (ifr.ifr_flags & IFF_UP) {
        state = BusState::STOPPED;
    } else {
        state = BusState::UNKNOWN;
    }

    // Check for bus-off
    if (is_bus_off()) {
        state = BusState::BUS_OFF;
    } else if (is_error_passive()) {
        state = BusState::PASSIVE;
    }

    return 0;
}

int SocketCAN::get_stats(CANStats& stats) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats = stats_;
    stats.tx_error_counter = tx_error_counter_.load();
    stats.rx_error_counter = rx_error_counter_.load();
    stats.bus_state = state_.load();
    return 0;
}

int SocketCAN::get_restart_ms(uint32_t& ms) {
    int fd = sock_fd_.load();
    if (fd < 0) {
        errno_ = ENOTCONN;
        return -1;
    }

    // Auto-restart configuration not directly accessible
    // Return 0 (disabled) as default
    ms = 0;
    return 0;
}

int SocketCAN::set_restart_ms(uint32_t ms) {
    (void)ms;
    // Auto-restart is configured via Linux driver
    // This would typically be done via iproute2 or sysfs
    return 0;
}

int SocketCAN::restart() {
    int fd = sock_fd_.load();
    if (fd < 0) {
        errno_ = ENOTCONN;
        return -1;
    }

    // Get interface index
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        errno_ = errno;
        return -1;
    }

    // Use netlink to bring up interface
    return set_bus_state(BusState::ACTIVE);
}

int SocketCAN::get_error_mask() const {
    return 0;
}

void SocketCAN::start_monitoring(std::function<void(const CANErrorFrame&)> callback) {
    if (monitoring_.load()) return;

    error_callback_ = callback;
    monitoring_.store(true);

    monitor_thread_ = std::thread([this]() {
        int fd = err_fd_.load();
        if (fd < 0) {
            fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (fd < 0) return;

            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);

            if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
                ::close(fd);
                return;
            }

            struct sockaddr_can addr;
            memset(&addr, 0, sizeof(addr));
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
                ::close(fd);
                return;
            }

            err_fd_.store(fd);
        }

        can_frame frame;
        while (monitoring_.load()) {
            fd_set rdfds;
            FD_ZERO(&rdfds);
            FD_SET(fd, &rdfds);

            struct timespec tv = {0, 10000000};  // 10ms timeout

            int ret = pselect(fd + 1, &rdfds, nullptr, nullptr, &tv, nullptr);
            if (ret <= 0) continue;

            ssize_t n = ::read(fd, &frame, sizeof(frame));
            if (n > 0 && (frame.can_id & CAN_ERR_FLAG)) {
                CANErrorFrame err;
                err.error_code = frame.can_id;
                err.tx_error_counter = 0;
                err.rx_error_counter = 0;

                if (frame.can_id & CAN_ERR_BUSOFF) {
                    state_.store(BusState::BUS_OFF);
                    stats_.bus_errors++;
                }

                if (error_callback_) {
                    error_callback_(err);
                }
            }
        }
    });
}

void SocketCAN::stop_monitoring() {
    monitoring_.store(false);

    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }

    error_callback_ = nullptr;
}

bool SocketCAN::is_operational() const {
    return sock_fd_.load() >= 0 && state_.load() == BusState::ACTIVE;
}

bool SocketCAN::is_bus_off() const {
    if (tx_error_counter_.load() >= 128) {
        return true;
    }
    return state_.load() == BusState::BUS_OFF;
}

bool SocketCAN::is_error_passive() const {
    return tx_error_counter_.load() >= 128 || rx_error_counter_.load() >= 128;
}

void SocketCAN::reset_error_counters() {
    tx_error_counter_.store(0);
    rx_error_counter_.store(0);

    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.tx_errors = 0;
    stats_.rx_errors = 0;
}

void SocketCAN::update_error_counters() {
    // Error counters are updated from error frames
}

const char* SocketCAN::get_error_str() const {
    return strerror(errno_);
}

int SocketCAN::handle_netlink_error(const char* operation) {
    (void)operation;
    errno_ = errno;
    return -1;
}

} // namespace canopen
