/**
 * @file bus_monitor.cpp
 * @brief CAN Bus Monitor implementation
 */

#include <canopen/can/raw/bus_monitor.hpp>
#include <canopen/can/raw/socket_can.hpp>
#include <algorithm>
#include <cstring>
#include <unistd.h>

namespace canopen {

BusMonitor::BusMonitor() {
    clock_gettime(CLOCK_MONOTONIC, &start_time_);
}

BusMonitor::~BusMonitor() {
    stop();
}

void BusMonitor::start(const std::string& interface) {
    if (active_.load()) return;

    interface_ = interface;
    active_.store(true);
    recovering_.store(false);

    clock_gettime(CLOCK_MONOTONIC, &start_time_);

    monitor_thread_ = std::thread(&BusMonitor::monitoring_loop, this);
}

void BusMonitor::stop() {
    active_.store(false);

    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

BusMetrics BusMonitor::get_metrics() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return metrics_;
}

void BusMonitor::reset_metrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_ = BusMetrics{};
    clock_gettime(CLOCK_MONOTONIC, &start_time_);
}

void BusMonitor::set_error_callback(std::function<void(const CANErrorFrame&)> callback) {
    error_callback_ = std::move(callback);
}

void BusMonitor::set_state_change_callback(std::function<void(BusState, BusState)> callback) {
    state_change_callback_ = std::move(callback);
}

void BusMonitor::add_observer(BusObserver* observer) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    observers_.push_back(observer);
}

void BusMonitor::remove_observer(BusObserver* observer) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    auto it = std::remove(observers_.begin(), observers_.end(), observer);
    observers_.erase(it, observers_.end());
}

void BusMonitor::set_warning_threshold(uint8_t tx_threshold, uint8_t rx_threshold) {
    warning_tx_threshold_ = tx_threshold;
    warning_rx_threshold_ = rx_threshold;
}

void BusMonitor::set_error_passive_threshold(uint8_t threshold) {
    passive_threshold_ = threshold;
}

bool BusMonitor::is_error_warning() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return metrics_.tx_error_counter >= warning_tx_threshold_ ||
           metrics_.rx_error_counter >= warning_rx_threshold_;
}

bool BusMonitor::is_error_passive() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return metrics_.tx_error_counter >= passive_threshold_ ||
           metrics_.rx_error_counter >= passive_threshold_;
}

bool BusMonitor::is_bus_off() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return metrics_.state == BusState::BUS_OFF;
}

double BusMonitor::get_tx_rate() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    auto elapsed = get_uptime();
    if (elapsed.count() == 0) return 0;
    return static_cast<double>(metrics_.tx_frames) / elapsed.count();
}

double BusMonitor::get_rx_rate() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    auto elapsed = get_uptime();
    if (elapsed.count() == 0) return 0;
    return static_cast<double>(metrics_.rx_frames) / elapsed.count();
}

double BusMonitor::get_bus_load() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    uint64_t total_frames = metrics_.tx_frames + metrics_.rx_frames;
    auto elapsed = get_uptime();
    if (elapsed.count() == 0) return 0;

    double avg_frame_bytes = 16.0;  // Typical CAN frame
    double bits_per_second = (total_frames * avg_frame_bytes * 8.0) / elapsed.count();
    double bus_capacity = 500000.0;  // 500 kbps

    return std::min(100.0, (bits_per_second / bus_capacity) * 100.0);
}

std::chrono::seconds BusMonitor::get_uptime() const {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    int64_t elapsed_sec = now.tv_sec - start_time_.tv_sec;
    return std::chrono::seconds(elapsed_sec);
}

std::chrono::milliseconds BusMonitor::time_since_last_frame() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    struct timespec now, diff;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (now.tv_sec > metrics_.last_rx_time.tv_sec ||
        (now.tv_sec == metrics_.last_rx_time.tv_sec &&
         now.tv_nsec >= metrics_.last_rx_time.tv_nsec)) {
        diff.tv_sec = now.tv_sec - metrics_.last_rx_time.tv_sec;
        diff.tv_nsec = now.tv_nsec - metrics_.last_rx_time.tv_nsec;
        if (diff.tv_nsec < 0) {
            diff.tv_sec--;
            diff.tv_nsec += 1000000000;
        }
    } else {
        diff.tv_sec = 0;
        diff.tv_nsec = 0;
    }

    return std::chrono::milliseconds(diff.tv_sec * 1000 + diff.tv_nsec / 1000000);
}

void BusMonitor::monitoring_loop() {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        close(fd);
        return;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd);
        return;
    }

    can_frame frame;
    fd_set read_fds;

    while (active_.load()) {
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);

        struct timespec timeout = {0, 10000000};  // 10ms

        int ret = pselect(fd + 1, &read_fds, nullptr, nullptr, &timeout, nullptr);
        if (ret < 0) continue;
        if (ret == 0) continue;

        ssize_t n = read(fd, &frame, sizeof(frame));
        if (n <= 0) continue;

        if (frame.can_id & CAN_ERR_FLAG) {
            CANErrorFrame err;
            err.error_code = frame.can_id;
            err.tx_error_counter = 0;
            err.rx_error_counter = 0;

            if (frame.can_dlc >= 3) {
                err.tx_error_counter = frame.data[0];
                err.rx_error_counter = frame.data[1];
                err.error_type = frame.data[2];
            }

            update_metrics(err);
            notify_observers(err);

            if (error_callback_) {
                error_callback_(err);
            }

            if (recovering_.load() && err.error_code == CAN_ERR_RESTARTED) {
                recovering_.store(false);
            }
        }
    }

    close(fd);
}

void BusMonitor::update_metrics(const CANErrorFrame& error) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);

    metrics_.tx_error_counter = error.tx_error_counter;
    metrics_.rx_error_counter = error.rx_error_counter;
    clock_gettime(CLOCK_MONOTONIC, &metrics_.last_error_time);

    if (error.error_code & CAN_ERR_BUSOFF) {
        if (metrics_.state != BusState::BUS_OFF) {
            BusState old = metrics_.state;
            metrics_.state = BusState::BUS_OFF;
            notify_state_change(old, BusState::BUS_OFF);
            recovering_.store(true);
        }
    } else if (error.tx_error_counter >= passive_threshold_ ||
               error.rx_error_counter >= passive_threshold_) {
        if (metrics_.state != BusState::PASSIVE) {
            BusState old = metrics_.state;
            metrics_.state = BusState::PASSIVE;
            notify_state_change(old, BusState::PASSIVE);
        }
    } else if (error.tx_error_counter >= warning_tx_threshold_ ||
               error.rx_error_counter >= warning_rx_threshold_) {
        if (metrics_.state != BusState::WARNING) {
            BusState old = metrics_.state;
            metrics_.state = BusState::WARNING;
            notify_state_change(old, BusState::WARNING);
        }
    } else if (metrics_.state != BusState::ACTIVE) {
        BusState old = metrics_.state;
        metrics_.state = BusState::ACTIVE;
        notify_state_change(old, BusState::ACTIVE);
    }

    metrics_.bus_errors++;
    if (error.error_code & CAN_ERR_LOSTARB) {
        metrics_.arbitration_lost++;
    }
}

void BusMonitor::notify_observers(const CANErrorFrame& error) {
    std::lock_guard<std::mutex> lock(observers_mutex_);
    for (auto* obs : observers_) {
        obs->on_error(error);
    }
}

void BusMonitor::notify_state_change(BusState old_state, BusState new_state) {
    last_state_.store(new_state);

    if (state_change_callback_) {
        state_change_callback_(old_state, new_state);
    }

    std::lock_guard<std::mutex> lock(observers_mutex_);
    for (auto* obs : observers_) {
        obs->on_state_change(old_state, new_state);
    }
}

} // namespace canopen
