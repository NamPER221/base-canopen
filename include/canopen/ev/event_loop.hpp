/**
 * @file event_loop.hpp
 * @brief Event Loop for async operations
 */

#ifndef CANOPEN_EV_EVENT_LOOP_HPP
#define CANOPEN_EV_EVENT_LOOP_HPP

#include <functional>
#include <map>
#include <atomic>
#include <thread>
#include <chrono>
#include <optional>

namespace canopen {

/**
 * @brief Event Loop
 */
class EventLoop {
public:
    EventLoop();
    ~EventLoop();

    // ==================== Timer ====================

    using TimerHandle = uint64_t;

    /**
     * @brief Add timer
     * @param interval Timer interval
     * @param callback Callback function
     * @param repeating Repeating timer
     * @return Timer handle
     */
    TimerHandle add_timer(std::chrono::milliseconds interval,
                         std::function<void()> callback,
                         bool repeating = true);

    /**
     * @brief Cancel timer
     */
    void cancel_timer(TimerHandle handle);

    /**
     * @brief Cancel all timers
     */
    void cancel_all_timers();

    // ==================== File Descriptor Monitoring ====================

    using WatchHandle = uint64_t;

    /**
     * @brief Watch events
     */
    enum class WatchEvent : uint32_t {
        READ = 1,
        WRITE = 2,
        ERROR = 4,
    };

    /**
     * @brief Add file descriptor watch
     * @param fd File descriptor
     * @param events Events to watch
     * @param callback Callback function
     * @return Watch handle
     */
    WatchHandle add_watch(int fd, uint32_t events,
                         std::function<void(uint32_t)> callback);

    /**
     * @brief Remove file descriptor watch
     */
    void remove_watch(WatchHandle handle);

    // ==================== Delayed Execution ====================

    /**
     * @brief Post delayed callback
     */
    void post_delayed(std::chrono::milliseconds delay,
                     std::function<void()> callback);

    // ==================== Execution Control ====================

    /**
     * @brief Run event loop
     */
    void run();

    /**
     * @brief Run event loop in separate thread
     */
    void run_async();

    /**
     * @brief Stop event loop
     */
    void stop();

    /**
     * @brief Check if running
     */
    bool is_running() const { return running_.load(); }

    // ==================== Post ====================

    /**
     * @brief Post callback to event loop
     */
    void post(std::function<void()> callback);

private:
    void process_events(int timeout_ms);
    void process_timers();
    void process_watches(int timeout_ms);

    int epoll_fd_{-1};
    std::atomic<bool> running_{false};
    std::thread loop_thread_;

    struct TimerInfo {
        std::chrono::milliseconds interval;
        std::chrono::steady_clock::time_point next_time;
        std::function<void()> callback;
        bool repeating;
    };

    std::map<TimerHandle, TimerInfo> timers_;
    TimerHandle next_timer_handle_{1};

    struct WatchInfo {
        int fd;
        uint32_t events;
        std::function<void(uint32_t)> callback;
    };

    std::map<WatchHandle, WatchInfo> watches_;
    WatchHandle next_watch_handle_{1};

    std::mutex callbacks_mutex_;
    std::vector<std::function<void()>> pending_callbacks_;
};

} // namespace canopen

#endif // CANOPEN_EV_EVENT_LOOP_HPP
