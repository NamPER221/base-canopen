/**
 * @file event_loop.cpp
 * @brief Event Loop implementation
 */

#include <canopen/ev/event_loop.hpp>
#include <sys/epoll.h>
#include <unistd.h>
#include <cstring>

namespace canopen {

EventLoop::EventLoop() {
    epoll_fd_ = epoll_create1(0);
}

EventLoop::~EventLoop() {
    stop();
    if (epoll_fd_ >= 0) {
        close(epoll_fd_);
    }
}

EventLoop::TimerHandle EventLoop::add_timer(std::chrono::milliseconds interval,
                                          std::function<void()> callback,
                                          bool repeating) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);

    TimerHandle handle = next_timer_handle_++;
    TimerInfo info;
    info.interval = interval;
    info.next_time = std::chrono::steady_clock::now() + interval;
    info.callback = std::move(callback);
    info.repeating = repeating;

    timers_[handle] = info;
    return handle;
}

void EventLoop::cancel_timer(TimerHandle handle) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    timers_.erase(handle);
}

void EventLoop::cancel_all_timers() {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    timers_.clear();
}

EventLoop::WatchHandle EventLoop::add_watch(int fd, uint32_t events,
                                           std::function<void(uint32_t)> callback) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);

    WatchHandle handle = next_watch_handle_++;
    WatchInfo info;
    info.fd = fd;
    info.events = events;
    info.callback = std::move(callback);

    watches_[handle] = info;

    struct epoll_event ev;
    ev.events = events;
    ev.data.u64 = handle;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev);

    return handle;
}

void EventLoop::remove_watch(WatchHandle handle) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);

    auto it = watches_.find(handle);
    if (it != watches_.end()) {
        epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, it->second.fd, nullptr);
        watches_.erase(it);
    }
}

void EventLoop::post_delayed(std::chrono::milliseconds delay,
                            std::function<void()> callback) {
    add_timer(delay, [cb = std::move(callback)]() { cb(); }, false);
}

void EventLoop::run() {
    running_.store(true);

    while (running_.load()) {
        process_events(100);  // 100ms timeout
    }
}

void EventLoop::run_async() {
    loop_thread_ = std::thread(&EventLoop::run, this);
}

void EventLoop::stop() {
    running_.store(false);

    if (loop_thread_.joinable()) {
        loop_thread_.join();
    }
}

void EventLoop::post(std::function<void()> callback) {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    pending_callbacks_.push_back(std::move(callback));
}

void EventLoop::process_events(int timeout_ms) {
    // Process pending callbacks
    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        for (auto& cb : pending_callbacks_) {
            cb();
        }
        pending_callbacks_.clear();
    }

    // Process timers
    process_timers();

    // Process file descriptors
    process_watches(timeout_ms);
}

void EventLoop::process_timers() {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);

    auto now = std::chrono::steady_clock::now();

    for (auto it = timers_.begin(); it != timers_.end(); ) {
        if (now >= it->second.next_time) {
            if (it->second.repeating) {
                it->second.next_time = now + it->second.interval;
                it->second.callback();
                ++it;
            } else {
                it->second.callback();
                it = timers_.erase(it);
            }
        } else {
            ++it;
        }
    }
}

void EventLoop::process_watches(int timeout_ms) {
    struct epoll_event events[16];
    int nfds = epoll_wait(epoll_fd_, events, 16, timeout_ms);

    for (int i = 0; i < nfds; ++i) {
        WatchHandle handle = static_cast<WatchHandle>(events[i].data.u64);

        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        auto it = watches_.find(handle);
        if (it != watches_.end()) {
            it->second.callback(events[i].events);
        }
    }
}

} // namespace canopen
