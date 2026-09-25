/**
 * @file sync.cpp
 * @brief SYNC implementation
 */

#include <canopen/co/sync/sync.hpp>
#include <chrono>

namespace canopen {

SYNCService::SYNCService() = default;

void SYNCService::start_producer(uint16_t cycle_period_us) {
    if (producer_running_.load()) {
        stop_producer();
    }

    is_producer_ = true;
    cycle_period_us_ = cycle_period_us;
    producer_running_.store(true);

    producer_thread_ = std::thread(&SYNCService::producer_loop, this);
}

void SYNCService::stop_producer() {
    producer_running_.store(false);

    if (producer_thread_.joinable()) {
        producer_thread_.join();
    }
}

void SYNCService::start_consumer() {
    if (consumer_running_.load()) {
        stop_consumer();
    }

    is_consumer_ = true;
    consumer_running_.store(true);

    consumer_thread_ = std::thread(&SYNCService::consumer_loop, this);
}

void SYNCService::stop_consumer() {
    consumer_running_.store(false);

    if (consumer_thread_.joinable()) {
        consumer_thread_.join();
    }
}

void SYNCService::handle_frame(const CANFrame& frame) {
    if (!is_consumer_) return;

    uint8_t counter;
    if (!MessageFactory::parse_sync(frame, counter)) {
        return;
    }

    last_sync_time_ = std::chrono::steady_clock::now();
    counter_ = counter;
    sync_received_.store(true);

    // Check for counter discontinuity
    if (expected_counter_ != counter) {
        // Counter jumped - could indicate missed SYNC
        if (on_sync_lost) {
            on_sync_lost();
        }
    }

    expected_counter_ = (counter + 1) % 256;

    if (on_sync) {
        on_sync(counter);
    }
}

void SYNCService::set_consumer_timeout(uint32_t timeout_ms) {
    consumer_timeout_ms_ = timeout_ms;
}

void SYNCService::producer_loop() {
    while (producer_running_.load()) {
        send_sync();
        std::this_thread::sleep_for(std::chrono::microseconds(cycle_period_us_));
    }
}

void SYNCService::consumer_loop() {
    while (consumer_running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(consumer_timeout_ms_));

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_sync_time_).count();

        if (elapsed > consumer_timeout_ms_ && sync_received_.load()) {
            sync_received_.store(false);
            if (on_sync_lost) {
                on_sync_lost();
            }
        }
    }
}

void SYNCService::send_sync() {
    if (bus_) {
        CANFrame frame = MessageFactory::create_sync(counter_);
        bus_->send(frame);
    }

    counter_ = (counter_ + 1) % 256;
}

void SYNCService::handle_sync_frame(const CANFrame& frame) {
    handle_frame(frame);
}

} // namespace canopen
