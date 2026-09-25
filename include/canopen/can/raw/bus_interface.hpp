/**
 * @file bus_interface.hpp
 * @brief Abstract CAN bus interface with COB-ID frame routing
 *
 * Every CANopen service (NMT, SDO, PDO, EMCY, ...) communicates with the
 * outside world exclusively through this interface, making the protocol
 * stack independent from the underlying CAN driver (SocketCAN, fake bus
 * for unit tests, etc.).
 */

#ifndef CANOPEN_CAN_RAW_BUS_INTERFACE_HPP
#define CANOPEN_CAN_RAW_BUS_INTERFACE_HPP

#include <canopen/can/frame/frame.hpp>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace canopen {

/**
 * @brief Abstract CAN bus interface
 *
 * - `send()` transmits a frame onto the bus.
 * - `dispatch()` routes a received frame to all registered handlers whose
 *   (can_id & mask) pattern matches. Implementations call this from their
 *   RX thread; tests call it directly to simulate incoming frames.
 */
class BusInterface {
public:
    using Handler = std::function<void(const CANFrame&)>;
    using RouteHandle = int;

    virtual ~BusInterface() = default;

    // Non-copyable
    BusInterface(const BusInterface&) = delete;
    BusInterface& operator=(const BusInterface&) = delete;

    /**
     * @brief Transmit a frame onto the bus
     * @return true if the frame was queued/written successfully
     */
    virtual bool send(const CANFrame& frame) = 0;

    /**
     * @brief Check if the bus is up and usable
     */
    virtual bool is_up() const = 0;

    // ==================== Frame Routing ====================

    /**
     * @brief Register a handler for frames matching (can_id & mask)
     * @return Route handle (use with remove_route), or 0 on error
     */
    RouteHandle add_route(uint32_t can_id, uint32_t mask, Handler handler) {
        if (!handler) return 0;
        std::lock_guard<std::mutex> lock(routes_mutex_);
        auto route = std::make_shared<Route>();
        route->handle = next_handle_;
        route->can_id = can_id;
        route->mask = mask;
        route->handler = std::move(handler);
        routes_.push_back(std::move(route));
        return next_handle_++;
    }

    /**
     * @brief Register a handler receiving every frame
     *
     * SocketCAN filter semantics: frame matches when
     * (frame.can_id & mask) == (filter.can_id & mask). Mask 0 therefore
     * matches EVERY frame.
     */
    RouteHandle add_route_all(Handler handler) {
        return add_route(0x000, 0x000, std::move(handler));
    }

    /**
     * @brief Remove a previously registered route
     */
    void remove_route(RouteHandle handle) {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        routes_.erase(
            std::remove_if(routes_.begin(), routes_.end(),
                           [handle](const auto& r) { return r->handle == handle; }),
            routes_.end());
    }

    /**
     * @brief Remove all routes
     */
    void clear_routes() {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        routes_.clear();
    }

    /**
     * @brief Number of active routes
     */
    size_t route_count() const {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        return routes_.size();
    }

    /**
     * @brief Dispatch a frame to all matching handlers
     *
     * Called by the bus implementation's RX thread. Safe to call from
     * anywhere (tests call it to inject frames).
     */
    void dispatch(const CANFrame& frame) {
        // Snapshot routes so handlers may add/remove routes freely
        std::vector<std::shared_ptr<Route>> snapshot;
        {
            std::lock_guard<std::mutex> lock(routes_mutex_);
            snapshot = routes_;
        }

        const uint32_t cob_id = frame.id() & 0x7FF;
        for (const auto& route : snapshot) {
            if ((cob_id & route->mask) == (route->can_id & route->mask)) {
                route->handler(frame);
            }
        }
    }

protected:
    BusInterface() = default;

    struct Route {
        RouteHandle handle{0};
        uint32_t can_id{0};
        uint32_t mask{0};
        Handler handler;
    };

    std::vector<std::shared_ptr<Route>> routes_;
    mutable std::mutex routes_mutex_;
    RouteHandle next_handle_{1};
};

// ==================== Convenience Route Helpers ====================

namespace routes {

inline BusInterface::RouteHandle nmt(BusInterface& bus, BusInterface::Handler h) {
    return bus.add_route(0x000, 0x7FF, std::move(h));
}

inline BusInterface::RouteHandle sync(BusInterface& bus, BusInterface::Handler h) {
    return bus.add_route(0x080, 0x7FF, std::move(h));
}

inline BusInterface::RouteHandle heartbeat(BusInterface& bus, uint8_t node_id, BusInterface::Handler h) {
    return bus.add_route(0x700 + node_id, 0x7FF, std::move(h));
}

inline BusInterface::RouteHandle heartbeat_all(BusInterface& bus, BusInterface::Handler h) {
    return bus.add_route(0x700, 0x780, std::move(h));
}

inline BusInterface::RouteHandle emcy(BusInterface& bus, uint8_t node_id, BusInterface::Handler h) {
    return bus.add_route(0x080 + node_id, 0x7FF, std::move(h));
}

inline BusInterface::RouteHandle emcy_all(BusInterface& bus, BusInterface::Handler h) {
    // EMCY occupies 0x081-0x0FF; sync is 0x080
    return bus.add_route(0x081, 0x780, std::move(h));
}

inline BusInterface::RouteHandle tsdo(BusInterface& bus, uint8_t node_id, BusInterface::Handler h) {
    return bus.add_route(0x580 + node_id, 0x7FF, std::move(h));
}

inline BusInterface::RouteHandle rsdo(BusInterface& bus, uint8_t node_id, BusInterface::Handler h) {
    return bus.add_route(0x600 + node_id, 0x7FF, std::move(h));
}

inline BusInterface::RouteHandle tpdo(BusInterface& bus, uint8_t node_id, uint8_t num, BusInterface::Handler h) {
    static constexpr uint32_t bases[4] = {0x180, 0x280, 0x380, 0x480};
    return bus.add_route(bases[(num - 1) & 0x03] + (node_id & 0x7F), 0x7FF, std::move(h));
}

inline BusInterface::RouteHandle rpdo(BusInterface& bus, uint8_t node_id, uint8_t num, BusInterface::Handler h) {
    static constexpr uint32_t bases[4] = {0x200, 0x300, 0x400, 0x500};
    return bus.add_route(bases[(num - 1) & 0x03] + (node_id & 0x7F), 0x7FF, std::move(h));
}

} // namespace routes

} // namespace canopen

#endif // CANOPEN_CAN_RAW_BUS_INTERFACE_HPP
