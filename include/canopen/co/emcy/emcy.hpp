/**
 * @file emcy.hpp
 * @brief EMCY (Emergency) Protocol
 */

#ifndef CANOPEN_CO_EMCY_EMCY_HPP
#define CANOPEN_CO_EMCY_EMCY_HPP

#include <canopen/can/msg/message_factory.hpp>
#include <canopen/co/object_dictionary/object_dictionary.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <array>
#include <functional>

namespace canopen {

// Forward declaration
class BusInterface;

/**
 * @brief Emergency error structure
 */
struct EmergencyError {
    uint16_t error_code;
    uint8_t error_register;
    uint8_t manufacturer_specific[5];
};

/**
 * @brief EMCY Service
 */
class EMCYService {
public:
    /**
     * @brief Construct EMCY Service
     */
    EMCYService(ObjectDictionary& od);

    /**
     * @brief Set bus interface
     */
    void set_bus(BusInterface* bus) { bus_ = bus; }

    /**
     * @brief Register for automatic dispatch of EMCY frames on this bus
     */
    void attach(BusInterface& bus) {
        bus_ = &bus;
        route_ = routes::emcy_all(bus, [this](const CANFrame& f) {
            handle_frame(f);
        });
    }

    /**
     * @brief Unregister from the bus
     */
    void detach(BusInterface& bus) {
        if (route_) bus.remove_route(route_);
        route_ = 0;
        bus_ = nullptr;
    }

    /**
     * @brief Set node ID
     */
    void set_node_id(uint8_t id);

    /**
     * @brief Handle received EMCY frame
     */
    void handle_frame(const CANFrame& frame);

    /**
     * @brief Send emergency message
     */
    void send(uint16_t error_code, uint8_t error_register,
              const uint8_t* manufacturer = nullptr);

    /**
     * @brief Clear error register
     */
    void clear_error_register();

    /**
     * @brief Get error history
     */
    const std::array<EmergencyError, 8>& get_error_history() const { return error_history_; }

    /**
     * @brief Get current error register
     */
    uint8_t get_error_register() const { return error_register_; }

    /**
     * @brief Callback on emergency received
     */
    std::function<void(uint8_t node_id, const EmergencyError& error)> on_emergency;

    /**
     * @brief Callback on error register change
     */
    std::function<void(uint8_t register_bits)> on_error_register_change;

private:
    void update_error_register();
    void add_to_history(const EmergencyError& error);

    ObjectDictionary& od_;
    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_{0};
    uint8_t node_id_{0};

    uint8_t error_register_{0};
    uint8_t error_history_index_{0};
    std::array<EmergencyError, 8> error_history_{};
};

/**
 * @brief EMCY Consumer
 */
class EMCYConsumer {
public:
    EMCYConsumer();

    /**
     * @brief Add producer to monitor
     */
    void add_producer(uint8_t node_id);

    /**
     * @brief Remove producer
     */
    void remove_producer(uint8_t node_id);

    /**
     * @brief Handle frame
     */
    void handle_frame(const CANFrame& frame);

    /**
     * @brief Get error for node
     */
    const EmergencyError* get_error(uint8_t node_id) const;

    /**
     * @brief Clear error for node
     */
    void clear_error(uint8_t node_id);

    /**
     * @brief Callback on emergency
     */
    std::function<void(uint8_t node_id, const EmergencyError& error)> on_emergency;

private:
    struct ProducerInfo {
        EmergencyError last_error;
        bool has_error{false};
    };

    std::map<uint8_t, ProducerInfo> producers_;
};

} // namespace canopen

#endif // CANOPEN_CO_EMCY_EMCY_HPP
