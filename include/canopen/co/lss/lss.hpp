/**
 * @file lss.hpp
 * @brief LSS (Layer Setting Services) Master/Slave
 */

#ifndef CANOPEN_CO_LSS_LSS_HPP
#define CANOPEN_CO_LSS_LSS_HPP

#include <canopen/can/msg/message_factory.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>
#include <vector>

namespace canopen {

// Forward declaration
class BusInterface;

/**
 * @brief LSS State
 */
enum class LSSState : uint8_t {
    IDLE,
    WAITING_FOR_CONFIG,
    CONFIGURING,
    SWITCHED_TO_GLOBAL,
};

/**
 * @brief LSS Master
 */
class LSSMaster {
public:
    LSSMaster();

    void set_bus(BusInterface* bus) { bus_ = bus; }
    void attach(BusInterface& bus) {
        bus_ = &bus;
        route_ = bus.add_route(0x7E5, 0x7FF, [this](const CANFrame& f) {
            handle_frame(f);
        });
    }
    void detach(BusInterface& bus) {
        if (route_) bus.remove_route(route_);
        route_ = 0;
        bus_ = nullptr;
    }

    /**
     * @brief Switch specific node to config mode
     */
    int switch_to_config(uint8_t node_id);

    /**
     * @brief Switch all nodes to config mode (global)
     */
    int switch_to_config_global();

    /**
     * @brief Switch back to operational mode
     */
    int switch_to_operational();

    /**
     * @brief Set node ID
     */
    int set_node_id(uint8_t node_id);

    /**
     * @brief Set bit timing
     * @param bit_timing_index CiA 301 bit timing index
     */
    int set_bit_timing(uint8_t bit_timing_index);

    /**
     * @brief Store configuration
     */
    int store_configuration();

    /**
     * @brief Inquire LSS address
     */
    int inquire_lss_address(LSSAddress& address);

    /**
     * @brief Inquire node ID
     */
    int inquire_node_id(uint8_t& node_id);

    /**
     * @brief Handle LSS response
     */
    void handle_frame(const CANFrame& frame);

    /**
     * @brief Callback on node discovered
     */
    std::function<void(const LSSAddress& address, uint8_t node_id)> on_node_discovered;

    /**
     * @brief Callback on configuration complete
     */
    std::function<void(bool success)> on_config_complete;

private:
    void handle_switch_response(const CANFrame& frame);
    void handle_config_response(const CANFrame& frame);
    void handle_inquiry_response(const CANFrame& frame);

    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_{0};
    LSSState state_{LSSState::IDLE};
    uint8_t selected_node_{0};

    std::vector<LSSAddress> discovered_nodes_;
    std::vector<uint8_t> discovered_node_ids_;
};

/**
 * @brief LSS Slave
 */
class LSSSlave {
public:
    LSSSlave();

    void set_bus(BusInterface* bus) { bus_ = bus; }
    void attach(BusInterface& bus) {
        bus_ = &bus;
        route_ = bus.add_route(0x7E4, 0x7FF, [this](const CANFrame& f) {
            handle_frame(f);
        });
    }
    void detach(BusInterface& bus) {
        if (route_) bus.remove_route(route_);
        route_ = 0;
        bus_ = nullptr;
    }

    /**
     * @brief Set LSS address
     */
    void set_lss_address(const LSSAddress& address);

    /**
     * @brief Set current node ID
     */
    void set_node_id(uint8_t id);

    /**
     * @brief Get current node ID
     */
    uint8_t get_node_id() const { return node_id_; }

    /**
     * @brief Set current state
     */
    void set_state(LSSState state) { state_ = state; }

    /**
     * @brief Get current state
     */
    LSSState get_state() const { return state_; }

    /**
     * @brief Handle LSS message
     */
    void handle_frame(const CANFrame& frame);

    /**
     * @brief Callback on node ID change
     */
    std::function<void(uint8_t old_id, uint8_t new_id)> on_node_id_change;

    /**
     * @brief Callback on configuration mode change
     */
    std::function<void(bool config_mode)> on_mode_change;

private:
    void handle_switch_mode_selective(const CANFrame& frame);
    void handle_configure_command(const CANFrame& frame);
    void handle_store_command(const CANFrame& frame);
    void handle_inquiry_command(const CANFrame& frame);
    void handle_activate_bit_timing(const CANFrame& frame);

    void send_response(uint8_t specifier, const uint8_t* data = nullptr, size_t len = 0);

    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_{0};
    LSSAddress address_;
    uint8_t node_id_{127};
    LSSState state_{LSSState::IDLE};

    bool awaiting_activate_bit_timing_{false};
};

} // namespace canopen

#endif // CANOPEN_CO_LSS_LSS_HPP
