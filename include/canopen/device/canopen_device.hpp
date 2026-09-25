/**
 * @file canopen_device.hpp
 * @brief Base CANopen Device classes (Device / Master / Slave)
 */

#ifndef CANOPEN_DEVICE_CANOPEN_DEVICE_HPP
#define CANOPEN_DEVICE_CANOPEN_DEVICE_HPP

#include <canopen/co/object_dictionary/object_dictionary.hpp>
#include <canopen/co/nmt/nmt.hpp>
#include <canopen/co/sdo/sdo.hpp>
#include <canopen/co/pdo/pdo.hpp>
#include <canopen/co/emcy/emcy.hpp>
#include <canopen/co/heartbeat/heartbeat.hpp>
#include <canopen/co/lss/lss.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace canopen {

/**
 * @brief Base CANopen Device
 */
class CANopenDevice {
public:
    explicit CANopenDevice(uint8_t node_id);
    virtual ~CANopenDevice() = default;

    // Non-copyable
    CANopenDevice(const CANopenDevice&) = delete;
    CANopenDevice& operator=(const CANopenDevice&) = delete;

    // ==================== Accessors ====================

    uint8_t get_node_id() const { return node_id_; }
    ObjectDictionary& get_od() { return od_; }
    const ObjectDictionary& get_od() const { return od_; }
    NMTState get_nmt_state() const { return nmt_state_; }

    // ==================== Lifecycle ====================

    virtual void init() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void shutdown() = 0;

protected:
    void set_nmt_state(NMTState state);
    virtual void on_nmt_state_change(NMTState, NMTState) {}
    virtual void on_od_value_change(uint16_t, uint8_t) {}

    uint8_t node_id_;
    ObjectDictionary od_;
    NMTState nmt_state_{NMTState::INITIALISING};
};

/**
 * @brief CANopen Master — manages slaves over the bus
 */
class CANopenMaster : public CANopenDevice {
public:
    /**
     * @param node_id Master node ID (typically 1..127 or 0)
     * @param bus Bus interface (SocketCanBus, fake bus, ...)
     */
    CANopenMaster(uint8_t node_id, BusInterface* bus);

    // From CANopenDevice
    void init() override;
    void start() override;
    void stop() override;
    void shutdown() override;

    // ==================== Node Management ====================

    void add_slave(uint8_t node_id, uint16_t heartbeat_time);
    void remove_slave(uint8_t node_id);

    /**
     * @brief Attach a DCF/EDS file so boot_slave() configures the slave
     *        automatically from the file (writes all RW/WO entries via SDO)
     */
    void set_slave_dcf_path(uint8_t node_id, const std::string& dcf_path);

    bool boot_slave(uint8_t node_id, uint32_t timeout_ms = 3000);
    bool boot_all_slaves(uint32_t timeout_ms = 3000);

    /**
     * @brief Configure slave objects from a DCF file via SDO
     */
    int configure_slave_from_dcf(uint8_t node_id, const std::string& dcf_path);

    // ==================== Accessors ====================

    SDOClient& get_sdo_client() { return *sdo_client_; }
    NMTService& get_nmt() { return *nmt_; }
    LSSMaster& get_lss_master() { return *lss_master_; }
    EMCYConsumer& get_emcy_consumer() { return *emcy_consumer_; }

    /**
     * @brief Check if slave is operational (state + heartbeat)
     */
    bool is_slave_operational(uint8_t node_id) const;

protected:
    void on_nmt_state_change(NMTState old_state, NMTState new_state) override;

private:
    BusInterface* bus_;
    std::unique_ptr<SDOClient> sdo_client_;
    std::unique_ptr<NMTService> nmt_;
    std::unique_ptr<EMCYConsumer> emcy_consumer_;
    std::unique_ptr<LSSMaster> lss_master_;

    struct SlaveInfo {
        uint16_t heartbeat_time;
        NMTState expected_state{NMTState::OPERATIONAL};
        bool configured{false};
        std::string dcf_path;
    };
    std::map<uint8_t, SlaveInfo> slaves_;
};

/**
 * @brief CANopen Slave — serves SDO, produces heartbeat, sends PDO/EMCY
 */
class CANopenSlave : public CANopenDevice {
public:
    CANopenSlave(uint8_t node_id, BusInterface* bus);

    // From CANopenDevice
    void init() override;
    void start() override;
    void stop() override;
    void shutdown() override;

    // ==================== Send Data ====================

    void send_pdo(uint8_t pdo_num);
    void send_emcy(uint16_t error_code, uint8_t error_register,
                   const uint8_t* manufacturer = nullptr);

    // ==================== Configuration ====================

    void set_heartbeat_producer_time(uint16_t time_ms);
    void set_identity(uint32_t vendor_id, uint32_t product_code,
                      uint32_t revision, uint32_t serial);
    TPDO& get_tpdo(uint8_t num) { return *tpdos_[num - 1]; }
    RPDO& get_rpdo(uint8_t num) { return *rpdos_[num - 1]; }
    SDOServer& get_sdo_server() { return *sdo_server_; }
    EMCYService& get_emcy_service() { return *emcy_service_; }

protected:
    void on_nmt_state_change(NMTState old_state, NMTState new_state) override;

private:
    BusInterface* bus_;
    std::unique_ptr<SDOServer> sdo_server_;
    std::unique_ptr<HeartbeatProducer> heartbeat_producer_;
    std::unique_ptr<EMCYService> emcy_service_;

    std::unique_ptr<TPDO> tpdos_[4];
    std::unique_ptr<RPDO> rpdos_[4];
};

} // namespace canopen

#endif // CANOPEN_DEVICE_CANOPEN_DEVICE_HPP
