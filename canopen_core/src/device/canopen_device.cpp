/**
 * @file canopen_device.cpp
 * @brief CANopen Device / Master / Slave implementation
 */

#include <canopen/device/canopen_device.hpp>
#include <canopen/can/msg/message_factory.hpp>

namespace canopen {

// ==================== CANopenDevice ====================

CANopenDevice::CANopenDevice(uint8_t node_id)
    : node_id_(node_id), od_(node_id) {
}

void CANopenDevice::set_nmt_state(NMTState state) {
    if (nmt_state_ != state) {
        NMTState old = nmt_state_;
        nmt_state_ = state;
        on_nmt_state_change(old, state);
    }
}

// ==================== CANopenMaster ====================

CANopenMaster::CANopenMaster(uint8_t node_id, BusInterface* bus)
    : CANopenDevice(node_id), bus_(bus) {
}

void CANopenMaster::init() {
    if (!bus_) return;

    // SDO client for configuring slaves
    sdo_client_ = std::make_unique<SDOClient>(bus_);
    sdo_client_->set_timeout(200);
    sdo_client_->attach(*bus_);

    // NMT (commands + heartbeat consumer)
    nmt_ = std::make_unique<NMTService>(od_, bus_, node_id_);
    nmt_->attach(*bus_);

    // EMCY consumer
    emcy_consumer_ = std::make_unique<EMCYConsumer>();

    // LSS master
    lss_master_ = std::make_unique<LSSMaster>();
    lss_master_->attach(*bus_);
}

void CANopenMaster::start() {
    if (nmt_) {
        nmt_->start();
    }
    set_nmt_state(NMTState::OPERATIONAL);
}

void CANopenMaster::stop() {
    set_nmt_state(NMTState::STOPPED);
    if (nmt_) {
        nmt_->stop();
    }
}

void CANopenMaster::shutdown() {
    stop();
    sdo_client_.reset();
    nmt_.reset();
    emcy_consumer_.reset();
    lss_master_.reset();
    slaves_.clear();
}

void CANopenMaster::add_slave(uint8_t node_id, uint16_t heartbeat_time) {
    SlaveInfo info;
    info.heartbeat_time = heartbeat_time;
    slaves_[node_id] = info;

    if (nmt_) {
        nmt_->add_heartbeat_consumer(node_id, heartbeat_time);
    }
}

void CANopenMaster::remove_slave(uint8_t node_id) {
    slaves_.erase(node_id);
    if (nmt_) {
        nmt_->remove_heartbeat_consumer(node_id);
    }
}

void CANopenMaster::set_slave_dcf_path(uint8_t node_id, const std::string& dcf_path) {
    if (slaves_.find(node_id) == slaves_.end()) {
        add_slave(node_id, 1000);
    }
    slaves_[node_id].dcf_path = dcf_path;
}

bool CANopenMaster::boot_slave(uint8_t node_id, uint32_t timeout_ms) {
    if (!nmt_ || !sdo_client_) return false;
    if (slaves_.find(node_id) == slaves_.end()) {
        add_slave(node_id, 1000);
    }

    // 1. Reset communication (node sends bootup 0x700+id with state 0)
    nmt_->send_command(node_id, NMTCommand::RESET_COMMUNICATION);

    // 2. Wait for bootup
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    bool booted = false;
    while (std::chrono::steady_clock::now() < deadline) {
        if (nmt_->get_node_state(node_id).bootup_received) {
            booted = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!booted) return false;

    // 3. Configure from DCF if provided
    auto& info = slaves_[node_id];
    if (!info.dcf_path.empty()) {
        if (configure_slave_from_dcf(node_id, info.dcf_path) < 0) {
            return false;
        }
    }

    // 4. Start the node
    nmt_->send_command(node_id, NMTCommand::OPERATIONAL);
    info.configured = true;
    return true;
}

bool CANopenMaster::boot_all_slaves(uint32_t timeout_ms) {
    bool all = true;
    for (const auto& [node_id, info] : slaves_) {
        if (!boot_slave(node_id, timeout_ms)) {
            all = false;
        }
    }
    return all;
}

int CANopenMaster::configure_slave_from_dcf(uint8_t node_id, const std::string& dcf_path) {
    if (!sdo_client_) return -1;

    ObjectDictionary dcf;
    dcf.set_node_id(node_id);
    int rc = dcf.load_dcf(dcf_path);
    if (rc < 0) return rc;

    // Write all writable entries to the slave via SDO
    int errors = 0;
    for (const auto& entry : dcf) {
        if (entry.access != AccessType::RW && entry.access != AccessType::WO) {
            continue;
        }
        if (entry.value.empty()) continue;

        sdo_client_->set_server_node_id(node_id);
        SDOError err = sdo_client_->download_sync(
            entry.index, entry.subindex, entry.value.data(), entry.value.size());
        if (err != SDOError::OK) {
            errors++;
        }
    }
    return errors == 0 ? 0 : -1;
}

bool CANopenMaster::is_slave_operational(uint8_t node_id) const {
    auto it = slaves_.find(node_id);
    if (it == slaves_.end()) return false;

    if (nmt_) {
        const auto info = nmt_->get_node_state(node_id);
        if (info.state != NMTState::OPERATIONAL || info.heartbeat_timeout) {
            return false;
        }
    }
    return true;
}

void CANopenMaster::on_nmt_state_change(NMTState, NMTState new_state) {
    // Broadcast our state via heartbeat producer
    if (nmt_ && new_state != NMTState::INITIALISING) {
        // State propagated through heartbeat
    }
}

// ==================== CANopenSlave ====================

CANopenSlave::CANopenSlave(uint8_t node_id, BusInterface* bus)
    : CANopenDevice(node_id), bus_(bus) {
}

void CANopenSlave::init() {
    if (!bus_) return;

    // SDO server serving our OD
    sdo_server_ = std::make_unique<SDOServer>(od_, bus_);
    sdo_server_->set_node_id(node_id_);
    sdo_server_->attach(*bus_);

    // Heartbeat producer
    heartbeat_producer_ = std::make_unique<HeartbeatProducer>();
    heartbeat_producer_->set_node_id(node_id_);
    heartbeat_producer_->attach(*bus_);

    // EMCY service
    emcy_service_ = std::make_unique<EMCYService>(od_);
    emcy_service_->set_node_id(node_id_);
    emcy_service_->attach(*bus_);

    // Default 4 TPDO / 4 RPDO
    for (int i = 0; i < 4; i++) {
        tpdos_[i] = std::make_unique<TPDO>(od_, static_cast<uint8_t>(i + 1));
        rpdos_[i] = std::make_unique<RPDO>(od_, static_cast<uint8_t>(i + 1));
        rpdos_[i]->attach(*bus_);
    }
}

void CANopenSlave::start() {
    set_nmt_state(NMTState::OPERATIONAL);

    // Start heartbeat producer (interval from OD 0x1017 if set)
    uint16_t hb_ms = 1000;
    uint16_t od_hb = 0;
    size_t sz = sizeof(od_hb);
    if (od_.read(0x1017, 0x00, &od_hb, sz) == 0 && od_hb > 0) {
        hb_ms = od_hb;
    }
    heartbeat_producer_->set_state(NMTState::OPERATIONAL);
    heartbeat_producer_->start(hb_ms);
}

void CANopenSlave::stop() {
    set_nmt_state(NMTState::STOPPED);
    if (heartbeat_producer_) {
        heartbeat_producer_->set_state(NMTState::STOPPED);
    }
}

void CANopenSlave::shutdown() {
    stop();
    if (heartbeat_producer_) {
        heartbeat_producer_->stop();
    }
    for (int i = 0; i < 4; i++) {
        if (tpdos_[i]) tpdos_[i]->stop();
    }
    sdo_server_.reset();
    heartbeat_producer_.reset();
    emcy_service_.reset();
}

void CANopenSlave::send_pdo(uint8_t pdo_num) {
    if (pdo_num < 1 || pdo_num > 4) return;
    tpdos_[pdo_num - 1]->send();
}

void CANopenSlave::send_emcy(uint16_t error_code, uint8_t error_register,
                             const uint8_t* manufacturer) {
    if (emcy_service_) {
        emcy_service_->send(error_code, error_register, manufacturer);
    }
}

void CANopenSlave::set_heartbeat_producer_time(uint16_t time_ms) {
    if (heartbeat_producer_ && heartbeat_producer_->is_running()) {
        heartbeat_producer_->stop();
        heartbeat_producer_->start(time_ms);
    }
}

void CANopenSlave::set_identity(uint32_t vendor_id, uint32_t product_code,
                                uint32_t revision, uint32_t serial) {
    od_.write(0x1018, 0x01, &vendor_id, sizeof(vendor_id));
    od_.write(0x1018, 0x02, &product_code, sizeof(product_code));
    od_.write(0x1018, 0x03, &revision, sizeof(revision));
    od_.write(0x1018, 0x04, &serial, sizeof(serial));
}

void CANopenSlave::on_nmt_state_change(NMTState, NMTState new_state) {
    if (heartbeat_producer_) {
        heartbeat_producer_->set_state(new_state);
    }
}

} // namespace canopen
