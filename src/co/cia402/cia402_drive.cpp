/**
 * @file cia402_drive.cpp
 * @brief CiA 402 Drive implementation (generic, SDO-based)
 */

#include <canopen/co/cia402/cia402_drive.hpp>
#include <cstring>
#include <algorithm>

namespace canopen {

// CiA 402 Controlword bits
namespace ControlwordBits {
    constexpr uint16_t SWITCH_ON = 0x0001;
    constexpr uint16_t ENABLE_VOLTAGE = 0x0002;
    constexpr uint16_t QUICK_STOP = 0x0004;
    constexpr uint16_t ENABLE_OPERATION = 0x0008;
    constexpr uint16_t NEW_SETPOINT = 0x0010;
    constexpr uint16_t HOMING_OPERATION_START = 0x0010;
    constexpr uint16_t CHANGE_SET_IMMEDIATELY = 0x0020;
    constexpr uint16_t FAULT_RESET = 0x0080;
    constexpr uint16_t HALT = 0x0100;
}

// CiA 402 Statusword bits
namespace StatuswordBits {
    constexpr uint16_t READY_TO_SWITCH_ON = 0x0001;
    constexpr uint16_t SWITCHED_ON = 0x0002;
    constexpr uint16_t OPERATION_ENABLED = 0x0004;
    constexpr uint16_t FAULT = 0x0008;
    constexpr uint16_t VOLTAGE_ENABLED = 0x0010;
    constexpr uint16_t QUICK_STOP = 0x0020;
    constexpr uint16_t SWITCH_ON_DISABLED = 0x0040;
    constexpr uint16_t WARNING = 0x0080;
    constexpr uint16_t TARGET_REACHED = 0x0400;
    constexpr uint16_t INTERNAL_LIMIT_ACTIVE = 0x0800;
    constexpr uint16_t SETPOINT_ACKNOWLEDGE = 0x1000;
    constexpr uint16_t HOMING_COMPLETE = 0x2000;
    constexpr uint16_t HOMING_ERROR = 0x4000;
}

// ==================== CiA 402 Drive ====================

CiA402Drive::CiA402Drive(uint8_t node_id, BusInterface* bus)
    : node_id_(node_id), bus_(bus) {
    if (bus_) {
        sdo_ = std::make_unique<SDOClient>(bus_, node_id_);
        sdo_->set_timeout(sdo_timeout_ms_);
        sdo_->set_verbose(verbose_);
        sdo_->attach(*bus_);
    }
}

CiA402Drive::~CiA402Drive() {
    stop();
}

void CiA402Drive::set_bus(BusInterface* bus) {
    bus_ = bus;
    if (bus_) {
        sdo_ = std::make_unique<SDOClient>(bus_, node_id_);
        sdo_->set_timeout(sdo_timeout_ms_);
        sdo_->set_verbose(verbose_);
        sdo_->attach(*bus_);
    } else {
        sdo_.reset();
    }
}

void CiA402Drive::attach(BusInterface& bus) {
    bus_ = &bus;
    if (!sdo_) {
        sdo_ = std::make_unique<SDOClient>(bus_, node_id_);
        sdo_->set_timeout(sdo_timeout_ms_);
    }
    sdo_->attach(bus);
}

void CiA402Drive::detach(BusInterface& bus) {
    if (sdo_) sdo_->detach(bus);
    bus_ = nullptr;
}

void CiA402Drive::init() {
    state_.store(CiA402State::SWITCH_ON_DISABLED);
    controlword_.store(0);
    statusword_.store(StatuswordBits::SWITCH_ON_DISABLED);
}

void CiA402Drive::start() {
    if (running_.load()) return;

    running_.store(true);

    // Start motion monitoring thread
    motion_monitor_running_.store(true);
    motion_monitor_thread_ = std::thread(&CiA402Drive::monitor_motion_loop, this);
}

void CiA402Drive::stop() {
    running_.store(false);
    motion_monitor_running_.store(false);

    if (motion_monitor_thread_.joinable()) {
        motion_monitor_thread_.join();
    }
}

void CiA402Drive::shutdown() {
    stop();
    state_.store(CiA402State::SWITCH_ON_DISABLED);
}

// ==================== Control ====================

void CiA402Drive::set_controlword(uint16_t cw) {
    controlword_.store(cw);
    send_controlword();
}

bool CiA402Drive::set_state(CiA402State target) {
    uint16_t cw = controlword_.load();
    CiA402State current = state_.load();

    // State transitions according to CiA 402
    switch (current) {
        case CiA402State::SWITCH_ON_DISABLED:
            if (target == CiA402State::READY_TO_SWITCH_ON) {
                cw |= ControlwordBits::ENABLE_VOLTAGE;
                cw &= ~ControlwordBits::QUICK_STOP;
            }
            break;

        case CiA402State::READY_TO_SWITCH_ON:
            if (target == CiA402State::SWITCHED_ON) {
                cw |= ControlwordBits::SWITCH_ON;
            } else if (target == CiA402State::SWITCH_ON_DISABLED) {
                cw &= ~ControlwordBits::ENABLE_VOLTAGE;
            }
            break;

        case CiA402State::SWITCHED_ON:
            if (target == CiA402State::OPERATION_ENABLED) {
                cw |= ControlwordBits::ENABLE_OPERATION;
            } else if (target == CiA402State::READY_TO_SWITCH_ON) {
                cw &= ~ControlwordBits::SWITCH_ON;
            }
            break;

        case CiA402State::OPERATION_ENABLED:
            if (target == CiA402State::SWITCHED_ON) {
                cw &= ~ControlwordBits::ENABLE_OPERATION;
            } else if (target == CiA402State::QUICK_STOP_ACTIVE) {
                cw |= ControlwordBits::QUICK_STOP;
            }
            break;

        case CiA402State::QUICK_STOP_ACTIVE:
            if (target == CiA402State::OPERATION_ENABLED) {
                cw &= ~ControlwordBits::QUICK_STOP;
                cw |= ControlwordBits::ENABLE_OPERATION;
            }
            break;

        case CiA402State::FAULT:
            if (target == CiA402State::SWITCH_ON_DISABLED) {
                cw |= ControlwordBits::FAULT_RESET;
            }
            break;

        default:
            return false;
    }

    controlword_.store(cw);
    send_controlword();

    // Refresh state from the drive
    poll_status();
    return true;
}

void CiA402Drive::quick_stop() {
    uint16_t cw = controlword_.load();
    cw |= ControlwordBits::QUICK_STOP;
    cw &= ~ControlwordBits::ENABLE_OPERATION;
    controlword_.store(cw);
    send_controlword();
}

void CiA402Drive::fault_reset() {
    uint16_t cw = controlword_.load();
    cw |= ControlwordBits::FAULT_RESET;
    controlword_.store(cw);
    send_controlword();

    // Clear fault reset bit after sending
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    cw &= ~ControlwordBits::FAULT_RESET;
    controlword_.store(cw);
    send_controlword();

    if (on_fault_reset) {
        on_fault_reset();
    }
}

// ==================== Operation Mode ====================

bool CiA402Drive::set_operation_mode(OperationMode mode) {
    current_mode_.store(mode);
    return sdo_write_i8(cia402_od::MODES_OF_OPERATION, 0x00,
                        static_cast<int8_t>(mode));
}

// ==================== Position Control ====================

void CiA402Drive::set_target_position(int32_t position) {
    target_position_.store(position);
    last_motion_command_ = std::chrono::steady_clock::now();
    sdo_write_i32(cia402_od::TARGET_POSITION, 0x00, position);
}

void CiA402Drive::set_position_profile(uint32_t max_speed, uint32_t acceleration,
                                       uint32_t deceleration) {
    sdo_write_u32(cia402_od::PROFILE_VELOCITY, 0x00, max_speed);
    sdo_write_u32(cia402_od::PROFILE_ACCELERATION, 0x00, acceleration);
    sdo_write_u32(cia402_od::PROFILE_DECELERATION, 0x00, deceleration);
}

void CiA402Drive::start_position_move(int32_t position, bool absolute) {
    set_target_position(position);

    uint16_t cw = controlword_.load();
    cw |= ControlwordBits::NEW_SETPOINT;
    if (!absolute) {
        cw |= ControlwordBits::CHANGE_SET_IMMEDIATELY;
    }
    set_controlword(cw);

    // Clear new setpoint bit
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    cw &= ~ControlwordBits::NEW_SETPOINT;
    set_controlword(cw);
}

void CiA402Drive::start_relative_move(int32_t offset) {
    start_position_move(offset, false);
}

void CiA402Drive::halt_position() {
    uint16_t cw = controlword_.load();
    cw |= ControlwordBits::HALT;
    set_controlword(cw);
}

void CiA402Drive::immediate_position(int32_t position) {
    set_target_position(position);

    uint16_t cw = controlword_.load();
    cw |= ControlwordBits::NEW_SETPOINT | ControlwordBits::CHANGE_SET_IMMEDIATELY;
    set_controlword(cw);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    cw &= ~ControlwordBits::NEW_SETPOINT;
    set_controlword(cw);
}

// ==================== Velocity Control ====================

void CiA402Drive::set_target_velocity(int32_t velocity) {
    target_velocity_.store(velocity);
    last_motion_command_ = std::chrono::steady_clock::now();
    sdo_write_i32(cia402_od::TARGET_VELOCITY, 0x00, velocity);
}

void CiA402Drive::set_velocity_profile(uint32_t max_acceleration,
                                       uint32_t max_deceleration) {
    sdo_write_u32(cia402_od::PROFILE_ACCELERATION, 0x00, max_acceleration);
    sdo_write_u32(cia402_od::PROFILE_DECELERATION, 0x00, max_deceleration);
}

void CiA402Drive::start_velocity_move(int32_t velocity) {
    set_operation_mode(OperationMode::VELOCITY);
    set_target_velocity(velocity);
}

void CiA402Drive::halt_velocity() {
    uint16_t cw = controlword_.load();
    cw |= ControlwordBits::HALT;
    set_controlword(cw);
}

// ==================== Torque Control ====================

void CiA402Drive::set_target_torque(int16_t torque) {
    target_torque_.store(torque);
    last_motion_command_ = std::chrono::steady_clock::now();
    sdo_write_i16(cia402_od::TARGET_TORQUE, 0x00, torque);
}

void CiA402Drive::set_torque_slope(uint16_t slope) {
    (void)slope;  // Manufacturer-specific (e.g., 0x2086) — override in vendor driver
}

// ==================== Homing ====================

void CiA402Drive::start_homing(int16_t method) {
    sdo_write_i16(cia402_od::HOMING_METHOD, 0x00, method);
    set_operation_mode(OperationMode::HOMING);

    uint16_t cw = controlword_.load();
    cw |= ControlwordBits::HOMING_OPERATION_START;
    set_controlword(cw);
}

bool CiA402Drive::is_homing() const {
    return current_mode_.load() == OperationMode::HOMING;
}

// ==================== Cyclic Sync Modes ====================

void CiA402Drive::set_target_position_cyclic(int32_t position) {
    target_position_.store(position);
    last_motion_command_ = std::chrono::steady_clock::now();
    sdo_write_i32(cia402_od::TARGET_POSITION, 0x00, position);
}

void CiA402Drive::set_target_velocity_cyclic(int32_t velocity) {
    target_velocity_.store(velocity);
    last_motion_command_ = std::chrono::steady_clock::now();
    sdo_write_i32(cia402_od::TARGET_VELOCITY, 0x00, velocity);
}

void CiA402Drive::set_target_torque_cyclic(int16_t torque) {
    target_torque_.store(torque);
    last_motion_command_ = std::chrono::steady_clock::now();
    sdo_write_i16(cia402_od::TARGET_TORQUE, 0x00, torque);
}

// ==================== Interpolation ====================

void CiA402Drive::set_interpolation_time_period(uint16_t period_us, uint8_t index) {
    // 0x60C2: interpolation time period (units of 10^-6 s)
    sdo_write_i8(cia402_od::SOFTWARE_LIMIT_MIN, 0x00, 0);  // placeholder guard
    (void)period_us;
    (void)index;
}

void CiA402Drive::set_interpolation_data_record(int32_t position, int16_t velocity,
                                                int16_t torque) {
    (void)position;
    (void)velocity;
    (void)torque;
}

// ==================== Safety ====================

void CiA402Drive::set_motion_timeout(uint32_t timeout_ms) {
    motion_timeout_ms_.store(timeout_ms);
    if (timeout_ms > 0) {
        motion_timeout_enabled_.store(true);
    }
}

void CiA402Drive::enable_motion_timeout(bool enable) {
    motion_timeout_enabled_.store(enable);
}

bool CiA402Drive::check_motion_timeout() {
    if (!motion_timeout_enabled_.load()) return false;
    if (motion_timeout_ms_.load() == 0) return false;

    auto elapsed = std::chrono::steady_clock::now() - last_motion_command_;
    auto timeout = std::chrono::milliseconds(motion_timeout_ms_.load());

    if (elapsed > timeout) {
        if (!motion_timeout_occurred_.load()) {
            motion_timeout_occurred_.store(true);
            if (on_motion_timeout) {
                on_motion_timeout();
            }
        }
        return true;
    }
    return false;
}

void CiA402Drive::reset_motion_timeout() {
    motion_timeout_occurred_.store(false);
    last_motion_command_ = std::chrono::steady_clock::now();
}

// ==================== Limits ====================

void CiA402Drive::set_software_position_limits(int32_t min, int32_t max) {
    sdo_write_i32(cia402_od::SOFTWARE_LIMIT_MIN, 0x01, min);
    sdo_write_i32(cia402_od::SOFTWARE_LIMIT_MAX, 0x02, max);
}

void CiA402Drive::set_position_range_limits(int32_t min, int32_t max) {
    sdo_write_i32(cia402_od::POSITION_LIMIT_MIN, 0x01, min);
    sdo_write_i32(cia402_od::POSITION_LIMIT_MAX, 0x02, max);
}

void CiA402Drive::set_max_motor_speed(uint32_t speed) {
    sdo_write_u32(cia402_od::MAX_MOTOR_SPEED, 0x00, speed);
}

void CiA402Drive::set_following_error_window(uint32_t window) {
    sdo_write_u32(cia402_od::POS_FOLLOWING_ERROR, 0x00, window);
}

void CiA402Drive::set_following_error_time(uint16_t time_ms) {
    sdo_write_u16(cia402_od::POSITION_WINDOW_TIME, 0x00, time_ms);
}

// ==================== Feedback ====================

int32_t CiA402Drive::get_actual_position() const {
    return actual_position_.load();
}

int32_t CiA402Drive::get_position_demand() const {
    return target_position_.load();
}

int32_t CiA402Drive::get_actual_velocity() const {
    return actual_velocity_.load();
}

int32_t CiA402Drive::get_velocity_demand() const {
    return target_velocity_.load();
}

int16_t CiA402Drive::get_actual_torque() const {
    return actual_torque_.load();
}

// ==================== Status ====================

bool CiA402Drive::is_enabled() const {
    uint16_t sw = statusword_.load();
    return (sw & StatuswordBits::OPERATION_ENABLED) != 0;
}

bool CiA402Drive::is_moving() const {
    return std::abs(actual_velocity_.load()) > 10;
}

bool CiA402Drive::has_fault() const {
    uint16_t sw = statusword_.load();
    return (sw & StatuswordBits::FAULT) != 0;
}

bool CiA402Drive::target_reached() const {
    uint16_t sw = statusword_.load();
    return (sw & StatuswordBits::TARGET_REACHED) != 0;
}

bool CiA402Drive::setpoint_acknowledged() const {
    uint16_t sw = statusword_.load();
    return (sw & StatuswordBits::SETPOINT_ACKNOWLEDGE) != 0;
}

SDOError CiA402Drive::poll_status() {
    uint16_t sw = 0;
    const SDOError err = sdo_read_u16_err(cia402_od::STATUSWORD, 0x00, sw);
    if (err == SDOError::OK) {
        statusword_.store(sw);
        update_state_from_statusword();
    }
    return err;
}

void CiA402Drive::handle_nmt_change(NMTState state) {
    if (state != NMTState::OPERATIONAL) {
        // Go to safe state
        if (is_enabled()) {
            quick_stop();
        }
    }
}

// ==================== Internal ====================

void CiA402Drive::update_state_from_statusword() {
    uint16_t sw = statusword_.load();
    CiA402State new_state;

    if (sw & StatuswordBits::FAULT) {
        new_state = (sw & StatuswordBits::OPERATION_ENABLED)
                        ? CiA402State::FAULT_REACTION_ACTIVE
                        : CiA402State::FAULT;
    } else if (sw & StatuswordBits::SWITCH_ON_DISABLED) {
        new_state = CiA402State::SWITCH_ON_DISABLED;
    } else if (!(sw & StatuswordBits::QUICK_STOP)) {
        new_state = CiA402State::QUICK_STOP_ACTIVE;
    } else if (sw & StatuswordBits::OPERATION_ENABLED) {
        new_state = CiA402State::OPERATION_ENABLED;
    } else if (sw & StatuswordBits::SWITCHED_ON) {
        new_state = CiA402State::SWITCHED_ON;
    } else if (sw & StatuswordBits::READY_TO_SWITCH_ON) {
        new_state = CiA402State::READY_TO_SWITCH_ON;
    } else {
        new_state = CiA402State::NOT_READY_TO_SWITCH_ON;
    }

    CiA402State old = state_.load();
    if (old != new_state) {
        state_.store(new_state);
        if (on_state_change) {
            on_state_change(old, new_state);
        }
    }

    // Check homing
    if ((sw & StatuswordBits::HOMING_COMPLETE) && is_homing()) {
        homed_.store(true);
        if (on_homing_complete) {
            on_homing_complete();
        }
    }
}

void CiA402Drive::send_controlword() {
    sdo_write_u16(cia402_od::CONTROLWORD, 0x00, controlword_.load());
}

void CiA402Drive::monitor_motion_loop() {
    while (motion_monitor_running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (!motion_timeout_enabled_.load()) continue;
        if (!is_enabled()) continue;

        if (check_motion_timeout()) {
            // Safety action: quick stop
            quick_stop();
        }
    }
}

void CiA402Drive::update_values() {
    // Refresh cached actual values from the drive via SDO
    int32_t pos = 0;
    int32_t vel = 0;
    int16_t torque = 0;
    uint16_t sw = 0;

    if (sdo_read_i32(cia402_od::POSITION_DEMAND, 0x00, pos)) {
        actual_position_.store(pos);
    }
    if (sdo_read_i32(cia402_od::VELOCITY_ACTUAL, 0x00, vel)) {
        actual_velocity_.store(vel);
    }
    if (sdo_read_i16(cia402_od::TORQUE_ACTUAL, 0x00, torque)) {
        actual_torque_.store(torque);
    }
    if (sdo_read_u16(cia402_od::STATUSWORD, 0x00, sw)) {
        statusword_.store(sw);
        update_state_from_statusword();
    }
}

// ==================== Timeout Safety Manager ====================

TimeoutSafetyManager::TimeoutSafetyManager() = default;

TimeoutSafetyManager::~TimeoutSafetyManager() {
    running_.store(false);
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

void TimeoutSafetyManager::register_drive(CiA402Drive* drive, uint32_t timeout_ms) {
    std::lock_guard<std::mutex> lock(drives_mutex_);

    DriveInfo info;
    info.drive = drive;
    info.node_id = drive->get_node_id();
    info.timeout_ms = timeout_ms;
    info.last_update = std::chrono::steady_clock::now();
    info.timed_out = false;

    monitored_drives_[drive] = info;

    if (!running_.load()) {
        running_.store(true);
        monitor_thread_ = std::thread(&TimeoutSafetyManager::monitoring_loop, this);
    }
}

void TimeoutSafetyManager::unregister_drive(CiA402Drive* drive) {
    std::lock_guard<std::mutex> lock(drives_mutex_);

    for (auto it = monitored_drives_.begin(); it != monitored_drives_.end(); ) {
        if (it->second.drive == drive) {
            it = monitored_drives_.erase(it);
        } else {
            ++it;
        }
    }

    if (monitored_drives_.empty() && running_.load()) {
        running_.store(false);
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
    }
}

void TimeoutSafetyManager::set_default_timeout(uint32_t ms) {
    default_timeout_ms_ = ms;
}

void TimeoutSafetyManager::trigger_safe_stop() {
    std::lock_guard<std::mutex> lock(drives_mutex_);

    for (auto& [drive_ptr, info] : monitored_drives_) {
        (void)drive_ptr;
        if (info.drive->is_enabled()) {
            info.drive->quick_stop();
        }
    }
}

bool TimeoutSafetyManager::is_all_safe() const {
    std::lock_guard<std::mutex> lock(drives_mutex_);

    for (const auto& [drive_ptr, info] : monitored_drives_) {
        (void)drive_ptr;
        if (info.drive->is_enabled() && !info.timed_out) {
            return false;
        }
    }
    return true;
}

std::vector<uint8_t> TimeoutSafetyManager::get_timed_out_nodes() const {
    std::lock_guard<std::mutex> lock(drives_mutex_);
    std::vector<uint8_t> result;

    for (const auto& [drive_ptr, info] : monitored_drives_) {
        (void)drive_ptr;
        if (info.timed_out) {
            result.push_back(info.node_id);
        }
    }
    return result;
}

void TimeoutSafetyManager::monitoring_loop() {
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::lock_guard<std::mutex> lock(drives_mutex_);

        auto now = std::chrono::steady_clock::now();

        for (auto& [drive_ptr, info] : monitored_drives_) {
            (void)drive_ptr;
            if (!info.drive->is_enabled()) continue;

            uint32_t timeout_ms = info.timeout_ms > 0 ? info.timeout_ms : default_timeout_ms_;
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - info.last_update).count();

            if (elapsed > timeout_ms && !info.timed_out) {
                info.timed_out = true;

                if (on_timeout) {
                    on_timeout(info.node_id);
                }

                // Trigger safe stop
                info.drive->quick_stop();
            }
        }
    }
}

} // namespace canopen
