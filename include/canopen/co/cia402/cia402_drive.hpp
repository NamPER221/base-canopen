/**
 * @file cia402_drive.hpp
 * @brief CiA 402 Drive Profile Implementation (generic)
 *
 * Works with any CiA 402 compliant servo drive over the CANopen protocol
 * layer (SDO for configuration/commands, OD access for feedback). The
 * drive is NOT tied to any specific vendor — manufacturer-specific
 * behavior belongs in vendor driver classes built on top of this.
 */

#ifndef CANOPEN_CO_CIA402_CIA402_DRIVE_HPP
#define CANOPEN_CO_CIA402_CIA402_DRIVE_HPP

#include <canopen/co/nmt/nmt.hpp>
#include <canopen/co/sdo/sdo.hpp>
#include <canopen/co/pdo/pdo.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>

namespace canopen {

// Forward declarations
class ObjectDictionary;

// ==================== CiA 402 OD Indexes (generic) ====================

namespace cia402_od {
    constexpr uint16_t CONTROLWORD           = 0x6040;
    constexpr uint16_t STATUSWORD            = 0x6041;
    constexpr uint16_t MODES_OF_OPERATION    = 0x6060;
    constexpr uint16_t MODES_OF_OPERATION_D  = 0x6061;
    constexpr uint16_t POSITION_DEMAND       = 0x6062;
    constexpr uint16_t POS_FOLLOWING_ERROR   = 0x6065;
    constexpr uint16_t POSITION_WINDOW       = 0x6067;
    constexpr uint16_t POSITION_WINDOW_TIME  = 0x6068;
    constexpr uint16_t VELOCITY_DEMAND       = 0x606B;
    constexpr uint16_t VELOCITY_ACTUAL       = 0x606C;
    constexpr uint16_t VELOCITY_WINDOW       = 0x606D;
    constexpr uint16_t TORQUE_DEMAND         = 0x6071;
    constexpr uint16_t TORQUE_MAX            = 0x6072;
    constexpr uint16_t TARGET_POSITION       = 0x607A;
    constexpr uint16_t POSITION_LIMIT_MIN    = 0x607B;
    constexpr uint16_t POSITION_LIMIT_MAX    = 0x607B;
    constexpr uint16_t TORQUE_ACTUAL         = 0x6077;
    constexpr uint16_t PROFILE_VELOCITY      = 0x6081;
    constexpr uint16_t MAX_MOTOR_SPEED       = 0x6080;
    constexpr uint16_t PROFILE_ACCELERATION  = 0x6083;
    constexpr uint16_t PROFILE_DECELERATION  = 0x6084;
    constexpr uint16_t QUICK_STOP_DECEL      = 0x6085;
    constexpr uint16_t HOMING_METHOD         = 0x6098;
    constexpr uint16_t HOMING_SPEED_SWITCH   = 0x6099;
    constexpr uint16_t HOMING_SPEED_ZERO     = 0x609A;
    constexpr uint16_t SOFTWARE_LIMIT_MIN    = 0x607D;
    constexpr uint16_t SOFTWARE_LIMIT_MAX    = 0x607D;
    constexpr uint16_t TARGET_VELOCITY       = 0x60FF;
    constexpr uint16_t TARGET_TORQUE         = 0x6071;
} // namespace cia402_od

/**
 * @brief CiA 402 State Machine
 */
enum class CiA402State : uint16_t {
    NOT_READY_TO_SWITCH_ON = 0x0000,
    SWITCH_ON_DISABLED = 0x0001,
    READY_TO_SWITCH_ON = 0x0002,
    SWITCHED_ON = 0x0003,
    OPERATION_ENABLED = 0x0004,
    QUICK_STOP_ACTIVE = 0x0005,
    FAULT_REACTION_ACTIVE = 0x0006,
    FAULT = 0x0007,
};

/**
 * @brief CiA 402 Operation Modes
 */
enum class OperationMode : int8_t {
    NO_MODE = 0,
    PROFILED_POSITION = 1,
    VELOCITY = 2,
    PROFILED_VELOCITY = 3,
    TORQUE = 4,
    PROFILED_TORQUE = 5,
    HOMING = 6,
    CYCLIC_SYNC_POSITION = 8,
    CYCLIC_SYNC_VELOCITY = 9,
    CYCLIC_SYNC_TORQUE = 10,
};

/**
 * @brief Fault Actions
 */
enum class FaultAction {
    NO_ACTION,
    FAULT_SIGNAL,
    HOLD_POSITION,
    QUICK_STOP,
    STOP_MOTOR,
};

/**
 * @brief CiA 402 Drive
 */
class CiA402Drive {
public:
    /**
     * @brief Construct CiA 402 Drive
     * @param node_id CANopen node ID of the drive
     * @param bus Bus interface (may be set later via set_bus/attach)
     */
    CiA402Drive(uint8_t node_id, BusInterface* bus = nullptr);

    ~CiA402Drive();

    // ==================== Configuration ====================

    void set_bus(BusInterface* bus);
    void attach(BusInterface& bus);
    void detach(BusInterface& bus);
    uint8_t get_node_id() const { return node_id_; }

    /**
     * @brief SDO timeout in ms (default 200)
     */
    void set_sdo_timeout(uint32_t ms) { sdo_timeout_ms_ = ms; }

    /**
     * @brief Bật trace SDO request/response ra stderr (debug)
     */
    void set_sdo_verbose(bool on) {
        verbose_ = on;
        if (sdo_) sdo_->set_verbose(on);
    }

    // ==================== Lifecycle ====================

    void init();
    void start();
    void stop();
    void shutdown();

    // ==================== CiA 402 State Machine ====================

    /**
     * @brief Get current state
     */
    CiA402State get_state() const { return state_.load(); }

    /**
     * @brief Get state word
     */
    uint16_t get_statusword() const { return statusword_.load(); }

    /**
     * @brief Get control word
     */
    uint16_t get_controlword() const { return controlword_.load(); }

    /**
     * @brief Set control word
     */
    void set_controlword(uint16_t cw);

    /**
     * @brief Transition to target state
     */
    bool set_state(CiA402State target);

    /**
     * @brief Quick stop (safety)
     */
    void quick_stop();

    /**
     * @brief Fault reset
     */
    void fault_reset();

    // ==================== Operation Mode ====================

    /**
     * @brief Set operation mode
     */
    bool set_operation_mode(OperationMode mode);

    /**
     * @brief Get current operation mode
     */
    OperationMode get_operation_mode() const { return current_mode_.load(); }

    // ==================== Position Control (Profiled Position Mode) ====================

    void set_target_position(int32_t position);
    void set_position_profile(uint32_t max_speed, uint32_t acceleration, uint32_t deceleration);
    void start_position_move(int32_t position, bool absolute = true);
    void start_relative_move(int32_t offset);
    void halt_position();
    void immediate_position(int32_t position);

    // ==================== Velocity Control ====================

    void set_target_velocity(int32_t velocity);
    void set_velocity_profile(uint32_t max_acceleration, uint32_t max_deceleration);
    void start_velocity_move(int32_t velocity);
    void halt_velocity();

    // ==================== Torque Control ====================

    void set_target_torque(int16_t torque);
    void set_torque_slope(uint16_t slope);

    // ==================== Homing ====================

    void start_homing(int16_t method);
    bool is_homing() const;
    bool is_homed() const { return homed_.load(); }

    // ==================== Cyclic Sync Modes ====================

    void set_target_position_cyclic(int32_t position);
    void set_target_velocity_cyclic(int32_t velocity);
    void set_target_torque_cyclic(int16_t torque);

    // ==================== Interpolation ====================

    void set_interpolation_time_period(uint16_t period_us, uint8_t index);
    void set_interpolation_data_record(int32_t position, int16_t velocity, int16_t torque);

    // ==================== Safety ====================

    /**
     * @brief Set motion timeout monitoring
     * @param timeout_ms Timeout in milliseconds (0 = disabled)
     */
    void set_motion_timeout(uint32_t timeout_ms);

    /**
     * @brief Enable/disable motion timeout
     */
    void enable_motion_timeout(bool enable);

    /**
     * @brief Check if motion timeout occurred
     */
    bool check_motion_timeout();

    /**
     * @brief Reset motion timeout flag
     */
    void reset_motion_timeout();

    // ==================== Position Limits ====================

    void set_software_position_limits(int32_t min, int32_t max);
    void set_position_range_limits(int32_t min, int32_t max);
    void set_max_motor_speed(uint32_t speed);

    // ==================== Follow Error ====================

    void set_following_error_window(uint32_t window);
    void set_following_error_time(uint16_t time_ms);

    // ==================== Current Values ====================

    int32_t get_actual_position() const;
    int32_t get_position_demand() const;
    int32_t get_actual_velocity() const;
    int32_t get_velocity_demand() const;
    int16_t get_actual_torque() const;

    // ==================== Multi-axis helpers ====================
    // For drives exposing per-axis data under shared CiA 402 objects
    // via subindexes (e.g., dual-axis drivers: 0x60FF:01 = Left,
    // 0x60FF:02 = Right, 0x60FF:03 = combined 32-bit).

    bool write_velocity_axis(uint8_t sub, int16_t rpm) {
        return sdo_write_i16(cia402_od::TARGET_VELOCITY, sub, rpm);
    }
    bool write_velocity_combined(uint32_t combined) {
        return sdo_write_u32(cia402_od::TARGET_VELOCITY, 0x03, combined);
    }
    // Gửi target velocity KHÔNG chờ SDO response — độ trễ ≈ 0 (như RPDO).
    // Dùng cho control loop khi firmware không hỗ trợ RPDO.
    bool write_velocity_combined_nowait(uint32_t combined) {
        return sdo_ && sdo_->download_nowait(
            cia402_od::TARGET_VELOCITY, 0x03, &combined, sizeof(combined));
    }
    /** Số lệnh nowait chưa được drive confirm */
    int sdo_outstanding() const { return sdo_ ? sdo_->outstanding() : 0; }
    bool read_velocity_axis(uint8_t sub, int16_t& rpm) {
        return sdo_read_i16(cia402_od::VELOCITY_ACTUAL, sub, rpm);
    }
    bool read_position_axis(uint8_t sub, int32_t& counts) {
        return sdo_read_i32(0x6064, sub, counts);  // position actual value
    }
    bool read_position_actual_axis(uint8_t sub, int32_t& counts) {
        return sdo_read_i32(0x6064, sub, counts);
    }

    // Per-axis motion profile (for drives exposing 0x6081/0x6083/0x6084
    // per axis via subindexes, e.g., ZLAC8015D). Units: RPM and RPM/s.
    bool write_profile_velocity_axis(uint8_t sub, uint32_t rpm) {
        return sdo_write_u32(cia402_od::PROFILE_VELOCITY, sub, rpm);
    }
    bool write_profile_acceleration_axis(uint8_t sub, uint32_t accel_rpm_s) {
        return sdo_write_u32(cia402_od::PROFILE_ACCELERATION, sub, accel_rpm_s);
    }
    bool write_profile_deceleration_axis(uint8_t sub, uint32_t decel_rpm_s) {
        return sdo_write_u32(cia402_od::PROFILE_DECELERATION, sub, decel_rpm_s);
    }
    bool write_quick_stop_deceleration_axis(uint8_t sub, uint32_t decel_rpm_s) {
        return sdo_write_u32(cia402_od::QUICK_STOP_DECEL, sub, decel_rpm_s);
    }

    // ==================== Encoder resolution (0x200E) ====================
    // ZLAC8015D: 0x200E = "Encoder Line" (counts per motor revolution),
    // sub1 = Left motor, sub2 = Right motor. Needed to convert 0x6064
    // position actual value (raw counts) into radians/odometry.
    bool read_encoder_line(uint8_t sub, uint16_t& lines) {
        return sdo_read_u16(0x200E, sub, lines);
    }
    bool write_encoder_line(uint8_t sub, uint16_t lines) {
        return sdo_write_u16(0x200E, sub, lines);
    }

    // ==================== Status Checks ====================

    bool is_enabled() const;
    bool is_moving() const;
    bool has_fault() const;
    bool target_reached() const;
    bool setpoint_acknowledged() const;

    /**
     * @brief Poll statusword from the drive via SDO and update state machine
     * @return SDOError result
     */
    SDOError poll_status();

    // ==================== Callbacks ====================

    std::function<void(CiA402State old_state, CiA402State new_state)> on_state_change;
    std::function<void()> on_target_reached;
    std::function<void(uint16_t error_code)> on_fault;
    std::function<void()> on_homing_complete;
    std::function<void()> on_motion_timeout;
    std::function<void()> on_fault_reset;

    // ==================== NMT Handling ====================

    void handle_nmt_change(NMTState state);

    // ==================== Raw SDO access (setup / config) ====================
    // Dùng để cấu hình PDO mapping, đọc/ghi object bất kỳ.
    // KHÔNG dùng trong control loop thời gian thực (SDO chặn chờ response).

    bool sdo_write_u8(uint16_t index, uint8_t sub, uint8_t value) {
        return sdo_ && sdo_->download(index, sub, value) == SDOError::OK;
    }
    bool sdo_write_u16(uint16_t index, uint8_t sub, uint16_t value) {
        return sdo_ && sdo_->download(index, sub, value) == SDOError::OK;
    }
    bool sdo_write_u32(uint16_t index, uint8_t sub, uint32_t value) {
        return sdo_ && sdo_->download(index, sub, value) == SDOError::OK;
    }
    bool sdo_write_i32(uint16_t index, uint8_t sub, int32_t value) {
        return sdo_ && sdo_->download(index, sub, value) == SDOError::OK;
    }
    bool sdo_write_i16(uint16_t index, uint8_t sub, int16_t value) {
        return sdo_ && sdo_->download(index, sub, value) == SDOError::OK;
    }
    bool sdo_write_i8(uint16_t index, uint8_t sub, int8_t value) {
        return sdo_ && sdo_->download(index, sub, value) == SDOError::OK;
    }
    bool sdo_read_u8(uint16_t index, uint8_t sub, uint8_t& value) {
        return sdo_ && sdo_->upload(index, sub, value) == SDOError::OK;
    }
    bool sdo_read_u16(uint16_t index, uint8_t sub, uint16_t& value) {
        return sdo_ && sdo_->upload(index, sub, value) == SDOError::OK;
    }
    bool sdo_read_u32(uint16_t index, uint8_t sub, uint32_t& value) {
        return sdo_ && sdo_->upload(index, sub, value) == SDOError::OK;
    }
    bool sdo_read_i32(uint16_t index, uint8_t sub, int32_t& value) {
        return sdo_ && sdo_->upload(index, sub, value) == SDOError::OK;
    }
    bool sdo_read_i16(uint16_t index, uint8_t sub, int16_t& value) {
        return sdo_ && sdo_->upload(index, sub, value) == SDOError::OK;
    }

    // Variant trả về SDOError để debug được lỗi thật
    SDOError sdo_read_u16_err(uint16_t index, uint8_t sub, uint16_t& value) {
        if (!sdo_) return SDOError::NO_BUS;
        return sdo_->upload(index, sub, value);
    }

private:
    void update_state_from_statusword();
    void send_controlword();
    void monitor_motion_loop();
    void update_values();

    uint8_t node_id_;
    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_{0};
    std::unique_ptr<SDOClient> sdo_;
    uint32_t sdo_timeout_ms_{200};
    bool verbose_{false};

    // CiA 402 State
    std::atomic<CiA402State> state_{CiA402State::NOT_READY_TO_SWITCH_ON};
    std::atomic<uint16_t> statusword_{0};
    std::atomic<uint16_t> controlword_{0x0000};

    // Values
    std::atomic<int32_t> target_position_{0};
    std::atomic<int32_t> actual_position_{0};
    std::atomic<int32_t> target_velocity_{0};
    std::atomic<int32_t> actual_velocity_{0};
    std::atomic<int16_t> target_torque_{0};
    std::atomic<int16_t> actual_torque_{0};

    // Mode
    std::atomic<OperationMode> current_mode_{OperationMode::NO_MODE};

    // Safety
    std::atomic<bool> motion_timeout_enabled_{false};
    std::atomic<uint32_t> motion_timeout_ms_{1000};
    std::atomic<bool> motion_timeout_occurred_{false};
    std::chrono::steady_clock::time_point last_motion_command_;

    std::thread motion_monitor_thread_;
    std::atomic<bool> motion_monitor_running_{false};

    // Homing
    std::atomic<bool> homed_{false};

    // Running state
    std::atomic<bool> running_{false};
};

/**
 * @brief Timeout Safety Manager
 */
class TimeoutSafetyManager {
public:
    TimeoutSafetyManager();
    ~TimeoutSafetyManager();

    /**
     * @brief Register drive for monitoring
     */
    void register_drive(CiA402Drive* drive, uint32_t timeout_ms);

    /**
     * @brief Unregister drive
     */
    void unregister_drive(CiA402Drive* drive);

    /**
     * @brief Set default timeout
     */
    void set_default_timeout(uint32_t ms);

    /**
     * @brief Trigger safe stop for all
     */
    void trigger_safe_stop();

    /**
     * @brief Check if all drives safe
     */
    bool is_all_safe() const;

    /**
     * @brief Get timed out nodes
     */
    std::vector<uint8_t> get_timed_out_nodes() const;

    /**
     * @brief Callback on timeout
     */
    std::function<void(uint8_t node_id)> on_timeout;

private:
    void monitoring_loop();

    struct DriveInfo {
        CiA402Drive* drive{nullptr};
        uint8_t node_id{0};
        uint32_t timeout_ms;
        std::chrono::steady_clock::time_point last_update;
        bool timed_out;
    };

    std::map<CiA402Drive*, DriveInfo> monitored_drives_;
    mutable std::mutex drives_mutex_;

    std::thread monitor_thread_;
    std::atomic<bool> running_{false};
    uint32_t default_timeout_ms_{1000};
};

} // namespace canopen

#endif // CANOPEN_CO_CIA402_CIA402_DRIVE_HPP
