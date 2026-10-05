/**
 * @file mbdv_driver.hpp
 * @brief Moons' MBDV Dual-Axis CANopen Servo Driver (MBDV-2X-520AC)
 *
 * Implements CiA 402 drive profile for Moons' MBDV Dual-Axis servo drives.
 * Each axis functions as an independent CANopen node on a shared physical CAN bus:
 *   - Axis 1 (Left):  Node ID configured via DIP switch SW1..SW3 (default = 1)
 *   - Axis 2 (Right): Node ID configured via DIP switch SW4..SW6 (default = 2)
 *
 * Provides:
 *   - Low-latency RPDO/TPDO velocity & position streaming
 *   - CiA 402 State Machine control (Shutdown -> Switch On -> Operation Enabled)
 *   - Built-in Differential Drive Kinematics (cmd_vel <-> Odometry)
 *   - Real-time telemetry, health monitoring, and hardware diagnostics
 */

#ifndef CANOPEN_DRIVERS_MBDV_MBDV_DRIVER_HPP
#define CANOPEN_DRIVERS_MBDV_MBDV_DRIVER_HPP

#include <canopen/co/cia402/cia402_drive.hpp>
#include <canopen/co/sync/sync.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <canopen/kinematics/differential_drive.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace canopen {
namespace drivers {

// ==================== Moons' MBDV OD Indexes ====================

namespace mbdv_od {
    // CiA 402 Standard Objects (subindex 0x00 for single-axis node)
    constexpr uint16_t ERROR_CODE              = 0x603F;
    constexpr uint16_t CONTROLWORD             = 0x6040;
    constexpr uint16_t STATUSWORD              = 0x6041;
    constexpr uint16_t MODES_OF_OPERATION      = 0x6060;
    constexpr uint16_t MODES_OF_OPERATION_DISP = 0x6061;
    constexpr uint16_t POSITION_ACTUAL         = 0x6064;
    constexpr uint16_t VELOCITY_ACTUAL         = 0x606C;
    constexpr uint16_t TARGET_TORQUE           = 0x6071;
    constexpr uint16_t TARGET_POSITION         = 0x607A;
    constexpr uint16_t PROFILE_VELOCITY        = 0x6081;
    constexpr uint16_t PROFILE_ACCELERATION    = 0x6083;
    constexpr uint16_t PROFILE_DECELERATION    = 0x6084;
    constexpr uint16_t QUICK_STOP_DECEL        = 0x6085;
    constexpr uint16_t DIGITAL_INPUTS          = 0x60FD;
    constexpr uint16_t DIGITAL_OUTPUTS         = 0x60FE;
    constexpr uint16_t TARGET_VELOCITY         = 0x60FF;

    // Manufacturer Specific Objects (from EDS & Hardware Manual)
    constexpr uint16_t TPDO1_COMM_PARAM        = 0x1800; // TPDO1 Communication Parameter
    constexpr uint16_t TPDO2_COMM_PARAM        = 0x1801; // TPDO2 Communication Parameter
    constexpr uint16_t TPDO3_COMM_PARAM        = 0x1802; // TPDO3 Communication Parameter
    constexpr uint16_t TPDO4_COMM_PARAM        = 0x1803; // TPDO4 Communication Parameter
    constexpr uint16_t DSP_STATUS_CODE         = 0x200B;
    constexpr uint16_t DSP_ALARM_CODE          = 0x200F;
    constexpr uint16_t DEVICE_TEMPERATURE      = 0x2019; // sub1: drive temp (°C)
    constexpr uint16_t DC_BUS_VOLTAGE          = 0x2030; // 0.1 V
    constexpr uint16_t ENCODER_RESOLUTION      = 0x2A0B; // raw resolution
    constexpr uint16_t STEPS_PER_REV           = 0x2A90; // Electronic Gearing (P3-05, default 10000)
    constexpr uint16_t SUB_ALARM_CODE          = 0x2AC0;
} // namespace mbdv_od

// ==================== Data Structures ====================

/**
 * @brief Kinematic configuration parameters for Moons' MBDV mobile base
 */
struct MbdvKinematicsConfig {
    double wheel_radius_m{0.07333};           // Wheel radius r [m]
    double wheelbase_m{0.4544};              // Wheel track width L [m]
    double gear_ratio{1.0};                  // Reduction ratio N
    int32_t encoder_cpr{10000};              // Counts per revolution (P3-05 / 0x2A90)
    bool invert_left{false};                 // Reverse left motor direction
    bool invert_right{false};                // Reverse right motor direction
    double max_linear_velocity_m_s{1.5};     // Linear clamp [m/s]
    double max_angular_velocity_rad_s{3.0};  // Angular clamp [rad/s]
};

/**
 * @brief Robot 2D pose in odometry frame
 */
struct MbdvPose {
    double x{0.0};      // [m]
    double y{0.0};      // [m]
    double theta{0.0};  // [rad], normalized to [-pi, pi]
};

/**
 * @brief Robot body twist (linear x, angular z)
 */
struct MbdvTwist {
    double linear_v{0.0};   // [m/s]
    double angular_w{0.0};  // [rad/s]
};

/**
 * @brief Individual axis real-time telemetry
 */
struct MbdvAxisTelemetry {
    uint8_t node_id{0};
    bool online{false};
    bool operational{false};
    CiA402State state{CiA402State::NOT_READY_TO_SWITCH_ON};
    uint16_t statusword{0};
    int32_t actual_position_counts{0};
    int32_t actual_velocity_counts_s{0};
    double actual_velocity_rpm{0.0};
    double voltage_v{0.0};
    int16_t temperature_c{0};
    uint16_t dsp_alarm_code{0};
    uint32_t tpdo_count{0};
    int64_t ms_since_heartbeat{-1};
};

/**
 * @brief Complete dual-axis mobile base telemetry snapshot
 */
struct MbdvTelemetry {
    MbdvAxisTelemetry axis1;
    MbdvAxisTelemetry axis2;
    MbdvPose pose;
    MbdvTwist twist;
    uint64_t timestamp_us{0};
};

// ==================== Single Axis Controller ====================

/**
 * @brief Controls a single axis of Moons' MBDV drive (CiA 402 CANopen node)
 */
class MbdvAxis {
public:
    explicit MbdvAxis(uint8_t node_id, std::string name = "Axis");
    ~MbdvAxis();

    MbdvAxis(const MbdvAxis&) = delete;
    MbdvAxis& operator=(const MbdvAxis&) = delete;

    void set_bus(BusInterface* bus);
    void set_sdo_timeout(uint32_t ms);
    uint8_t get_node_id() const { return node_id_; }
    const std::string& name() const { return name_; }

    CiA402Drive& drive() { return *drive_; }
    const CiA402Drive& drive() const { return *drive_; }

    // Lifecycle
    bool init(uint32_t timeout_ms = 3000);
    bool enable(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));
    bool disable();
    void quick_stop();
    bool fault_reset();

    // Mode & Motion
    bool set_operation_mode(OperationMode mode);
    bool set_profile(uint32_t accel_counts_s2, uint32_t decel_counts_s2);
    void set_target_velocity_counts(int32_t counts_per_sec);
    bool set_target_velocity_sdo(int32_t counts_per_sec);
    void set_target_position_counts(int32_t counts, bool new_setpoint = true,
                                    bool immediate = true, bool relative = false);

    // PDO Setup & Transmission
    void setup_pdo_routes(BusInterface* bus);
    void remove_pdo_routes();
    bool send_rpdo3_velocity(int32_t counts_per_sec, uint16_t controlword = 0x000F);
    bool send_rpdo2_position(int32_t counts, uint16_t controlword);

    // Telemetry & Health
    MbdvAxisTelemetry get_telemetry() const;
    bool check_health(uint32_t heartbeat_timeout_ms = 1500);
    bool poll_diagnostics_sdo();
    bool poll_feedback_sdo();
    void update_from_drive_poll();

    int32_t actual_position() const { return actual_position_.load(); }
    int32_t actual_velocity() const { return actual_velocity_.load(); }
    uint16_t statusword() const { return statusword_.load(); }
    CiA402State state() const { return state_.load(); }
    bool is_operational() const { return operational_.load(); }
    bool is_online() const { return online_.load(); }

    // Callbacks
    std::function<void(CiA402State old_s, CiA402State new_s)> on_state_change;
    std::function<void(uint16_t error_code)> on_emcy;

private:
    void handle_tpdo1(const CANFrame& frame);
    void handle_tpdo2(const CANFrame& frame);
    void handle_tpdo3(const CANFrame& frame);
    void handle_tpdo4(const CANFrame& frame);
    void handle_heartbeat(const CANFrame& frame);
    void handle_emcy(const CANFrame& frame);

    uint8_t node_id_;
    std::string name_;
    BusInterface* bus_{nullptr};
    std::unique_ptr<CiA402Drive> drive_;

    // Thread-safe telemetry state
    std::atomic<bool> online_{false};
    std::atomic<bool> operational_{false};
    std::atomic<CiA402State> state_{CiA402State::NOT_READY_TO_SWITCH_ON};
    std::atomic<uint16_t> statusword_{0};
    std::atomic<int32_t> actual_position_{0};
    std::atomic<int32_t> actual_velocity_{0};
    std::atomic<double> voltage_{0.0};
    std::atomic<int16_t> temperature_{0};
    std::atomic<uint16_t> dsp_alarm_{0};
    std::atomic<uint32_t> tpdo_count_{0};
    std::atomic<std::chrono::steady_clock::time_point::duration::rep> last_heartbeat_time_{0};
    int32_t last_target_velocity_{0};

    // Route handles
    std::vector<BusInterface::RouteHandle> route_handles_;
    mutable std::mutex telemetry_mutex_;
};

// ==================== Dual Axis Coordinator ====================

/**
 * @brief Dual-Axis controller for Moons' MBDV (MBDV-2X-520AC) with Kinematics
 */
class MbdvDriver {
public:
    /**
     * @brief Construct dual-axis driver
     * @param node_id_1 Axis 1 Node ID (default 1)
     * @param node_id_2 Axis 2 Node ID (default 2)
     * @param bus Bus interface
     */
    explicit MbdvDriver(uint8_t node_id_1 = 1, uint8_t node_id_2 = 2,
                        BusInterface* bus = nullptr);
    ~MbdvDriver();

    MbdvDriver(const MbdvDriver&) = delete;
    MbdvDriver& operator=(const MbdvDriver&) = delete;

    void set_bus(BusInterface* bus);
    void set_sdo_timeout(uint32_t ms);

    MbdvAxis& axis1() { return axis1_; }
    const MbdvAxis& axis1() const { return axis1_; }
    MbdvAxis& axis2() { return axis2_; }
    const MbdvAxis& axis2() const { return axis2_; }

    void set_single_axis_mode(bool single_axis) { single_axis_mode_ = single_axis; }
    bool is_single_axis_mode() const { return single_axis_mode_; }

    void enable_sync(bool enable) { sync_enabled_ = enable; }
    bool is_sync_enabled() const { return sync_enabled_; }

    // Kinematics Configuration
    void set_kinematics_config(const MbdvKinematicsConfig& config);
    const MbdvKinematicsConfig& kinematics_config() const { return config_; }

    // Lifecycle
    bool init(uint32_t timeout_ms = 3000);
    bool enable(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));
    bool disable();
    void quick_stop();
    bool fault_reset();

    // High-Level Kinematics Control
    /**
     * @brief Command robot velocities (linear m/s, angular rad/s)
     * Automatically applies inverse kinematics, velocity clamping, and sends RPDO3.
     */
    void set_cmd_vel(double linear_v, double angular_w);

    /**
     * @brief Set direct counts/s velocity target to both axes
     */
    void set_wheel_velocities(int32_t left_counts_s, int32_t right_counts_s);

    /**
     * @brief Update Odometry pose from encoder feedback
     * @param dt_sec Time elapsed since previous update
     */
    void update_odometry(double dt_sec);

    void reset_odometry(double x = 0.0, double y = 0.0, double theta = 0.0);

    // Telemetry
    MbdvPose get_pose() const;
    MbdvTwist get_twist() const;
    MbdvTelemetry get_telemetry() const;

    bool check_health();

private:
    double normalize_angle(double angle) const;

    uint8_t node_id_1_;
    uint8_t node_id_2_;
    bool single_axis_mode_{false};
    BusInterface* bus_{nullptr};

    MbdvAxis axis1_;
    MbdvAxis axis2_;
    MbdvKinematicsConfig config_;

    // Continuous CANopen SYNC Service (10 ms period, COB-ID 0x080)
    SYNCService sync_service_;
    bool sync_enabled_{true};

    // Odometry state
    mutable std::mutex odom_mutex_;
    MbdvPose pose_;
    MbdvTwist current_twist_;
    int32_t prev_ticks_left_{0};
    int32_t prev_ticks_right_{0};
    bool first_odom_update_{true};
};

} // namespace drivers
} // namespace canopen

#endif // CANOPEN_DRIVERS_MBDV_MBDV_DRIVER_HPP
