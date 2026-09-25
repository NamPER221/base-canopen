/**
 * @file zlac8015_driver.hpp
 * @brief ZLAC8015D Dual-Axis Servo Driver
 *
 * Vendor driver built on top of the generic CANopen core library.
 * The ZLAC8015D is a dual-axis drive exposing both motors under shared
 * CiA 402 objects:
 *   - 0x6040 Controlword (shared for both axes)
 *   - 0x6041 Statusword  (shared)
 *   - 0x60FF Target velocity: sub1 = Left (i16), sub2 = Right (i16),
 *             sub3 = Combined 32-bit (Left | Right << 16)
 *   - 0x606C Velocity actual: sub1 = Left, sub2 = Right (RPM)
 *   - 0x6064 Position actual: sub1 = Left, sub2 = Right (encoder counts)
 *   - 0x6071 Target torque:   sub1 = Left, sub2 = Right (mNm)
 */

#ifndef CANOPEN_DRIVERS_ZLAC8015_ZLAC8015_DRIVER_HPP
#define CANOPEN_DRIVERS_ZLAC8015_ZLAC8015_DRIVER_HPP

#include <canopen/co/cia402/cia402_drive.hpp>
#include <cstdint>
#include <functional>
#include <iostream>
#include <string>

namespace canopen {
namespace drivers {

// ==================== ZLAC8015D OD Indexes ====================

namespace zlac_od {
    constexpr uint16_t CAN_NODE_ID     = 0x200A;
    constexpr uint16_t CAN_BAUDRATE    = 0x200B;
    constexpr uint16_t MOTOR_POLE_L    = 0x200C;  // sub1
    constexpr uint16_t MOTOR_POLE_R    = 0x200C;  // sub2
    constexpr uint16_t MOTOR_MAX_SPEED = 0x2008;
    constexpr uint16_t SYNC_CONTROL    = 0x200F;
} // namespace zlac_od

/**
 * @brief ZLAC8015D driver — control both axes of one drive node
 */
class ZLAC8015Driver {
public:
    /**
     * @brief Construct driver
     * @param node_id CANopen node ID of the ZLAC8015D (default 1)
     * @param bus Bus interface
     */
    explicit ZLAC8015Driver(uint8_t node_id = 1, BusInterface* bus = nullptr);

    ~ZLAC8015Driver();

    // Non-copyable
    ZLAC8015Driver(const ZLAC8015Driver&) = delete;
    ZLAC8015Driver& operator=(const ZLAC8015Driver&) = delete;

    // ==================== Configuration ====================

    void set_bus(BusInterface* bus);
    void set_sdo_timeout(uint32_t ms);
    uint8_t get_node_id() const { return node_id_; }

    /**
     * @brief Set operation mode (0x6060) with SDO retry (5 lần, giống lely)
     * @param mode 1=Profile Position, 3=Profile Velocity, 4=Profile Torque
     * @return true nếu ghi thành công
     */
    bool set_operation_mode(int8_t mode);

    /**
     * @brief Wheel parameters used by kinematics helpers
     */
    void set_wheel_parameters(double wheel_radius_m, double wheelbase_m);
    double wheel_radius() const { return wheel_radius_; }
    double wheelbase() const { return wheelbase_; }

    // ==================== Lifecycle ====================

    /**
     * @brief Reset communication + wait bootup + enable operation
     * @param timeout_ms Timeout for bootup
     * @return true on success
     */
    bool init(uint32_t timeout_ms = 3000);

    /**
     * @brief Enable both motors (Controlword 0x000F, transition to
     *        Switch On -> Enable Operation)
     * @return true if the drive reports OPERATION_ENABLED
     */
    bool enable();

    /**
     * @brief Disable both motors (Controlword 0x0000)
     */
    bool disable();

    /**
     * @brief Emergency stop (quick stop deceleration)
     */
    void quick_stop();

    /**
     * @brief Fault reset
     */
    bool fault_reset();

    /**
     * @brief Set motion profile for BOTH axes (per-axis objects
     *        0x6081/0x6083/0x6084, units: RPM and RPM/s)
     * @param profile_velocity Max velocity during ramping (RPM)
     * @param accel Acceleration (RPM/s)
     * @param decel Deceleration (RPM/s)
     * @return true if all writes succeeded
     */
    bool set_profile(uint32_t profile_velocity, uint32_t accel, uint32_t decel);

    // ==================== Velocity Control (RPM) ====================

    /**
     * @brief Set velocity for both wheels
     * @param left_rpm  Left wheel RPM (negative = reverse)
     * @param right_rpm Right wheel RPM
     * @return true on success
     */
    bool set_velocity_rpm(int16_t left_rpm, int16_t right_rpm);

    /**
     * @brief Stop both wheels (0 RPM)
     */
    bool stop() { return set_velocity_rpm(0, 0); }

    // ==================== Feedback ====================

    /**
     * @brief Read statusword (0x6041)
     */
    uint16_t read_status();

    /**
     * @brief Read actual velocity 0x606C:01 / 0x606C:02 (RPM)
     */
    int16_t get_velocity_left();
    int16_t get_velocity_right();

    /**
     * @brief Read actual position 0x6064:01 / 0x6064:02 (encoder counts)
     */
    int32_t get_position_left();
    int32_t get_position_right();

    /**
     * @brief Refresh status + cached values from the drive
     */
    void update();

    // ==================== Status Checks ====================

    bool is_enabled() const { return drive_ && drive_->is_enabled(); }
    bool has_fault() const { return drive_ && drive_->has_fault(); }
    CiA402State get_state() const { return drive_ ? drive_->get_state() : CiA402State::NOT_READY_TO_SWITCH_ON; }

    // ==================== Callbacks ====================

    std::function<void(CiA402State old_state, CiA402State new_state)> on_state_change;
    std::function<void()> on_motion_timeout;

    /**
     * @brief Log hook — nhận thông báo từng bước khởi tạo/enable
     *        (mặc định in ra stderr để debug dễ dàng)
     */
    std::function<void(const std::string&)> logger;

    void log(const std::string& msg) const {
        if (logger) {
            logger(msg);
        } else {
            std::cerr << "[zlac] " << msg << std::endl;
        }
    }

private:
    uint8_t node_id_;
    BusInterface* bus_{nullptr};
    std::unique_ptr<CiA402Drive> drive_;
    double wheel_radius_{0.0865};
    double wheelbase_{0.400};
};

} // namespace drivers
} // namespace canopen

#endif // CANOPEN_DRIVERS_ZLAC8015_ZLAC8015_DRIVER_HPP
