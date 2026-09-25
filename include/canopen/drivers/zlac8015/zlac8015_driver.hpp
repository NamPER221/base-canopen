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
#include <atomic>
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

    // ==================== Encoder / Odometry ====================

    /**
     * @brief Đọc số counts/rev của encoder từ drive (0x200E:01 = Left,
     *        0x200E:02 = Right). Giá trị này drive tự biết — không hardcode.
     * @return true nếu đọc thành công
     */
    bool read_encoder_lines(uint16_t& left_lines, uint16_t& right_lines);

    /**
     * @brief Số counts/rev đã biết (0 = chưa đọc)
     */
    uint16_t encoder_lines_left() const { return encoder_lines_left_; }
    uint16_t encoder_lines_right() const { return encoder_lines_right_; }

    /**
     * @brief Đọc encoder_line từ drive (0x200E:01), fallback = 1024
     */
    uint16_t encoder_lines_left();

    /**
     * @brief counts → radian (dùng encoder_line thật của drive)
     */
    double counts_to_rad(int32_t counts) const;

    /**
     * @brief Ghi encoder_line cho cả 2 trục (0x200E:01/02) — dùng khi
     *        encoder thực tế khác mặc định của drive
     */
    bool set_encoder_lines(uint16_t lines);

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

    // ==================== PDO (thời gian thực, không chờ response) ====================

    /**
     * @brief Cấu hình PDO trên drive qua SDO (chỉ gọi 1 lần lúc setup):
     *   - RPDO1 (0x200+node): nhận 0x60FF:03 32-bit = left|right RPM
     *   - TPDO1 (0x180+node): gửi 0x606C:01 + 0x606C:02 (32-bit mỗi bên)
     *
     * Sau khi gọi, set_velocity_rpm() dùng RPDO (1 frame, không chặn),
     * và get_velocity_*_rpm() đọc từ TPDO cache (không chặn).
     *
     * @return true nếu cấu hình thành công
     */
    bool setup_pdo();

    /**
     * @brief PDO đã được cấu hình chưa
     */
    bool pdo_ready() const { return pdo_ready_; }

    /**
     * @brief Dùng RPDO để gửi tốc độ (mặc định khi pdo_ready).
     *        true = bỏ qua nếu tốc độ không đổi.
     */
    bool use_pdo(bool on) { pdo_enabled_ = on; return pdo_enabled_; }

    /**
     * @brief Tốc độ thực tế từ TPDO (0 = chưa nhận TPDO nào)
     */
    int32_t velocity_actual_left() const { return tpdo_vel_left_; }
    int32_t velocity_actual_right() const { return tpdo_vel_right_; }
    uint16_t statusword_pdo() const { return tpdo_statusword_; }
    bool tpdo_received() const { return tpdo_count_ > 0; }

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
    uint16_t encoder_lines_left_{0};   // 0 = chưa đọc từ drive
    uint16_t encoder_lines_right_{0};
    int16_t last_left_rpm_{0};
    int16_t last_right_rpm_{0};
    bool velocity_sent_{false};        // để phân biệt "chưa gửi" với "gửi 0 RPM"

    // PDO
    bool pdo_ready_{false};
    bool pdo_enabled_{true};
    uint32_t rpdo_cobid_{0x200};       // + node_id
    uint32_t tpdo_cobid_{0x180};       // + node_id
    int route_tpdo_{0};
    std::atomic<int32_t> tpdo_vel_left_{0};
    std::atomic<int32_t> tpdo_vel_right_{0};
    std::atomic<uint16_t> tpdo_statusword_{0};
    std::atomic<uint32_t> tpdo_count_{0};

    void on_tpdo_frame(const CANFrame& frame);
};

} // namespace drivers
} // namespace canopen

#endif // CANOPEN_DRIVERS_ZLAC8015_ZLAC8015_DRIVER_HPP
