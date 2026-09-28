/**
 * @file motor_device.hpp
 * @brief Lớp facade cấp cao cho một thiết bị động cơ CiA 402
 *
 * Mục tiêu: người dùng chỉ cần trỏ tới file EDS rồi gọi hàm, không phải biết
 * thiết bị dùng object nào, bao nhiêu byte, có dấu hay không, và enable theo
 * đúng thứ tự nào.
 *
 * @code
 *   auto profile = DeviceProfile::from_eds("servo.eds", 1);
 *   MotorDevice m(bus, profile);
 *   m.connect();                     // NMT + CiA 402 enable tự động
 *   m.set_operation_mode(OperationMode::PROFILE_VELOCITY);
 *   m.set_velocity(500);             // đơn vị do EDS khai báo
 *   double v = m.velocity();         // TPDO nếu có, không thì SDO
 * @endcode
 *
 * Với thiết bị lệch chuẩn, dùng DeviceProfile::set_override() trước khi tạo
 * MotorDevice — không phải viết lại driver.
 */

#ifndef CANOPEN_DEVICE_MOTOR_DEVICE_HPP
#define CANOPEN_DEVICE_MOTOR_DEVICE_HPP

#include <canopen/can/raw/bus_interface.hpp>
#include <canopen/co/cia402/cia402_drive.hpp>
#include <canopen/co/sdo/sdo.hpp>
#include <canopen/device/device_profile.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <string>

namespace canopen {

/**
 * @brief Thiết bị động cơ CiA 402, điều khiển được bằng API đơn giản
 */
class MotorDevice {
public:
    /**
     * @param bus  Bus đã mở
     * @param profile Hồ sơ thiết bị (từ EDS hoặc dựng tay)
     */
    MotorDevice(BusInterface& bus, DeviceProfile profile);
    ~MotorDevice();

    MotorDevice(const MotorDevice&) = delete;
    MotorDevice& operator=(const MotorDevice&) = delete;

    // ==================== Vòng đời kết nối ====================

    /**
     * @brief Kết nối và bật thiết bị.
     *
     * Gồm: chờ bootup → NMT Start → chờ Operational → chạy chuỗi
     * enable của CiA 402 cho tới khi statusword báo OPERATION_ENABLED.
     * Trả về false nếu thiết bị không phản hồi hoặc không vào được
     * OPERATION_ENABLED.
     */
    bool connect(uint32_t timeout_ms = 3000);

    /** @brief Đặt thiết bị về trạng thái dừng an toàn rồi tắt nguồn điều khiển */
    void disconnect();

    /** @brief Kết nối lại bằng cách chạy lại connect() (dùng khi mất giao tiếp) */
    bool reconnect(uint32_t timeout_ms = 3000);

    bool is_connected() const { return connected_.load(); }
    /** @brief Hồ sơ thiết bị có đủ object cần thiết không */
    bool is_usable() const { return profile_.is_usable(); }

    // ==================== Chế độ vận hành ====================

    /**
     * @brief Chuyển chế độ vận hành (velocity / position / torque)
     *
     * Một số drive cần ghi nhiều lần mới nhận, nên có sẵn cơ chế retry.
     */
    bool set_operation_mode(OperationMode mode, int attempts = 3);

    OperationMode operation_mode() const { return mode_; }

    // ==================== Profile tốc độ ====================

    /**
     * @brief Đặt tốc độ tham chiếu, ramp lên nhanh chậm tùy profile
     *
     * @param velocity Giá trị theo đơn vị thiết bị (xem velocity_unit())
     */
    bool set_profile_velocity(double velocity);
    bool set_profile_acceleration(double accel);
    bool set_profile_deceleration(double decel);

    // ==================== Điều khiển tốc độ ====================

    /**
     * @brief Ra lệnh tốc độ
     * @param velocity Giá trị theo đơn vị thiết bị
     *
     * Ghi trực tiếp vào object target velocity đã dò sẵn — không cần biết
     * index/subindex/kiểu dữ liệu của thiết bị.
     */
    bool set_velocity(double velocity);

    /**
     * @brief Đọc tốc độ thực tế
     * @return Giá trị theo đơn vị thiết bị, hoặc 0 nếu thiết bị không có
     *         object actual velocity
     */
    double velocity();

    /** @brief Dừng tốc độ ngay (velocity = 0) */
    bool stop();

    // ==================== Trạng thái ====================

    /** @brief Trạng thái CiA 402 (đọc statusword) */
    CiA402State cia402_state();

    /** @brief Đã bật motor hay chưa (Operation Enabled) */
    bool is_operational() { return cia402_state() == CiA402State::OPERATION_ENABLED; }

    /** @brief statusword thô */
    uint16_t statusword();

    /** @brief Mã lỗi nội bộ của drive (0x603F), 0 = không lỗi */
    uint32_t error_code();

    // ==================== Truy cập hạ cấp ====================

    /**
     * @brief Đọc bất kỳ object nào theo vai trò đã dò
     * @return false nếu thiết bị không có object đó
     */
    bool read_role(ObjectRole role, double& out);
    bool write_role(ObjectRole role, double value);

    /** @brief Đọc object tùy ý bằng địa chỉ thô (khi cần đọc thứ ngoài vai trò) */
    bool read_value(uint16_t index, uint8_t subindex, double& out);
    bool write_value(uint16_t index, uint8_t subindex, double value);

    const DeviceProfile& profile() const { return profile_; }

    /**
     * @brief Đổi thứ tự byte (cho thiết bị dùng big-endian)
     *
     * Mặc định LittleEndian theo CiA 301. Chỉ ảnh hưởng tới các lần đọc/ghi
     * qua hồ sơ (read_role/write_role), không đụng tới PDO.
     */
    void set_byte_order(ByteOrder order) { profile_.set_byte_order(order); }
    ByteOrder byte_order() const { return profile_.byte_order(); }
    SDOClient& sdo() { return *sdo_; }
    uint8_t node_id() const { return profile_.node_id(); }

    /** @brief Nhật ký của driver (mặc định im lặng) */
    std::function<void(const std::string&)> logger;

    /** @brief Bộ đếm số lần gọi set_velocity (tiện cho debug/đo) */
    uint32_t command_count() const { return command_count_.load(); }

protected:
    /** Ghi controlword và chờ statusword chuyển sang state mong muốn */
    bool transition(uint16_t controlword, CiA402State expect, uint32_t timeout_ms);

    void log(const std::string& msg) {
        if (logger) logger(msg);
    }

    BusInterface* bus_{nullptr};
    DeviceProfile profile_;
    std::unique_ptr<SDOClient> sdo_;
    OperationMode mode_{OperationMode::NO_MODE};
    std::atomic<bool> connected_{false};
    std::atomic<uint32_t> command_count_{0};
};

} // namespace canopen

#endif // CANOPEN_DEVICE_MOTOR_DEVICE_HPP
