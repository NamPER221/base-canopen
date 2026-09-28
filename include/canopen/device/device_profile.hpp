/**
 * @file device_profile.hpp
 * @brief Hồ sơ thiết bị: tự động tra EDS xác định các object cần dùng
 *
 * Mục đích: gọi thư viện với một thiết bị mà không cần biết trước nó dùng
 * object nào. CiA 402 quy định sẵn các object cơ bản (0x6040, 0x60FF, 0x606C
 * ...), EDS mô tả tên + kiểu dữ liệu + quyền truy cập của chúng, nên có thể
 * dò ra tự động.
 *
 * Với thiết bị lệch chuẩn (vd ZLAC8015D dùng 0x60FF:03 thay vì 0x60FF:00)
 * dùng set_override() để chỉ định lại — chỉ cần vài dòng, không phải viết
 * lại cả driver.
 */

#ifndef CANOPEN_DEVICE_DEVICE_PROFILE_HPP
#define CANOPEN_DEVICE_DEVICE_PROFILE_HPP

#include <canopen/can/frame/frame.hpp>
#include <canopen/co/object_dictionary/object_dictionary.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace canopen {

/**
 * @brief Một "vai trò" trong ứng dụng, ánh xạ tới object cụ thể của thiết bị
 */
enum class ObjectRole : uint8_t {
    Unknown = 0,
    Controlword,            // 0x6040 — lệnh bật/tắt motor
    Statusword,             // 0x6041 — trạng thái CiA 402
    ModesOfOperation,       // 0x6060 — chọn chế độ (3 = velocity)
    ModesOfOperationDisplay,// 0x6061
    TargetVelocity,         // 0x60FF — lệnh tốc độ
    ActualVelocity,         // 0x606C — tốc độ thực
    TargetPosition,         // 0x607A
    ActualPosition,         // 0x6064
    TargetTorque,           // 0x6071
    ActualTorque,           // 0x6077
    ProfileVelocity,        // 0x6081
    ProfileAcceleration,    // 0x6083
    ProfileDeceleration,    // 0x6084
    QuickStopDeceleration,  // 0x6085
    ErrorCode,              // 0x603F
};

/**
 * @brief Một object đã được phân giải: địa chỉ + thông tin kiểu dữ liệu
 */
struct ResolvedObject {
    bool valid{false};
    uint16_t index{0};
    uint8_t subindex{0};
    DataType data_type{DataType::UNSIGNED32};
    AccessType access{AccessType::RW};
    size_t size{4};
    bool is_signed{false};
    /// Thứ tự byte khi đóng gói/giải mã giá trị này
    ByteOrder byte_order{ByteOrder::LittleEndian};
    std::string name;         // tên trong EDS (đã lowercase)
    bool from_override{false};// true = do người dùng chỉ định, false = tự dò

    /** Giá trị đọc/ghi được đưa về kiểu double */
    bool to_double(const uint8_t* data, size_t len, double& out) const;
    /** Đóng gói double thành byte theo kiểu dữ liệu đã phân giải */
    bool from_double(double value, uint8_t* out, size_t& len) const;
};

/**
 * @brief Hồ sơ thiết bị, dựng từ EDS
 *
 * Cách dùng:
 * @code
 *   DeviceProfile p = DeviceProfile::from_eds("drive.eds", 1);
 *   p.set_override(ObjectRole::TargetVelocity, 0x60FF, 3);  // nếu lệch chuẩn
 *   if (!p.is_usable()) { ... }  // báo thiếu object bắt buộc
 *   MotorDevice dev(bus, p);
 * @endcode
 */
class DeviceProfile {
public:
    /**
     * @brief Nạp EDS và tự động dò các object CiA 402
     * @param eds_path Đường dẫn file .eds
     * @param node_id  Node ID của thiết bị (để thay $NODEID trong EDS)
     */
    static DeviceProfile from_eds(const std::string& eds_path, uint8_t node_id);

    /**
     * @brief Dựng hồ sơ từ ObjectDictionary đã nạp sẵn
     *
     * Dùng khi thiết bị không có file EDS: truyền OD đã biết các object
     * CiA 402 là đủ để dò ra vai trò.
     */
    static DeviceProfile from_dictionary(const ObjectDictionary& od, uint8_t node_id);

    /** @brief Chỉ định lại object cho một vai trò (thiết bị lệch chuẩn) */
    void set_override(ObjectRole role, uint16_t index, uint8_t subindex);

    /** @brief Object đã phân giải cho vai trò, hoặc !valid nếu không tìm thấy */
    const ResolvedObject& resolve(ObjectRole role) const;

    /** @brief Vai trò này đã tìm thấy (tự dò hoặc override) chưa? */
    bool has(ObjectRole role) const { return resolve(role).valid; }

    /**
     * @brief Hồ sơ dùng được không — tức đã có đủ những gì cần thiết
     *        để điều khiển tốc độ bằng CiA 402.
     *
     * Cần: controlword, statusword, target_velocity, modes_of_operation.
     * actual_velocity không bắt buộc (thiếu thì phản hồi sẽ đọc qua SDO).
     */
    bool is_usable() const { return missing_required().empty(); }

    /** @brief Danh sách vai trò bắt buộc còn thiếu (rỗng = dùng được) */
    std::vector<ObjectRole> missing_required() const;

    /** @brief Tóm tắt dạng chữ để in ra log */
    std::string describe() const;

    uint8_t node_id() const { return node_id_; }
    const std::string& device_name() const { return device_name_; }
    const ObjectDictionary& dictionary() const { return od_; }

    /**
     * @brief Đơn vị của target velocity theo EDS
     *
     * EDS mô tả đơn vị ở mục [DeviceInfo] và các object có trường
     * "ParameterValue"/"Unit". Nếu không tìm thấy thì trả về
     * "unknown" và người dùng phải tự quy đổi.
     */
    const std::string& velocity_unit() const { return velocity_unit_; }
    void set_velocity_unit(const std::string& unit) { velocity_unit_ = unit; }

    /**
     * @brief Thứ tự byte cho thiết bị này
     *
     * Mặc định LittleEndian theo CiA 301. Thiết bị dùng big-endian thì gọi
     * set_byte_order(ByteOrder::BigEndian) — mọi giá trị đọc/ghi qua hồ sơ
     * sẽ tự đảo byte cho đúng.
     */
    ByteOrder byte_order() const { return byte_order_; }
    /**
     * @brief Đặt thứ tự byte cho thiết bị
     *
     * Áp dụng cho cả các object đã dò trước đó — thường gọi sau from_eds().
     */
    void set_byte_order(ByteOrder order);

    /** @brief Đọc tên byte order để log ("little"/"big") */
    const char* byte_order_name() const;

    /** @brief Số encoder count mỗi vòng (0 nếu thiết bị không khai báo) */
    uint32_t encoder_resolution() const { return encoder_resolution_; }
    void set_encoder_resolution(uint32_t r) { encoder_resolution_ = r; }

    /** @brief Đường dẫn EDS đã nạp (rỗng nếu dựng từ dictionary) */
    const std::string& eds_path() const { return eds_path_; }

private:
    /** Dò một object theo index chuẩn, ưu tiên subindex 0 */
    void probe_index(ObjectRole role, uint16_t index);

    /**
     * @brief Dò object theo tên trong EDS
     *
     * Một số hãng đặt object ở index khác chuẩn nhưng tên vẫn mang nghĩa
     * ("Target_velocity"). Dò theo tên giúp nhận được cả những thiết bị
     * đặt lệch.
     */
    void probe_by_name(ObjectRole role, const char* name_fragment);

    void apply_all_probes();

    ObjectDictionary od_;
    uint8_t node_id_{1};
    std::string eds_path_;
    std::string device_name_{"unknown"};
    std::string velocity_unit_{"unknown"};
    ByteOrder byte_order_{ByteOrder::LittleEndian};
    uint32_t encoder_resolution_{0};

    std::map<ObjectRole, ResolvedObject> objects_;
    ResolvedObject empty_;
};

} // namespace canopen

#endif // CANOPEN_DEVICE_DEVICE_PROFILE_HPP
