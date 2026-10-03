# Hướng dẫn sử dụng API

Tài liệu này liệt kê **hàm có thật trong header**, theo thứ tự từ thấp lên cao.
Muốn tra chi tiết tham số thì xem comment Doxygen ngay trong header
(`doxygen/`), hoặc bấm vào tên hàm.

Ba tầng, dùng độc lập:

| Tầng | Header chính | Dùng khi |
|---|---|---|
| Giao thức | `canopen/can/*`, `canopen/co/*` | Cần nói chuyện CANopen thô |
| Thiết bị | `canopen/device/*` | Drive CiA 402 tuân thủ chuẩn |
| Driver hãng | `canopen/drivers/zlac8015/*` | ZLAC8015D |

---

## 0. Điểm khởi đầu: bus

Mọi thứ đều đi qua một `BusInterface`.

```cpp
#include <canopen/can/raw/socket_can_bus.hpp>

canopen::SocketCanBus bus("can0");
// open() trả int, KHÔNG phải bool: 0 = thành công, âm = lỗi.
// Viết `if (!bus.open())` là sai — sẽ báo lỗi ngay cả khi mở thành công.
if (bus.open() < 0 || !bus.is_up()) { /* xem lỗi hệ thống */ }
```

| Hàm | Viết gì |
|---|---|
| `int open()` | mở socket. **Trả `int`, không phải bool**: `0` = thành công, âm = lỗi |
| `void close()` | đóng socket |
| `bool send(const CANFrame&)` | gửi 1 frame; **trả false nếu bus chưa mở** |
| `bool is_up() const` | bus còn dùng được không (kiểm tra cả trạng thái `BUS_OFF`) |
| `add_route(can_id, mask, handler)` | đăng ký nhận theo COB-ID, trả `RouteHandle` (0 = lỗi) |
| `add_route_all(handler)` | nhận mọi frame |
| `remove_route(RouteHandle)` | huỷ đăng ký |

`callback` nhận `const CANFrame&`, được gọi trên thread của bus. **Không được
block lâu trong callback** — sẽ làm nghẽn nhận.

```cpp
bus.add_route(0x701, 0x7FF, [](const canopen::CANFrame& f) {
    std::printf("state 0x%02X\n", f.get_u8(0));
});
```

Dùng `FakeBus` thay `SocketCanBus` để chạy logic trên máy không có CAN:

```cpp
canopen::FakeBus bus;        // định nghĩa trong test, không cài cùng thư viện
```

---

## 1. CANFrame

```cpp
canopen::CANFrame f;                    // rỗng, coi như CAN đếm 0
f.set_id(0x601);                        // COB-ID (11 hoặc 29 bit)
f.set_len(8);
f.set_u8(0, 0x40);
```

| Hàm | Công dụng |
|---|---|
| `id()` / `set_id()` | COB-ID |
| `len()` / `set_len()` | độ dài 0–8 (giới hạn cứng 64 nội bộ) |
| `get_u8(i)` / `set_u8(i, v)` | 1 byte — không cần chọn byte order |
| `get_u16_le(i)` / `set_u16_le(i, v)` | 2 byte little-endian |
| `get_u16_be(i)` / `set_u16_be(i, v)` | 2 byte big-endian |
| `get_u32_le(i)` / `set_u32_le(i, v)` | 4 byte little-endian |
| `get_u32_be(i)` / `set_u32_be(i, v)` | 4 byte big-endian |
| `get_u64_le(i)` | 8 byte little-endian (chỉ đọc) |
| `data()` | con trỏ `uint8_t[8]`, dùng cho SDO expedited |
| `is_rtr()` / `is_extended()` | cờ frame |

**Không có `get_u16` trung tính — phải chọn `_le` hay `_be`.** Đây là chủ ý:
ZLAC đọc một số object theo big-endian trong khi CiA 301 quy định
little-endian. Hàm trung tính sẽ khiến người dùng phải tự đoán, và khi đoán sai
thì giá trị sai vẫn "chạy được" — kiểu lỗi âm thầm khó tìm nhất.

Khi dùng ở tầng cao (`DeviceProfile`, `MotorDevice`), byte order do
`profile.set_byte_order()` quyết định, bạn không cần tự chọn `_le`/`_be`.

---

## 2. MessageFactory — dựng frame, không cần biết byte layout

```cpp
#include <canopen/can/msg/message_factory.hpp>
using canopen::MessageFactory;
```

| Hàm | Tạo frame |
|---|---|
| `create_nmt(uint8_t node, NMTCommand cmd)` | `0x000` |
| `create_heartbeat(NMTState state)` | phát heartbeat của chính master (0x700) |
| `create_heartbeat(node, NMTState state)` | dựng frame heartbeat của node `0x700+node` |
| `create_sdo_upload_request(node, index, sub)` | `0x600+node` |
| `create_sdo_download_request(node, index, sub, data, size, encoding)` | `0x600+node` |
| `create_pdo_tx(node, pdo_num, data, len)` | frame TPDO truyền từ slave (`0x180+node`) |
| `create_pdo_rx(node, pdo_num, data, len)` | frame RPDO truyền tới slave (`0x200+node`) |
| `create_sdo_download_response(node, index, sub, data, size)` | dựng SDO response khi làm **server** |
| `create_sdo_upload_response(node, index, sub, data, size)` | dựng SDO response khi làm server |
| `create_sdo_abort(node, index, sub, abort_code)` | trả lỗi SDO |
| `create_lss_inquiry(cmd, data)` / `create_lss_response(data, len)` | LSS |
| `create_sync()` | `0x080` |
| `create_emcy(node, code, ...)` | `0x080+node` |

`NMTCommand`: `OPERATIONAL(0x01)`, `STOP(0x02)`, `PREOPERATIONAL(0x80)`,
`RESET_NODE(0x81)`, `RESET_COMMUNICATION(0x82)`.

> Lưu ý thứ tự tham số: `create_nmt(node, cmd)` — **node trước, command sau**.

---

## 3. SDOClient — đọc/ghi object dictionary

Đây là API quan trọng nhất khi làm việc với drive.

```cpp
canopen::SDOClient sdo(&bus, /*node=*/1);
sdo.set_timeout(200);                    // ms
sdo.set_encoding(canopen::SdoEncoding::Standard);
```

### Đồng bộ (chặn, trả về mã lỗi)

```cpp
int32_t mode;
auto err = sdo.upload(0x6060, 0, mode);          // template, tự đoán kiểu
if (err == canopen::SDOError::OK) { /* dùng mode */ }

uint16_t sw;
err = sdo.download(0x6040, 0, sw);
```

| Hàm | Trả về |
|---|---|
| `upload<T>(index, sub, T& out)` | `SDOError` — **kích thước `T` phải khớp object** |
| `download<T>(index, sub, const T& in)` | `SDOError` |
| `upload_sync(index, sub, void* buf, size_t& size)` | cho buffer tự quản lý kích thước |
| `download_sync(index, sub, const void* buf, size_t size)` | kích thước tường minh |
| `download_nowait(index, sub, data, size)` | `true` nếu **đã gửi được**, không chờ |
| `abort(code)` | huỷ giao dịch đang chờ |

`SDOError` gồm `OK`, `TIMEOUT`, `ABORT`, `BUSY`, `INVALID_ARGUMENT`, `NOT_SUPPORTED`.
Chi tiết lý do abort: `last_abort_code()`.

### Bất đồng bộ (không chặn)

```cpp
sdo.upload(0x606C, 0, [](SDOError e, const void* data, size_t len) {
    if (e == SDOError::OK) { /* ... */ }
});
```

Dùng khi không được phép dừng luồng điều khiển.

### Ghi không chặn — quan trọng cho điều khiển robot

```cpp
sdo.download_nowait(0x6041, 0, &cw, 2);
```

Gửi đi rồi trả về ngay, không đợi drive xác nhận. **Đây là kỹ thuật giữ
chu kỳ điều khiển ổn định**: nếu mỗi lệnh đều chờ SDO response thì vòng lặp
điều khiển sẽ bị gián đoạn mỗi lần drive chậm trả lời.

### Chế độ encoding

| Chế độ | Byte command | Dùng cho |
|---|---|---|
| `SdoEncoding::Standard` | `0x2C`/`0x2E` | CiA 301 — **mặc định** |
| `SdoEncoding::Legacy` | `0x23`/`0x2B` | ZLAC, nhiều drive Trung Quốc |

Parser nhận cả hai dạng response (`0x4C`/`0x43`) bất kể request gửi kiểu nào,
nên chỉ cần set đúng khi **gửi**.

---

## 4. CiA402Drive — điều khiển một trục theo CiA 402

```cpp
canopen::CiA402Drive drive(1, &bus);
drive.init();                                    // SW=0x06 → 0x07
drive.set_state(canopen::CiA402State::OPERATION_ENABLED);
```

| Nhóm | Hàm |
|---|---|
| Chu kỳ khởi tạo | `init()`, `start()`, `stop()`, `shutdown()` |
| Chuyển trạng thái | `set_state(CiA402State)`, `quick_stop()`, `fault_reset()` |
| Đọc trạng thái | `get_state()`, `get_statusword()`, `is_enabled()`, `has_fault()` |
| Controlword | `get_controlword()`, `set_controlword(uint16_t)` |
| Chế độ vận hành | `set_operation_mode(OperationMode)`, `get_operation_mode()` |
| Vị trí | `set_target_position()`, `start_position_move()`, `start_relative_move()`, `halt_position()`, `immediate_position()`, `set_position_profile()` |
| Tốc độ | `set_target_velocity()`, `set_velocity_profile()` |
| Theo dõi lỗi | `on_fault` — callback `void(uint16_t error_code)`, gán để nhận thông báo lỗi |

`CiA402State`: `NOT_READY_TO_SWITCH_ON` → `SWITCH_ON_DISABLED` →
`READY_TO_SWITCH_ON` → `SWITCH_ON` → `OPERATION_ENABLED`, cộng `FAULT`,
`QUICK_STOP_ACTIVE`, `SWITCH_ON_DISABLED`…

---

## 5. DeviceProfile — đọc EDS, tự tìm object

```cpp
auto profile = canopen::DeviceProfile::from_eds("servo.eds", 1);
if (!profile.is_usable()) {
    std::cerr << profile.describe();   // liệt kê còn thiếu gì
    return 1;
}
```

| Hàm | Công dụng |
|---|---|
| `from_eds(path, node)` | nạp EDS (CiA 306) |
| `from_dictionary(od, node)` | dựng từ Object Dictionary có sẵn |
| `is_usable()` | đủ object bắt buộc để điều khiển |
| `describe()` | mô tả bằng chữ, dùng khi `is_usable()` false |
| `missing_required()` | danh sách vai trò còn thiếu |
| `resolve(ObjectRole)` | ra index/sub/kích thước/kiểu của một vai trò |
| `has(ObjectRole)` | vai trò đó có dùng được không |
| `eds_has(index, sub)` | object có trong EDS không |
| `device_name()` | tên thiết bị từ EDS |
| `dictionary()` | `const ObjectDictionary&` |
| `set_override(role, index, sub, size)` | **ép lại khi EDS sai** |
| `set_byte_order(ByteOrder)` | khi drive dùng big-endian |
| `set_velocity_unit(std::string)` | `"rpm"`, `"mm/s"`… |
| `encoder_resolution()` | số xung encoder |

`ObjectRole`: `CONTROLWORD`, `STATUSWORD`, `OPERATION_MODE`, `OPERATION_MODE_DISPLAY`,
`TARGET_VELOCITY`, `ACTUAL_VELOCITY`, `PROFILE_ACCEL`, `PROFILE_DECEL`,
`TARGET_POSITION`, `ACTUAL_POSITION`, `FAULT_CODE`, `ERROR_CODE`…

`set_override` là chỗ xử lý **EDS nói dối**. Ví dụ ZLAC khai `0x6040` là 16-bit
trong khi thực tế 4 byte:

```cpp
profile.set_override(ObjectRole::CONTROLWORD, 0x6040, 0, /*size=*/4);
```

---

## 6. MotorDevice — lớp generic dùng cho drive CiA 402 chuẩn

```cpp
canopen::MotorDevice motor(bus, profile);   // bus truyền THAM CHIẾU
motor.set_sdo_timeout(200);

if (!motor.connect(3000)) { /* thất bại */ }    // NMT + bật drive tự động
motor.set_operation_mode(canopen::OperationMode::PROFILED_VELOCITY);
motor.set_profile_acceleration(800);
motor.set_velocity(55.0);
```

### Kết nối

| Hàm | Công dụng |
|---|---|
| `connect(timeout_ms=3000)` | NMT start → chờ operational → set controlword |
| `disconnect()` | dừng và ngắt |
| `reconnect(timeout_ms=3000)` | gọi lại `connect()` |
| `is_connected()` / `is_usable()` | trạng thái |
| `set_nmt_reset_on_connect(bool)` | có gửi `0x82` khi connect không (mặc định `true`) |

> `set_nmt_reset_on_connect(true)` sẽ **xoá cấu hình PDO** của drive. Nếu drive
> đã được cấu hình PDO xong và không cần reset thì tắt đi, nếu không phải
> cấu hình lại.

### Điều khiển

| Hàm | Công dụng |
|---|---|
| `set_operation_mode(mode, attempts=3)` | đổi mode, thử lại nếu drive chưa sẵn sàng |
| `operation_mode()` | mode hiện tại |
| `set_profile_velocity/acceleration/deceleration(v)` | hồ sơ chuyển động |
| `set_velocity(v)` | **đơn vị theo `profile.velocity_unit()`** |
| `velocity()` | tốc độ thực (TPDO nếu có, không thì SDO) |
| `stop()` | dừng |
| `read_role(role, double& out)` | đọc object theo vai trò |
| `write_role(role, double v)` | ghi object theo vai trò |
| `read_value(index, sub, double& out)` | đọc object tuỳ ý |
| `write_value(index, sub, double v)` | ghi object tuỳ ý |
| `write_controlword(uint16_t)` | ghi controlword thô |

### Theo dõi

| Hàm | Công dụng |
|---|---|
| `cia402_state()` | trạng thái CiA 402 |
| `is_operational()` | đang `OPERATION_ENABLED` |
| `statusword()` | đọc statusword |
| `error_code()` | mã lỗi |
| `sdo()` | truy cập `SDOClient` để làm việc tự do |
| `set_sdo_gap_ms(ms)` | nghỉ tối thiểu giữa 2 SDO, tránh dồn ép drive |

**Lưu ý:** `MotorDevice` chưa từng chạy trên drive phần cứng thật — mới được
kiểm chứng bằng drive ảo trong unit test. Nếu gặp lỗi, đó là chỗ cần debug đầu
tiên.

---

## 7. DifferentialDriveKinematics — động học robot 2 bánh

```cpp
canopen::kinematics::DifferentialDriveKinematics kin(0.0865, 0.400, 205.0);
```

Tham số: bán kính bánh (m), chiều dài cơ sở (m), RPM tối đa.

### Chuyển đổi

| Hàm | Trả về |
|---|---|
| `velocity_to_rpm(v, omega)` | `WheelRPM{int16_t left, right}` — **đã clamp về ±max_rpm** |
| `velocity_to_wheels(v, omega)` | vận tốc tuyến tính mỗi bánh (m/s) |
| `linear_to_rpm(v, radius)` *(static)* | 1 v → RPM |
| `rpm_to_linear(rpm, radius)` *(static)* | RPM → 1 v |
| `update(dp_left, dp_right, dt)` | cập nhật odometry từ góc quay bánh |
| `get_pose()` | `Pose{x, y, theta}` |
| `get_body_velocity()` | `BodyVelocity{linear, angular}` |
| `reset_pose(x, y, theta)` / `reset_encoders()` | đặt lại gốc toạ độ |

Quy ước: `v_L = v - omega·L/2`, `v_R = v + omega·L/2`. Vậy `omega > 0` là
**quay trái** (bánh phải tiến, bánh trái lùi).

`WheelRPM` là `int16_t` — giá trị bị **cắt bỏ phần thập phân** khi lưu, nên khi
so sánh phải chấp nhận sai số tối đa 0.5.

---

## 8. ZLAC8015Driver — driver riêng cho ZLAC8015D

Dùng khi thiết bị **không** tuân thủ CiA 301 (xem README, mục ngoại lệ).

```cpp
canopen::drivers::ZLAC8015Driver motor(1, &bus);   // ctor: (node_id, bus* = nullptr)
motor.set_sdo_timeout(200);
motor.init();                    // reset → start → enable → cấu hình PDO
```

### Khởi tạo và vận hành

| Hàm | Công dụng |
|---|---|
| `init(timeout_ms=3000)` | chuỗi khởi tạo đầy đủ |
| `enable()` / `disable()` | bật/tắt drive |
| `quick_stop()` / `fault_reset()` | dừng khẩn cấp / xoá lỗi |
| `set_operation_mode(int8_t mode)` | **số nguyên**, theo tài liệu ZLAC (3 = profile velocity) |
| `set_profile(vel, accel, decel)` | hồ sơ |
| `setup_pdo(save_to_eeprom=false)` | cấu hình RPDO + TPDO |
| `pdo_ready()` | đã cấu hình PDO xong |
| `use_pdo(bool)` | bật/tắt dùng PDO |

### Điều khiển

| Hàm | Công dụng |
|---|---|
| `set_velocity_rpm(int16_t left, int16_t right)` | gửi qua RPDO nếu bật, không chặn |
| `stop()` | tương đương `set_velocity_rpm(0,0)` |
| `get_velocity_left/right()` | đọc actual |
| `get_position_left/right()` | đọc vị trí (encoder) |
| `read_status()` | đọc statusword |
| `update()` | gọi mỗi vòng lặp để xử lý frame TPDO |

### Đọc trạng thái

| Hàm | Công dụng |
|---|---|
| `velocity_actual_left/right()` | actual từ TPDO |
| `statusword_pdo()` | statusword từ TPDO |
| `tpdo_received()` / `tpdo_frame_count()` | đã nhận TPDO chưa, đã nhận bao nhiêu frame |
| `nmt_state()` / `is_operational()` | trạng thái NMT |
| `is_enabled()` / `has_fault()` / `get_state()` | trạng thái drive |
| `is_online()` / `ms_since_heartbeat()` / `offline_count()` | sức khoẻ kết nối |
| `rpdo_sent()` / `sdo_sent()` | **đếm số lệnh đã gửi theo từng kênh** |

`rpdo_sent()` / `sdo_sent()` rất hữu ích khi chẩn đoán: nếu robot không chạy mà
`rpdo_sent()` vẫn tăng thì vấn đề ở phía drive, còn nếu cả hai đều đứng yên
thì vòng lặp điều khiển chưa tới được chỗ gửi.

### Kết nối lại

| Hàm | Công dụng |
|---|---|
| `check_health()` | true nếu còn sống; false thì đã dừng motor |
| `reconnect()` | chạy lại toàn bộ `init()` |
| `set_heartbeat_timeout_ms(ms)` | ngưỡng coi là mất kết nối |
| `set_heartbeat_producer_ms(ms)` | nhờ drive phát heartbeat bao lâu một lần |
| `set_velocity_readback_divisor(d)` | hệ số chia khi đọc actual |

### Ghi chú quan trọng

- `0x82` (Reset Communication) **xoá cấu hình PDO**. `reconnect()` đã chạy lại
  `setup_pdo()` nên không sao, nhưng nếu bạn tự gửi `0x82` thì phải gọi lại
  `setup_pdo()`.
- DLC của RPDO phải **đúng 4 byte** cho mapping `0x60FF:03`. Gửi DLC=8 (đệm)
  sẽ bị drive bỏ qua hoàn toàn.
- Mapping 1 trục (`0x60FF:00`) **không** hoạt động trên firmware này.

---

## 9. NMTService

Dành cho master nhiều node — thư viện hiện dùng nội bộ trong driver, nhưng có
thể dùng trực tiếp.

| Hàm | Công dụng |
|---|---|
| `start()` / `stop()` | bật/tắt cơ chế heartbeat |
| `send_command(node, cmd)` | gửi lệnh NMT |
| `start_heartbeat_producer(interval_ms)` | bắt đầu phát heartbeat cho master |
| `add_heartbeat_consumer(node, producer_time_ms)` | theo dõi 1 node |
| `remove_heartbeat_consumer(node)` | bỏ theo dõi |
| `set_heartbeat_timeout(node, timeout_ms)` | ngưỡng timeout từng node |
| `get_node_state(node)` | `NodeStateInfo{state, last_heartbeat, bootup_received, heartbeat_timeout, consecutive_timeouts, heartbeat_timeout_ms}` |
| `get_all_node_states()` | trạng thái mọi node |

---

## 10. Thứ tự khởi động chuẩn

Sai thứ tự là nguyên nhân phổ biến nhất khiến drive "không phản hồi":

```cpp
// 1. bus
canopen::SocketCanBus bus("can0");
bus.open();

// 2. dò thiết bị (không bắt buộc nhưng nên có khi lắp nhiều drive)
//    examples/can_probe

// 3. tạo driver, gắn bus
canopen::drivers::ZLAC8015Driver motor(1, &bus);   // ctor: (node_id, bus* = nullptr)
motor.set_sdo_timeout(200);

// 4. init — tự làm reset → start → enable → PDO
if (!motor.init()) { /* dừng, kiểm tra can0 và node id */ }

// 5. cấu hình tham số
motor.set_profile(205, 500, 500);
motor.set_operation_mode(3);

// 6. vòng lặp điều khiển
while (running) {
    if (!motor.check_health()) {
        motor.reconnect();
        continue;
    }
    motor.set_velocity_rpm(cmd_left, cmd_right);
    motor.update();               // xử lý TPDO
    sleep_for(5ms);
}
```

Với drive CiA 402 chuẩn, thay bước 3–6 bằng `MotorDevice` và bỏ qua `setup_pdo`.
