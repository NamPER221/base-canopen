# canopen — thư viện CANopen C++20

Thư viện giao tiếp CANopen cho robot, tách thành ba tầng độc lập:

```
libcanopen.a           GIAO THỨC — nói chuyện CANopen với mọi thiết bị
  ├─ SDO, NMT, PDO, EMCY, SYNC, Heartbeat, LSS
  ├─ EDS parser (CiA 306), Object Dictionary
  └─ BusInterface + SocketCAN

libcanopen_zlac.a       THIẾT BỊ — ZLAC8015D 2 trục (bao gồm mọi ngoại lệ của hãng)
libcanopen_kinematics.a ĐỘNG HỌC — robot 2 bánh, v/ω ↔ RPM, odometry
```

**Yêu cầu:** C++20, CMake ≥ 3.16, Linux (SocketCAN).

`libyaml-cpp` là **tuỳ chọn và chỉ dùng cho `examples/`** — bản thân thư viện không cần.
Thiếu nó thì ví dụ vẫn build được, chỉ dùng giá trị mặc định hardcode.

---

## Cài đặt

```bash
sudo apt install build-essential cmake

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build            # cài vào /usr/local
```

### Gói phát hành

```bash
./scripts/package.sh              # tạo dist/
```

| Gói | Dùng cho |
|---|---|
| `canopen-*-amd64.deb` | máy dev — `sudo dpkg -i canopen-0.2.0-amd64.deb` |
| `canopen-*-arm64.deb` | robot (cross-compile) |
| `canopen-*-arm64.tar.gz` | robot — giải nén là chạy, không cần root |
| `canopen-*-Source.tar.gz` | build ở bất kỳ máy nào |

Bản arm64 cần toolchain chéo:
```bash
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
./scripts/package.sh aarch64
```

### Dùng từ dự án khác

```cmake
find_package(canopen REQUIRED)
target_link_libraries(my_app PRIVATE canopen::canopen)
# tuỳ chọn:
target_link_libraries(my_app PRIVATE canopen::canopen_zlac)
target_link_libraries(my_app PRIVATE canopen::canopen_kinematics)
```

Hoặc bằng `pkg-config`:

```bash
g++ main.cpp $(pkg-config --cflags --libs canopen) -o app
```

### Thiết lập bus CAN

```bash
sudo modprobe can can_raw
sudo ip link set can0 up type can bitrate 500000
```

---

## Dùng nhanh

### Giao thức cơ bản

```cpp
#include <canopen/can/raw/socket_can_bus.hpp>
#include <canopen/can/msg/message_factory.hpp>

canopen::SocketCanBus bus("can0");
if (bus.open() < 0) { /* lỗi: ví dụ can0 chưa ip link set up */ }

bus.send(canopen::MessageFactory::create_nmt(1, canopen::NMTCommand::OPERATIONAL));

CANFrame req = canopen::MessageFactory::create_sdo_upload_request(1, 0x6041, 0);
bus.send(req);   // phản hồi về 0x581, xử lý qua bus.add_route(...)
```

### Thiết bị CiA 402 tuân thủ chuẩn — chỉ cần file EDS

```cpp
#include <canopen/device/device_profile.hpp>
#include <canopen/device/motor_device.hpp>

auto profile = canopen::DeviceProfile::from_eds("servo.eds", 1);
if (!profile.is_usable()) { /* profile.describe() cho biết thiếu gì */ }

canopen::MotorDevice motor(bus, profile);
motor.connect();                                  // NMT + enable CiA 402 tự động
motor.set_operation_mode(canopen::OperationMode::PROFILED_VELOCITY);
motor.set_profile_acceleration(800);
motor.set_velocity(500);                          // không cần biết index/kiểu dữ liệu
double v = motor.velocity();                       // TPDO nếu có, không thì SDO
motor.stop();
```

`DeviceProfile` tự đọc EDS để tìm `controlword`, `statusword`, `target_velocity`…,
kèm **kiểu dữ liệu** (16/32 bit, có dấu), quyền truy cập và **thứ tự byte**. Nó phân
biệt được device một trục (`0x60FF:00` là giá trị) với device nhiều trục
(`0x60FF:00` là `HighestSubIndex` header — như ZLAC).

### ZLAC8015D

```cpp
#include <canopen/drivers/zlac8015/zlac8015_driver.hpp>

canopen::drivers::ZLAC8015Driver motor(1, &bus);
motor.init();                     // NMT reset → start → enable → cấu hình PDO
motor.set_operation_mode(3);       // profile velocity
motor.set_profile(205, 500, 500);  // max_rpm, accel, decel
motor.set_velocity_rpm(55, 55);    // gửi qua RPDO, không chặn
```

### Moons' MBDV Dual-Axis (MBDV-2X-520AC)

Xem chi tiết tại: **[drivers/mbdv/README.md](drivers/mbdv/README.md)**

```bash
# Điều khiển bằng bàn phím (giống ROS2 teleop_twist_keyboard, 200 Hz):
sudo ./build/examples/mbdv_teleop_keyboard can0 -1 1 -2 2

# Điều khiển vi sai tự động + Odometry:
sudo ./build/examples/mbdv_diff_drive can0 -1 1 -2 2 -v 0.3 -w 0.0 -t 3.0
```

---

## Cấu hình PDO của ZLAC8015D (đã kiểm chứng trên phần cứng)

| Thông số | Giá trị |
|---|---|
| Mapping `0x1600:01` | `0x60FF0320` — 1 entry, `0x60FF:03` 32-bit gộp 2 trục |
| COB-ID `0x1400:01` | `0x200 + node` (node 1 → `0x201`) |
| Transmission type | 254–255 |
| **DLC** | **4 — phải khớp chính xác, KHÔNG đệm** |
| TPDO `0x1A00` | `0x606C0120` + `0x606C0220`, timer 100 ms |

> Firmware ZLAC **hủy frame nếu DLC ≠ tổng byte của mapping**. Gửi DLC=8 (đệm 4 byte 0)
> sẽ bị bỏ qua hoàn toàn, dù mapping verify đúng.

---

## Ngoại lệ thiết bị đã xử lý

ZLAC8015D không tuân thủ CiA 301 ở nhiều chỗ. Thư viện xử lý sẵn, và các bài học này
**áp dụng được cho nhiều drive Trung Quốc**:

| Vấn đề | Xử lý trong thư viện |
|---|---|
| SDO response dùng encoding cũ (`0x43` thay vì `0x4C`) | Parser nhận cả hai chế độ |
| SDO **request** cũng phải dùng encoding cũ | `SdoEncoding::Legacy` (mặc định `Standard`) |
| EDS khai sai kích thước object (`0x6040` ghi 16-bit, thực tế 4 byte) | `DeviceProfile::set_override(role, index, sub, size)` |
| Confirm `0x60` không echo index/subindex | Khớp response theo **loại giao dịch** |
| Không có object `0x1019` (bắt buộc theo CiA 301) | `connect()` không phụ thuộc object đó |
| `0x60FF` ở subindex 3 thay vì 0 (vi phạm CiA 402) | Ghi đè qua `set_override()` |

---

## Tài liệu

- **[docs/API.md](docs/API.md)** — hướng dẫn dùng từng nhóm hàm, có ví dụ gọi thật
- **Doxygen** — tra cứu chữ ký đầy đủ:
  ```bash
  ./docs/generate_docs.sh        # → docs/html/index.html
  ```
- **[CHANGELOG.md](CHANGELOG.md)** — lịch sử thay đổi

Tài liệu được kiểm tra tự động bởi `test_docs`: mọi hàm ghi trong `docs/API.md`
phải tồn tại trong header, nên tài liệu không thể lệch khỏi code.

## Kiểm thử

```bash
ctest --test-dir build --output-on-failure     # 11/11 unit test, không cần phần cứng
./scripts/test_install.sh                       # cài rồi dùng từ dự án bên ngoài
```

Test đơn vị dùng `FakeBus` nên chạy được trên máy không có CAN.

Test phần cứng (cần robot thật):

```bash
sudo ./examples/pdo_test                       # xác minh RPDO (đọc actual qua SDO)
sudo ./examples/can_probe                      # quét node 1-127
sudo ./examples/dual_motor_keyboard            # teleop 2 động cơ + tự dừng mất kết nối
sudo ./examples/robot_move                     # chạy theo quỹ đạo
sudo ./examples/motor_device_demo can0 1 ZLAC8015D.eds   # tầng generic

# Ghi đè cấu hình (mặc định đã trỏ sẵn tới config/zlac8015_config.yaml)
sudo ./examples/dual_motor_keyboard --config my_robot.yaml
```

---

## Cấu hình

Mặc định đọc `config/zlac8015_config.yaml` (CAN interface, node ID, thông số động cơ,
động học, phím tắt, ngưỡng an toàn). Xem `config/zlac8015_config.yaml` để biết đầy đủ
các khoá.

Tham số quan trọng khi hiệu chuẩn:

```yaml
motor:
  wheel_radius_m: 0.0865     # bánh Ø173mm
  encoder_resolution: 4096   # phải khớp encoder thật
  max_rpm: 205               # Max speed trong datasheet — hàng rào an toàn
keyboard:
  hold_timeout_ms: 60        # TRỄ DỪNG khi nhả phím; phải > chu kỳ lặp terminal
  first_press_grace_ms: 700  # che độ trễ auto-repeat đầu tiên của terminal
```

---

## Trạng thái

| Hạng mục | Trạng thái |
|---|---|
| Giao thức lõi (SDO/NMT/PDO/EMCY/SYNC/HB/LSS) | ✅ |
| EDS parser (CiA 306) | ✅ |
| Tầng generic CiA 402 (`MotorDevice` + `DeviceProfile`) | ✅ 30/30 test với drive ảo chuẩn |
| Driver ZLAC8015D (RPDO/TPDO, tự dừng, tự kết nối lại) | ✅ kiểm chứng trên phần cứng |
| Đóng gói CMake / pkg-config | ✅ test bằng dự án bên ngoài |
| Unit test + CI (4 job: compiler, sanitizer, install, ví dụ) | ✅ 11/11 |
| Đóng gói: .deb + tar.gz + source, cho amd64 và arm64 | ✅ |
| Tài liệu API + Doxygen | ✅ có test chống lệch |
| Nhiều thiết bị (driver thứ 2) | ⚠️ chưa có — kiến trúc đã sẵn sàng |

**Đã biết:** `MotorDevice` dùng được với thiết bị tuân thủ CiA 402. ZLAC8015D vi
phạm chuẩn ở nhiều chỗ nên cần `ZLAC8015Driver` — đây là chủ ý tách lớp, không
phải hạn chế. Xem bảng ngoại lệ ở trên.

---

## Cấu trúc mã nguồn

```
include/canopen/     can/ co/ device/ ev/ kinematics/ recovery/ drivers/
src/                 (mirror của include)
test/                unit test (FakeBus) + consumer (kiểm tra đóng gói)
examples/            chương trình mẫu
scripts/             test_install.sh, sync_to_robot.sh
cmake/               canopenConfig.cmake.in
```

Lõi (`canopen/can`, `canopen/co`) **không chứa mã vendor** — chỉ có comment giải
thích. Toàn bộ ngoại lệ của từng hãng nằm trong `canopen/drivers/`.
