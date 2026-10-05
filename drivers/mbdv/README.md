# Hướng dẫn Điều khiển Động cơ Moons' MBDV Dual-Axis qua CANopen

Tài liệu hướng dẫn cấu hình phần cứng, biên dịch và chạy điều khiển động cơ Servo 2 trục **Moons' MBDV (MBDV-2X-520AC)** sử dụng giao thức **CANopen (CiA 402)** kết hợp động học vi sai (**Differential Drive Kinematics**).

---

## 1. Thiết lập CAN Bus (SocketCAN)

### Cắm phần cứng
* Kết nối cáp CAN (CAN_H, CAN_L, GND) giữa máy tính (USB-CAN adapter) và driver MBDv.
* Đảm bảo nguồn driver đã bật và trở đầu cuối 120Ω đã được kết nối đúng.
* Cài đặt Node ID bằng DIP switch trên driver:
  * **Trục 1 (Trái)**: Mặc định Node ID = `1`
  * **Trục 2 (Phải)**: Mặc định Node ID = `2`

### Khởi tạo interface `can0`
Chạy lệnh khởi động interface SocketCAN với tốc độ baudrate **500 kbps**:

```bash
sudo modprobe can can_raw
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

Kiểm tra trạng thái interface:
```bash
ip link show can0
```

Kiểm tra driver có đang trực tuyến không (lắng nghe Heartbeat `0x701`, `0x702`):
```bash
candump can0,701:7FF,702:7FF -n 2
```

---

## 2. Biên dịch Dự án (Build)

```bash
# Tạo thư mục build và cấu hình CMake
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_MBDV_DRIVER=ON

# Biên dịch toàn bộ
cmake --build build -j$(nproc)
```

Các file thực thi sẽ được tạo trong `build/examples/`:
* `mbdv_teleop_keyboard`: Điều khiển bàn phím thời gian thực (giống ROS2 teleop keyboard).
* `mbdv_diff_drive`: Điều khiển theo vận tốc đặt trước và đo Odometry.

---

## 3. Cách chạy Điều khiển

### Cách 1: Điều khiển Bàn phím (`mbdv_teleop_keyboard`) - KHUYÊN DÙNG

Chương trình này hoạt động tương tự như gói `teleop_twist_keyboard` của ROS2, tích hợp sẵn **bộ đo tần số thời gian thực (Real-time Hz meter)**:

```bash
# Chạy mặc định ở tần số 200 Hz
sudo ./build/examples/mbdv_teleop_keyboard can0 -1 1 -2 2
```

#### Sơ đồ bàn phím:

```
        u    i    o          Tiến-trái  |   Tiến   |  Tiến-phải
        j    k    l          Quay-trái  |   Dừng   |  Quay-phải
        m    ,    .          Lùi-trái   |   Lùi    |  Lùi-phải

   Phím mũi tên (↑, ↓, ←, →) cũng hoạt động!

   [k] hoặc [SPACE] : Dừng xe (v=0, w=0)
   [q] / [z]        : Tăng / giảm tốc độ tiến-lùi (±0.1 m/s)
   [w] / [x]        : Tăng / giảm tốc độ quay góc (±0.2 rad/s)
   [e]              : DỪNG KHẨN CẤP (Quick Stop)
   [r]              : Reset Odometry về (0, 0, 0)
   [t]              : In bảng Telemetry chi tiết (điện áp, nhiệt độ, mã lỗi...)
   [ESC] / [Ctrl+C] : Thoát chương trình an toàn
```

#### Thanh trạng thái thời gian thực:
```text
v= 0.200 w= 0.000 | odom: x= 0.251 y= 0.000 θ= 0.0° | FB: 200Hz [CMD: 200Hz]
```
* **`FB: 200Hz`**: Tần số nhận phản hồi vị trí/vận tốc thực tế từ động cơ về máy tính.
* **`CMD: 200Hz`**: Tần số phát lệnh điều khiển RPDO3 xuống động cơ.

#### Chạy với tần số cao hơn (lên đến 400 Hz):
```bash
sudo ./build/examples/mbdv_teleop_keyboard can0 -1 1 -2 2 --rate 400
```

---

### Cách 2: Chạy theo Kịch bản / Vận tốc cố định (`mbdv_diff_drive`)

Dùng để kiểm thử phần cứng hoặc chạy thử nghiệm quãng đường cố định:

```bash
# 1. Chạy thẳng với vận tốc 0.3 m/s trong 3 giây
sudo ./build/examples/mbdv_diff_drive can0 -1 1 -2 2 -v 0.3 -w 0.0 -t 3.0

# 2. Quay tại chỗ với vận tốc 1.0 rad/s trong 3 giây
sudo ./build/examples/mbdv_diff_drive can0 -1 1 -2 2 -v 0.0 -w 1.0 -t 3.0

# 3. Đi theo đường cong (cung tròn)
sudo ./build/examples/mbdv_diff_drive can0 -1 1 -2 2 -v 0.2 -w 0.5 -t 5.0

# 4. Chỉ giám sát dữ liệu và trạng thái (không quay bánh xe)
sudo ./build/examples/mbdv_diff_drive can0 -1 1 -2 2 --monitor

# 5. Chế độ chỉ điều khiển 1 trục (Trục 1)
sudo ./build/examples/mbdv_diff_drive can0 -1 1 --single-axis -v 0.2 -t 3.0
```

---

## 4. Tham số Động học & Hiệu chuẩn (Kinematics Config)

Nếu robot đi lệch quãng đường hoặc tính toán Odometry chưa khớp kích thước cơ khí thật, có thể cấu hình qua tham số dòng lệnh:

| Tham số | Ý nghĩa | Mặc định | Mô tả |
|---|---|---|---|
| `-r, --radius` | Bán kính bánh xe ($r$) | `0.07333` m | Đo từ tâm trục tới mép lốp ngoài |
| `-l, --track` | Khoảng cách 2 bánh ($L$) | `0.4544` m | Khoảng cách giữa 2 điểm tiếp xúc của lốp |
| `-c, --cpr` | Encoder CPR | `10000` | Số xung/vòng (thanh ghi `P3-05` / `0x2A90`) |
| `--invert-right` | Đảo chiều bánh phải | `true` | Chiều quay vi sai 2 bánh đối xứng |
| `--no-invert-right`| Không đảo chiều bánh phải | `false` | Áp dụng khi 2 động cơ quay cùng hướng |
| `--max-v` | Giới hạn vận tốc thẳng | `2.0` m/s | Hàng rào an toàn cho xe |
| `--max-w` | Giới hạn vận tốc góc | `4.0` rad/s | Hàng rào an toàn xoay |

Ví dụ chạy với thông số robot tùy chỉnh:
```bash
sudo ./build/examples/mbdv_teleop_keyboard can0 -1 1 -2 2 -r 0.085 -l 0.480 -c 10000
```

---

## 5. Kiến trúc Truyền thông CANopen (PDO & SYNC)

Hệ thống được thiết kế tối ưu hóa độ trễ thấp theo chuẩn **CiA 402**:

```
PC Master                                    Moons' MBDV Driver
   │                                                 │
   ├────────── SYNC (0x080, chu kỳ 5ms) ────────────►│ (Đồng bộ nhịp)
   │                                                 │
   ├────────── RPDO3 (0x401 / 0x402) ───────────────►│ (Lệnh Controlword + Velocity)
   │                                                 │
   │◄───────── TPDO4 (0x481 / 0x482) ────────────────┤ (Phản hồi Position + Velocity)
   │                                                 │
   ▼                                                 ▼
[Odometry 200 Hz]                             [Motor Execution]
```

* **Lệnh lái (PC $\rightarrow$ Drive)**: Dùng **RPDO3** (`0x400 + Node ID`, DLC = 6 byte gồm Controlword 16-bit và Target Velocity 32-bit). Gửi không chặn (non-blocking) với tần số 200–400 Hz.
* **Đồng bộ**: Máy tính phát frame **SYNC** (`0x080`) chu kỳ **5 ms (200 Hz)**.
* **Phản hồi (Drive $\rightarrow$ PC)**: Driver phát **TPDO4** (`0x480 + Node ID`) đồng bộ theo nhịp SYNC, loại bỏ hoàn toàn độ trễ của cơ chế hỏi-đáp SDO.
* **Xử lý đặc thù Moons' MBDv**:
  * Chế độ hoạt động: Tự động ghi tham số nội bộ `P1-00 (0x2A30) = 15` khi chọn Profile Velocity.
  * Mở khóa bit 31 của COB-ID (`0x1803:01`) và kích hoạt Transmission Type = 1 trong Pre-Operational để đạt tốc độ phản hồi 200 Hz.
  * Tự động xử lý độ trễ nhả phanh điện từ (250 ms) khi Enable.

---

## 6. Xử lý Sự cố Thường gặp (Troubleshooting)

### 1. `Error: CAN bus is not up!`
* **Hiện tượng**: Báo bus chưa bật dù cáp đã cắm.
* **Nguyên nhân**: Adapter USB-CAN (như canable, gs_usb) thường không hỗ trợ phần cứng dò sóng mang (carrier detect), kernel báo trạng thái `NO-CARRIER`.
* **Khắc phục**: Thư viện đã được cập nhật để chấp nhận trạng thái `IFF_UP`. Bạn chỉ cần đảm bảo đã chạy `sudo ip link set can0 up type can bitrate 500000`.

### 2. Động cơ báo lỗi / Không nhận lệnh (`FAULT`)
* Nhấn phím `t` trong màn hình teleop để xem mã cảnh báo `dsp_alarm_code`.
* Nhấn phím bất kỳ di chuyển hoặc gọi `fault_reset()`: Hệ thống sẽ tự động xóa cảnh báo hãng Moons' (`0x2006 = 1`) và gửi chuỗi Fault Reset CiA 402 để kích hoạt lại Servo.

### 3. Kiểm tra lưu lượng CAN trực tiếp qua terminal khác:
```bash
# Đo tần số nhịp xung SYNC (mặc định 0.005s = 200 Hz):
candump can0,080:7FF -td

# Đo tần số phản hồi TPDO4 trục 1:
candump can0,481:7FF -td

# Đo tải toàn bus CAN:
canbusload can0@500000
```
