# Tài liệu Tổng quan về Động học và Odometry của Robot

Tài liệu này tổng hợp cấu trúc triển khai, công thức động học thuận (Forward Kinematics) để tính toán Odometry và động học nghịch (Inverse Kinematics) để điều khiển robot trong dự án (bao gồm cả môi trường mô phỏng và robot thực tế).

---

## 1. Cấu trúc Triển khai Odometry trong Dự án

Dự án hỗ trợ cả hai môi trường: mô phỏng (Gazebo) và chạy trên robot thực tế thông qua các cơ chế tính toán khác nhau.

### 1.1. Môi trường mô phỏng (Simulation)
- **[SimpleController](file:///home/namnc/V-SLam/moblie/src/controller_robot/controller_robot/simple_controller.py#L15)**: Đọc thông tin phản hồi từ khớp bánh xe thông qua topic `/joint_states`. Hàm callback [jointCallback](file:///home/namnc/V-SLam/moblie/src/controller_robot/controller_robot/simple_controller.py#L77) tính toán động học thuận vi sai để xuất bản Odometry lên topic `mobile_controller/odom` và truyền TF `odom` $\rightarrow$ `base_footprint`.
- **Cách tính khoảng thời gian `dt`**: 
  `dt` là hiệu số thời gian giữa hai thông điệp nhận được liên tiếp từ topic `/joint_states` dựa trên timestamp của thông điệp:
  ```python
  dt = Time.from_msg(msg.header.stamp) - self.prev_time_
  ```

### 1.2. Môi trường thực tế (Real Robot)
- **[MobileInterface](file:///home/namnc/V-SLam/moblie/src/robot_firmware/src/mobile_interface.cpp#L7)**: Là lớp phần cứng (hardware interface) kết nối trực tiếp với vi điều khiển (MCU) của robot qua cổng Serial. Lớp này đọc vận tốc bánh xe do MCU gửi lên, sau đó tích phân để cập nhật góc xoay khớp bánh xe (`position_states_`).
- **Cách tính khoảng thời gian `dt`**:
  Do dữ liệu nhận được trực tiếp qua giao tiếp Serial, `dt` được đo bằng đồng hồ thời gian thực của máy tính điều khiển PC tại hàm [read](file:///home/namnc/V-SLam/moblie/src/robot_firmware/src/mobile_interface.cpp#L139):
  ```cpp
  auto dt = (rclcpp::Clock().now() - last_run_).seconds();
  ```
  Sau đó, góc quay bánh xe được tích lũy theo công thức:
  ```cpp
  position_states_.at(i) += velocity_states_.at(i) * dt;
  ```

### 1.3. Bộ điều khiển chuyển động (`diff_drive_controller`)
- File cấu hình [mobile_controller.yaml](file:///home/namnc/V-SLam/moblie/src/controller_robot/config/mobile_controller.yaml#L22) cấu hình plugin `diff_drive_controller/DiffDriveController` để điều khiển robot vi sai khi chạy thực tế (được kích hoạt trong [real_robot.launch.py](file:///home/namnc/V-SLam/moblie/src/robot_bringup/launch/real_robot.launch.py#L45)).
- Nó đọc góc quay tích lũy (`position_states_`) từ `MobileInterface` và tự động tính toán Odometry dựa trên chu kỳ điều khiển (`publish_rate` = 50.0 Hz).
- Tham số `enable_odom_tf` đặt thành `false` nhằm nhường việc truyền TF cho bộ lọc Kalman.

### 1.4. Bộ lọc Kalman (Sensor Fusion)
- **EKF Node (`robot_localization`)**: Cấu hình tại [ekf.yaml](file:///home/namnc/V-SLam/moblie/src/mobile_localization/config/ekf.yaml) kết hợp dữ liệu odom thô từ bánh xe và vận tốc góc từ IMU để xuất bản ước lượng vị trí chính xác hơn và truyền TF `odom` $\rightarrow$ `base_footprint`.

---

## 2. Công thức Động học Thuận (Tính Odometry từ Encoder)

### Đầu vào (Inputs)
- Bán kính bánh xe: `r` (0.07333 m)
- Khoảng cách hai bánh xe: `L` (0.4544 m)
- Góc quay hiện tại từ encoder: `phi_left`, `phi_right`
- Góc quay ở chu kỳ trước: `phi_left_prev`, `phi_right_prev`
- Chu kỳ thời gian trôi qua: `dt` (Lấy từ timestamp của `/joint_states` trong mô phỏng, hoặc đo bằng PC clock trong chạy thực tế).

### Công thức tính toán
1. **Lượng dịch chuyển góc bánh xe:**
   ```text
   dp_left = phi_left - phi_left_prev
   dp_right = phi_right - phi_right_prev
   ```
2. **Vận tốc góc bánh xe tức thời:**
   ```text
   fi_left = dp_left / dt
   fi_right = dp_right / dt
   ```
3. **Vận tốc tuyến tính (`linear`) và vận tốc góc (`angular`) của robot:**
   ```text
   linear = r * (fi_right + fi_left) / 2
   angular = r * (fi_right - fi_left) / L
   ```
4. **Lượng dịch chuyển quãng đường (`d_s`) và góc xoay (`d_theta`) của robot:**
   ```text
   d_s = r * (dp_right + dp_left) / 2
   d_theta = r * (dp_right - dp_left) / L
   ```
5. **Cập nhật tọa độ tư thế mới (`x`, `y`, `theta`):**
   ```text
   theta = theta_old + d_theta
   x = x_old + d_s * cos(theta)
   y = y_old + d_s * sin(theta)
   ```

### Đầu ra (Outputs)
- **Tọa độ tư thế:** `x`, `y`, `theta` (truyền qua TF `odom` -> `base_footprint`).
- **Vận tốc tức thời:** Vận tốc tuyến tính `linear` và vận tốc góc `angular`.

---

## 3. Công thức Động học Nghịch (Điều khiển Bánh xe từ cmd_vel)

Khi điều khiển robot từ bàn phím (hoặc node điều hướng), robot nhận lệnh vận tốc dài `v` và vận tốc góc `omega` qua topic `/cmd_vel`. 

### Công thức chuyển đổi sang vận tốc bánh xe
Để tính vận tốc góc cần đặt cho bánh xe trái (`omega_left`) và bánh xe phải (`omega_right`):

```text
omega_left = (2 * v - omega * L) / (2 * r)
omega_right = (2 * v + omega * L) / (2 * r)
```

### Đầu ra (Outputs)
- Mảng vận tốc gửi tới driver điều khiển động cơ: `[omega_left, omega_right]` qua topic `simple_velocity_controller/commands`.

---

## 4. Ứng dụng Động học Nghịch Điều khiển Động cơ Thực tế (ZLAC8015D)

### 4.1. Thông số Phần cứng Động cơ
* **Đường kính bánh xe ($D$):** $173\text{ mm} = 0.173\text{ m} \implies$ **Bán kính bánh xe ($r$):** $0.0865\text{ m}$.
* **Khoảng cách 2 bánh xe ($L$):** $400\text{ mm} = 0.400\text{ m}$.
* **Đơn vị vận tốc cài đặt cho ZLAC8015D (`0x60FF:03`):** RPM (vòng/phút).

### 4.2. Công thức Đổi từ $(v, \omega)$ sang Vòng/Phút (RPM)
Từ vận tốc dài $v$ ($\text{m/s}$) và vận tốc góc $\omega$ ($\text{rad/s}$) nhận được:

1. **Vận tốc tuyến tính của từng bánh xe ($\text{m/s}$):**
   $$v_L = v - \frac{\omega \cdot L}{2} = v - \frac{\omega \cdot 0.400}{2} = v - 0.200 \cdot \omega$$
   $$v_R = v + \frac{\omega \cdot L}{2} = v + \frac{\omega \cdot 0.400}{2} = v + 0.200 \cdot \omega$$

2. **Vận tốc góc của từng bánh xe ($\text{rad/s}$):**
   $$\omega_L = \frac{v_L}{r} = \frac{v_L}{0.0865}$$
   $$\omega_R = \frac{v_R}{r} = \frac{v_R}{0.0865}$$

3. **Chuyển đổi sang RPM cho Driver ZLAC8015D:**
   $$\text{RPM}_L = \omega_L \cdot \frac{60}{2\pi} = \frac{v_L}{0.0865} \cdot \frac{30}{\pi} \approx v_L \cdot 110.316$$
   $$\text{RPM}_R = \omega_R \cdot \frac{60}{2\pi} = \frac{v_R}{0.0865} \cdot \frac{30}{\pi} \approx v_R \cdot 110.316$$

4. **Đóng gói dữ liệu CANopen gửi vào `0x60FF:03`:**
   $$\text{Combined\_32bit} = (\text{RPM}_L \;\&\; \text{0xFFFF}) \;\|\; ((\text{RPM}_R \;\&\; \text{0xFFFF}) \ll 16)$$

