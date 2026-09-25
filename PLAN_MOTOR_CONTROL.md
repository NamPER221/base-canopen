# Kế hoạch Triển khai Điều khiển Động cơ ZLAC8015D với Động học Vi sai

## Context

Triển khai module điều khiển động cơ ZLAC8015D (motor driver) kết hợp với động học vi sai (differential drive kinematics) dựa trên:
- CANopen framework đã có sẵn (CAN Driver, SDO, PDO, NMT)
- EDS file chứa Object Dictionary đầy đủ
- Tài liệu động học (Forward/Inverse Kinematics)

---

## Tổng quan Kiến trúc

```
┌─────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Motor Test   │  │  Kinematics  │  │   Controller │      │
│  │   (TEST)     │  │   (NODE)     │  │   (NODE)     │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
├─────────────────────────────────────────────────────────────┤
│                   MOTOR DRIVER LAYER                         │
│  ┌──────────────────────────────────────────────────┐       │
│  │            ZLAC8015Driver                         │       │
│  │  - Enable/Disable motor                          │       │
│  │  - Set velocity (RPM)                           │       │
│  │  - Read status/position/velocity                 │       │
│  └──────────────────────────────────────────────────┘       │
├─────────────────────────────────────────────────────────────┤
│                   CANopen LAYER (Đã có)                     │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐   │
│  │  NMT   │ │  SDO   │ │  PDO   │ │  EMCY  │ │  HB    │   │
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘   │
├─────────────────────────────────────────────────────────────┤
│                   CAN DRIVER LAYER (Đã có)                  │
│                    SocketCAN Interface                       │
└─────────────────────────────────────────────────────────────┘
```

---

## Thông số Hardware từ EDS File

| Thông số | Giá trị |
|----------|---------|
| Tên driver | ZLAC8015D |
| Wheel diameter | 173 mm → r = 0.0865 m |
| Wheelbase | 400 mm → L = 0.400 m |
| Max speed | 1000 RPM |
| CAN Node ID default | 1 |
| CAN Baudrate default | 1 (500Kbps) |

---

## Thông số Kinematics từ odometry_kinematics_summary.md

### Forward Kinematics (Encoder → Robot Pose)
```cpp
// Wheel angular velocities
fi_L = (phi_L - phi_L_prev) / dt
fi_R = (phi_R - phi_R_prev) / dt

// Robot velocities
linear_v = r * (fi_R + fi_L) / 2
angular_w = r * (fi_R - fi_L) / L

// Pose update
d_s = r * (dp_R + dp_L) / 2
d_theta = r * (dp_R - dp_L) / L
x += d_s * cos(theta)
y += d_s * sin(theta)
theta += d_theta
```

### Inverse Kinematics (cmd_vel → Wheel Velocities)
```cpp
// From cmd_vel (v, omega) to wheel linear velocities
v_L = v - omega * L / 2
v_R = v + omega * L / 2

// Convert to angular velocity (rad/s)
omega_L_rad = v_L / r
omega_R_rad = v_R / r

// Convert to RPM for ZLAC8015D
RPM_L = omega_L_rad * 60 / (2 * PI) ≈ omega_L_rad * 9.5493
RPM_R = omega_R_rad * 60 / (2 * PI) ≈ omega_R_rad * 9.5493

// Pack into 32-bit for 0x60FF:03
// Combined = (RPM_L & 0xFFFF) | ((RPM_R & 0xFFFF) << 16)
```

---

## Object Dictionary Quan trọng từ EDS

### Control Objects (CiA 402)
| Index:Sub | Tên | Mô tả |
|-----------|-----|-------|
| 0x6040 | Controlword | 0x000F = Enable, 0x0007 = Shutdown |
| 0x6041 | Statusword | Bit 0=SwitchedOn, Bit 1=Enabled, Bit 3=Fault |
| 0x6060 | Modes of operation | 1=Profile Position, 3=Profile Velocity, -3=Velocity |
| 0x6061 | Mode of operation display | Chế độ hiện tại |
| 0x6064 | Position actual value | [sub1=Left, sub2=Right] (encoder counts) |
| 0x606C | Velocity actual value | [sub1=Left, sub2=Right] (RPM) |
| 0x6071 | Target torque | [sub1=Left, sub2=Right] (mNm) |
| 0x6077 | Torque actual value | [sub1=Left, sub2=Right] |
| 0x60FF | **Target velocity** | **[sub1=Left, sub2=Right, sub3=Combined32]** |

### Manufacturer Objects (0x2000-0x2035)
| Index:Sub | Tên | Mô tả |
|-----------|-----|-------|
| 0x2001 | RS485 Node ID | 1-127 |
| 0x2002 | RS485 Baudrate | 0-6 |
| 0x200A | CAN Node ID | 1-127 |
| 0x200B | CAN Baudrate | 0-6 (0=1M, 1=500K, 2=250K, 3=125K...) |
| 0x200C:1 | Left Motor Pole | 4-64, default=15 |
| 0x200C:2 | Right Motor Pole | 4-64, default=15 |
| 0x200F | Syn Control | 0-1 (sync control enable) |

---

## Danh sách Task

### Task 1: Tạo Motor Driver Layer (ZLAC8015Driver)
**Priority: 10 | Dependencies: None**

**Mục tiêu:** Tạo class ZLAC8015Driver để điều khiển motor qua CANopen SDO

**Files cần tạo:**
- `include/canopen/device/zlac8015_driver.hpp`
- `src/device/zlac8015_driver.cpp`

**Steps:**
1. Tạo class ZLAC8015Driver kế thừa từ CANopenDevice
2. Implement các methods:
   - `enable_motor()` - Gửi SDO 0x6040 = 0x000F
   - `disable_motor()` - Gửi SDO 0x6040 = 0x0000
   - `set_velocity_rpm(int16_t left_rpm, int16_t right_rpm)` - Gửi SDO 0x60FF:03
   - `read_status()` - Đọc SDO 0x6041
   - `read_velocity()` - Đọc SDO 0x606C:01, 0x606C:02
   - `read_position()` - Đọc SDO 0x6064:01, 0x6064:02
3. Implement error handling và timeout

**Code structure:**
```cpp
class ZLAC8015Driver : public CANopenDevice {
public:
    ZLAC8015Driver(uint8_t node_id, void* bus);
    
    // Motor control
    bool enable();
    bool disable();
    bool set_velocity(int16_t left_rpm, int16_t right_rpm);
    
    // Feedback
    Statusword get_status();
    int16_t get_velocity_left();
    int16_t get_velocity_right();
    int32_t get_position_left();
    int32_t get_position_right();
    
private:
    std::unique_ptr<SDOClient> sdo_;
};
```

---

### Task 2: Tạo Kinematics Layer
**Priority: 9 | Dependencies: Task 1**

**Mục tiêu:** Implement Forward/Inverse Kinematics cho differential drive

**Files cần tạo:**
- `include/canopen/kinematics/differential_drive.hpp`
- `src/kinematics/differential_drive.cpp`

**Steps:**
1. Tạo class DifferentialDriveKinematics
2. Implement Forward Kinematics:
   - `update_from_encoders(dp_left, dp_right, dt)` → (v, omega, x, y, theta)
3. Implement Inverse Kinematics:
   - `velocity_to_wheel_rpm(v, omega)` → (rpm_left, rpm_right)
4. Implement helper methods:
   - `set_wheel_parameters(radius, wheelbase)`
   - `reset_pose(x, y, theta)`
   - `get_pose()`

**Code structure:**
```cpp
class DifferentialDriveKinematics {
public:
    DifferentialDriveKinematics(double wheel_radius, double wheelbase);
    
    // Forward kinematics
    void update(double dp_left, double dp_right, double dt);
    double get_linear_velocity() const;
    double get_angular_velocity() const;
    
    // Pose
    struct Pose { double x, y, theta; };
    Pose get_pose() const;
    void reset_pose(double x, double y, double theta);
    
    // Inverse kinematics
    struct WheelRPM { int16_t left; int16_t right; };
    WheelRPM velocity_to_rpm(double v, double omega) const;
    
private:
    double wheel_radius_, wheelbase_;
    Pose pose_;
};
```

---

### Task 3: Tạo Test Motor Control
**Priority: 10 | Dependencies: Task 1**

**Mục tiêu:** Tạo test để validate điều khiển 1 motor

**Files cần tạo:**
- `test/src/test_motor_driver.cpp`
- `examples/motor_test.cpp`

**Steps:**
1. Test kết nối CAN và đọc Statusword
2. Test enable/disable motor
3. Test set velocity 50 RPM
4. Test đọc velocity feedback
5. Test stop motor

**Test cases:**
```cpp
TEST_F(ZLAC8015DriverTest, EnableMotor) {
    EXPECT_TRUE(driver.enable());
    auto status = driver.get_status();
    EXPECT_TRUE(status.is_enabled());
}

TEST_F(ZLAC8015DriverTest, SetVelocity) {
    driver.enable();
    EXPECT_TRUE(driver.set_velocity(50, 50));  // 50 RPM both wheels
    std::this_thread::sleep_for(100ms);
    auto v_left = driver.get_velocity_left();
    EXPECT_NEAR(v_left, 50, 5);  // ±5 RPM tolerance
}

TEST_F(ZLAC8015DriverTest, StopMotor) {
    driver.enable();
    driver.set_velocity(0, 0);
    auto v = driver.get_velocity_left();
    EXPECT_EQ(v, 0);
}
```

---

### Task 4: Tạo Test Kinematics
**Priority: 9 | Dependencies: Task 2**

**Mục tiêu:** Validate Forward/Inverse Kinematics calculations

**Files cần tạo:**
- `test/src/test_kinematics.cpp`

**Steps:**
1. Test forward kinematics - straight line
2. Test forward kinematics - rotation
3. Test inverse kinematics - straight line
4. Test inverse kinematics - arc motion
5. Test round-trip accuracy (FK then IK should match)

**Test cases:**
```cpp
TEST(DifferentialKinematicsTest, ForwardKinematics_StraightLine) {
    DifferentialDriveKinematics kin(0.0865, 0.400);
    kin.update(0.1, 0.1, 0.1);  // Same encoder delta
    auto pose = kin.get_pose();
    EXPECT_NEAR(pose.theta, 0.0, 1e-6);  // No rotation
}

TEST(DifferentialKinematicsTest, InverseKinematics_StraightLine) {
    DifferentialDriveKinematics kin(0.0865, 0.400);
    auto rpm = kin.velocity_to_rpm(0.5, 0.0);  // 0.5 m/s, no rotation
    EXPECT_EQ(rpm.left, rpm.right);  // Same RPM
}
```

---

### Task 5: Test Dual Motor với Kinematics
**Priority: 10 | Dependencies: Task 3, Task 4**

**Mục tiêu:** Test điều khiển 2 motor đồng thời với kinematics

**Files cần tạo:**
- `examples/dual_motor_kinematics_test.cpp`

**Steps:**
1. Khởi tạo ZLAC8015Driver
2. Khởi tạo DifferentialDriveKinematics
3. Test điều khiển thẳng (v=0.5, omega=0)
4. Test điều khiển quay tại chỗ (v=0, omega=1.0)
5. Test điều khiển arc (v=0.3, omega=0.5)
6. Log kết quả odometry

**Scenario:**
```cpp
// Straight line
kin.velocity_to_rpm(0.5, 0.0);  // 0.5 m/s forward
// → RPM_L ≈ RPM_R ≈ 55 RPM

// Point turn
kin.velocity_to_rpm(0.0, 1.57);  // 90 deg/s rotation
// → RPM_L = -RPM_R

// Arc turn
kin.velocity_to_rpm(0.3, 0.5);  
// → RPM_L ≠ RPM_R
```

---

### Task 6: Integration với ROS2 Control
**Priority: 8 | Dependencies: Task 5**

**Mục tiêu:** Tạo ROS2 hardware interface cho robot

**Files cần tạo:**
- `include/canopen/device/zlac8015_hw_interface.hpp`
- `src/device/zlac8015_hw_interface.cpp`
- `config/zlac8015_hw_interface.yaml`

**Steps:**
1. Tạo class kế thừa `hardware_interface::SystemInterface`
2. Implement `on_init()` - Khởi tạo CAN và driver
3. Implement `on_configure()` - Enable motors
4. Implement `read()` - Đọc encoder/velocity
5. Implement `write()` - Gửi velocity command
6. Tạo URDF và xacro

**Interface specification:**
```yaml
hardware_interface:
  joints:
    - name: "left_wheel_joint"
      velocity_command_interface: true
      position_state_interface: true
    - name: "right_wheel_joint"
      velocity_command_interface: true
      position_state_interface: true
```

---

## File Structure sau khi triển khai

```
canopen/
├── include/canopen/
│   ├── device/
│   │   ├── canopen_device.hpp    # (existing)
│   │   ├── zlac8015_driver.hpp   # [NEW] Motor driver
│   │   └── zlac8015_hw_interface.hpp  # [NEW] ROS2 HW interface
│   └── kinematics/
│       └── differential_drive.hpp # [NEW] Kinematics
├── src/
│   ├── device/
│   │   ├── canopen_device.cpp     # (existing)
│   │   ├── zlac8015_driver.cpp    # [NEW]
│   │   └── zlac8015_hw_interface.cpp  # [NEW]
│   └── kinematics/
│       └── differential_drive.cpp  # [NEW]
├── examples/
│   ├── master_example.cpp        # (existing)
│   ├── motor_test.cpp            # [NEW] Single motor test
│   └── dual_motor_kinematics_test.cpp  # [NEW] Dual motor + kinematics
└── test/
    ├── src/
    │   ├── test_motor_driver.cpp  # [NEW]
    │   └── test_kinematics.cpp    # [NEW]
    └── CMakeLists.txt
```

---

## Execution Order

```
Task 1: Motor Driver Layer (ZLAC8015Driver)
    ↓
Task 2: Kinematics Layer
    ↓
Task 3: Test Motor Driver ←┐
    ↓                      │
Task 4: Test Kinematics ──┼── (có thể song song)
    ↓                      │
Task 5: Test Dual Motor với Kinematics
    ↓
Task 6: ROS2 Control Integration (optional)
```

---

## Verification Plan

### Phase 1: Motor Driver Tests
```bash
# Build
colcon build --packages-select canopen

# Run motor tests
ros2 run canopen test_motor_driver

# Expected output:
# [PASSED] EnableMotor
# [PASSED] SetVelocity_50RPM
# [PASSED] SetVelocity_NegativeRPM
# [PASSED] ReadVelocity
# [PASSED] DisableMotor
```

### Phase 2: Kinematics Tests
```bash
ros2 run canopen test_kinematics

# Expected output:
# [PASSED] ForwardKinematics_StraightLine
# [PASSED] ForwardKinematics_Rotation
# [PASSED] InverseKinematics_StraightLine
# [PASSED] InverseKinematics_Arc
# [PASSED] RoundTripAccuracy
```

### Phase 3: Integration Test
```bash
# Hardware-in-the-loop
ros2 run canopen dual_motor_kinematics_test

# Verify:
# 1. Robot moves straight when v=0.5, omega=0
# 2. Robot rotates when v=0, omega≠0
# 3. Odometry matches expected trajectory
```

---

## CANopen Message Sequences

### Enable Motor
```
Master → Slave (NMT): 0x000 0x01  # Switch to Operational
Master → Slave (SDO): 0x600+ID 0x23 0x40 0x60 0x00 0x0F 0x00 0x00 0x00  # Controlword = 0x000F
```

### Set Velocity
```
# Object 0x60FF:03 - Combined 32-bit
# Format: Left RPM (16-bit) | Right RPM (16-bit)
# Example: RPM_L = 50, RPM_R = 50
# Combined = (50 & 0xFFFF) | ((50 & 0xFFFF) << 16)
#          = 0x0032 | (0x00320000)
#          = 0x00320032

Master → Slave (SDO): 0x600+ID 0x23 0xFF 0x60 0x03 0x32 0x00 0x32 0x00
```

### Read Status
```
Master → Slave (SDO): 0x600+ID 0x40 0x41 0x60 0x00 0x00 0x00 0x00 0x00
Slave → Master (SDO): 0x580+ID 0x43 0x41 0x60 0x00 0x37 0x00 0x00 0x00  # Status = 0x0037
```
