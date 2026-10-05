# Hướng dẫn Cài đặt & Tích hợp (Integration Guide)

Dự án `base-canopen` được thiết kế theo chuẩn CMake hiện đại, xuất các Targets (`canopen::canopen`, `canopen::canopen_mbdv`, v.v.) và hỗ trợ `pkg-config`. Điều này giúp bạn dễ dàng kết hợp (nhúng) vào các dự án C++ khác hoặc các Node ROS2.

---

## 1. Cài đặt vào hệ thống (System Install)

Trước khi một dự án khác có thể gọi được thư viện này, bạn cần cài đặt nó vào hệ thống Ubuntu/Linux. Có 2 cách:

### Cách A: Cài đặt trực tiếp qua Make
Từ thư mục gốc của `base-canopen`:
```bash
mkdir -p build && cd build
cmake ..
make -j4
sudo make install
```
Lệnh này sẽ copy các file header vào `/usr/local/include/canopen` và các file `.a` / `.so` vào `/usr/local/lib/`.

### Cách B: Đóng gói thành file `.deb` (Khuyên dùng)
Nếu bạn muốn dễ dàng gỡ cài đặt hoặc mang sang robot khác (Raspberry Pi, Jetson):
```bash
cd build
cpack -G DEB
sudo dpkg -i canopen-0.2.0-Linux.deb
```

---

## 2. Kết hợp vào dự án C++ chuẩn (CMake)

Trong dự án C++ của bạn, thêm đoạn sau vào `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_robot_app)

# 1. Tìm thư viện canopen đã được cài đặt
find_package(canopen REQUIRED)

add_executable(my_app main.cpp)

# 2. Link thư viện lõi (canopen::canopen) 
#    và driver tương ứng (canopen::canopen_mbdv hoặc canopen::canopen_zlac)
target_link_libraries(my_app PRIVATE 
    canopen::canopen 
    canopen::canopen_mbdv
    canopen::canopen_kinematics
)
```

**Trong file C++ của bạn (`main.cpp`):**
```cpp
#include <canopen/can/raw/socket_can_bus.hpp>
#include <canopen/drivers/mbdv/mbdv_driver.hpp>

int main() {
    canopen::SocketCanBus bus("can0");
    bus.open();
    // Tạo driver node 1
    canopen::drivers::MbdvAxis axis(1, "LeftMotor", &bus);
    axis.init(2000);
    axis.enable();
    axis.set_target_velocity_counts(50000); // quay!
    return 0;
}
```

---

## 3. Kết hợp vào dự án ROS2 (Colcon/Ament)

Nếu bạn muốn viết một Hardware Interface (ros2_control) hoặc một ROS2 Node đơn giản để điều khiển robot, `base-canopen` tích hợp cực kỳ trơn tru với `ament_cmake`.

Trong `CMakeLists.txt` của ROS2 package (`my_robot_controller`):

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Tìm thư viện base-canopen
find_package(canopen REQUIRED)

add_executable(canopen_node src/canopen_node.cpp)

# Tích hợp ROS2 và base-canopen
ament_target_dependencies(canopen_node rclcpp)
target_link_libraries(canopen_node PRIVATE 
    canopen::canopen 
    canopen::canopen_zlac 
    canopen::canopen_kinematics
)

install(TARGETS canopen_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
```

### Cách nạp file YAML cho Driver trong ROS2
Bạn có thể đọc file cấu hình `.yaml` (ví dụ: `mbdv_config.yaml` mà chúng ta vừa tạo) vào ROS2 Node hoặc dùng C++ `yaml-cpp` thuần.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
// ... include các header canopen ...

YAML::Node config = YAML::LoadFile("/đường/dẫn/tới/drivers/mbdv/config/mbdv_config.yaml");
double wheelbase = config["kinematics"]["wheelbase"].as<double>();
// Khởi tạo các driver với thông số đọc từ YAML
```

---

## 4. Compile bằng dòng lệnh thuần không dùng CMake (pkg-config)

Nếu bạn chỉ muốn thử nghiệm một script C++ nhỏ bằng `g++`:

```bash
g++ my_test.cpp $(pkg-config --cflags --libs canopen) -o my_test
./my_test
```
