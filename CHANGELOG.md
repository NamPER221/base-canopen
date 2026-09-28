# Changelog

Tất cả thay đổi đáng chú ý của dự án này sẽ được ghi vào file này.

Định dạng dựa trên [Keep a Changelog](https://keepachangelog.com/vi/1.1.0/),
dự án tuân thủ [Semantic Versioning](https://semver.org/lang/vi/).

## [Chưa phát hành]

### Đã thêm
- **Đóng gói phát hành**: `.deb` (amd64/arm64), tarball nhị phân, và tarball mã
  nguồn, qua `scripts/package.sh` và CPack.
- **`cmake/toolchain-aarch64.cmake`**: cross-compile sang arm64 cho robot.
- **`docs/API.md`**: hướng dẫn dùng từng nhóm hàm kèm ví dụ gọi thật.
- **Doxygen** (`docs/generate_docs.sh`): tra cứu chữ ký đầy đủ từ header.
- **`test_docs`**: kiểm tra mọi hàm ghi trong `docs/API.md` đều tồn tại trong
  header, để tài liệu không lệch khỏi code sau khi đổi tên hàm.
- **Đóng gói CMake**: `canopenConfig.cmake` + `canopenConfigVersion.cmake`, hỗ trợ
  `find_package(canopen 0.2 REQUIRED)` và `canopen::canopen`.
- **`test/consumer/`** — dự án mẫu bên ngoài dùng `find_package(canopen)`, chứng minh
  thư viện dùng được từ bên ngoài chứ không chỉ build tại chỗ.
- **`scripts/test_install.sh`** — build → `install()` vào prefix tạm → build consumer →
  chạy test, kèm kiểm tra `pkg-config`.
- **CI (`.github/workflows/ci.yml`)** — 4 job: build trên GCC 12/13 + Clang 16 (C++20
  và C++23), AddressSanitizer + UBSan, install & consume, build ví dụ.
- **`canopen/version.hpp`** sinh từ CMake — một nguồn version duy nhất cho cả
  `find_package()` lẫn `canopen_version()`, kèm macro `CANOPEN_VERSION_MAJOR/MINOR/PATCH`.
- **README.md** — cài đặt, quick start, tài liệu các ngoại lệ thiết bị, trạng thái thư viện.

### Đã sửa
- Version lệch nhau giữa `CMakeLists.txt` (`1.0.0`) và `canopen_version()` (`0.2.0`).
  Nay cả hai lấy từ `project(canopen VERSION ...)`.
- `canopen.pc` ghi cứng `prefix` lúc configure, nên bản đóng gói `.deb` (cài vào
  `/usr`) trỏ nhầm sang `/usr/local`; `libdir` còn là đường dẫn tương đối.
  Nay dùng `${pcfiledir}` để file `.pc` tự suy ra prefix và có thể đặt ở bất kỳ đâu.
- Tên gói CPack thiếu kiến trúc (`canopen-0.2.0-Linux.deb`); nay suy ra từ
  `CMAKE_SYSTEM_PROCESSOR` ra tên kiến trúc kiểu Debian.
- `scripts/test_install.sh` trước đây chỉ kiểm `pkg-config --modversion`, nên
  bỏ sót lỗi đường dẫn; nay kiểm tra cả include/lib phải tuyệt đối và tồn tại.

## [0.2.0]

### Đã thêm
- **Tầng generic CiA 402**: `DeviceProfile` (nạp EDS theo CiA 306, tự dò vai trò
  động cơ, phân biệt object đơn với object mảng, override kích thước, hỗ trợ
  little/big endian) và `MotorDevice` (`connect()`, operation mode, profile,
  velocity, đọc/ghi theo vai trò).
- **Driver ZLAC8015D** (`libcanopen_zlac`): RPDO/TPDO, tự dừng an toàn khi mất
  heartbeat, tự kết nối lại và cấu hình lại toàn bộ.
- **Bộ đếm runtime** `rpdo_sent()` / `sdo_sent()` để chẩn đoán kênh điều khiển.
- **Theo dõi heartbeat** có phục hồi: node offline rồi online lại sẽ kích hoạt
  lại cơ chế an toàn thay vì bị bỏ qua.
- **`scripts/sync_to_robot.sh`** — đồng bộ mã nguồn sang robot kèm kiểm tra checksum.
- 10 bộ unit test dùng `FakeBus`, chạy được không cần phần cứng.

### Đã sửa
- **SDO**: thêm `SdoEncoding` với hai chế độ. `Standard` theo CiA 301
  (`0x2C`/`0x2E`), `Legacy` cho thiết bị dùng encoding cũ (`0x23`/`0x2B`).
  Parser nhận cả hai dạng response (`0x4C`/`0x43`) và so khớp response confirm
  `0x60` không echo index/subindex.
- **Heartbeat**: timeout có thể hồi phục, thêm callback `recovered`, tách timeout
  riêng cho từng node.
- **PDO của ZLAC**: DLC phải khớp chính xác 4 byte cho mapping `0x60FF:03` 32-bit
  gộp 2 trục — gửi DLC=8 bị drive bỏ qua. Cấu hình PDO được áp dụng lại sau mỗi
  lần NMT Reset Communication (`0x82`) vì lệnh này xoá cấu hình.
- Ghi EEPROM khi cấu hình PDO giờ là **tuỳ chọn**, mặc định không ghi mỗi lần boot.
- Thông số động cơ ZLLG65ASM250 cập nhật theo datasheet: encoder `4096`,
  tốc độ tối đa `205` RPM.

### Đã biết
- `MotorDevice` không kết nối được ZLAC8015D vì thiết bị này vi phạm CiA 301
  (SDO encoding cũ, sai kích thước object trong EDS, thiếu `0x1019`). Cần dùng
  `ZLAC8015Driver`. Đây là chủ ý tách lớp, không phải hạn chế.
