# Cross-compile cho robot arm64 (aarch64)
#
# Dùng:
#   cmake -S . -B build-arm64 \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64.cmake \
#         -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-arm64 -j$(nproc)
#   cpack --config build-arm64/CPackConfig.cmake
#
# Cài toolchain trước (cần sudo):
#   sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
#
# Vì sao không dùng toolchain của máy build: file .pc và canopenConfig.cmake
# sẽ ghi cứng đường dẫn của máy build. Đặt CMAKE_INSTALL_PREFIX=/opt/canopen
# (một đường dẫn giống nhau trên mọi máy) giúp bản cài đặt chạy được sau khi
# copy sang robot mà không phải sửa lại file nào.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)

# Tìm thư viện hệ thống TRONG sysroot của toolchain, không tìm trên máy build
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Chặn thư viện của máy x86 lọt vào gói arm — đây là nguyên nhân kinh điển
# làm file .deb "sai kiến trúc" mà vẫn đóng gói thành công.
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE_EXCLUDE
    "x86_64-linux-gnu" "i386-linux-gnu" "i686-linux-gnu")

# Cùng một đường dẫn cài đặt trên mọi máy để bản đóng gói chạy được ở
# cả máy dev lẫn robot mà không cần chỉnh sửa.
set(CMAKE_INSTALL_PREFIX /opt/canopen CACHE PATH "prefix" FORCE)

# Không sinh đường dẫn ghi tuyệt đối vào binary
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
