cmake_minimum_required(VERSION 3.16)
project(canopen VERSION 0.2.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# Build options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(BUILD_TESTING "Build tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)

# Find packages
find_package(Threads REQUIRED)

# Source groups
set(CANOPEN_CAN_RAW_SOURCES
    src/can/raw/socket_can.cpp
    src/can/raw/filter_manager.cpp
    src/can/raw/bus_monitor.cpp
    src/can/raw/socket_can_bus.cpp
)

set(CANOPEN_CAN_FRAME_SOURCES
    src/can/frame/frame.cpp
    src/can/msg/message_factory.cpp
)

set(CANOPEN_CO_SOURCES
    src/co/object_dictionary/object_dictionary.cpp
    src/co/object_dictionary/eds_parser.cpp
    src/co/nmt/nmt.cpp
    src/co/sdo/sdo.cpp
    src/co/pdo/pdo.cpp
    src/co/emcy/emcy.cpp
    src/co/heartbeat/heartbeat.cpp
    src/co/sync/sync.cpp
    src/co/lss/lss.cpp
    src/co/cia402/cia402_drive.cpp
)

set(CANOPEN_RECOVERY_SOURCES
    src/recovery/bus_state_manager.cpp
)

set(CANOPEN_DEVICE_SOURCES
    src/device/canopen_device.cpp
    src/device/device_profile.cpp
    src/device/motor_device.cpp
)

set(CANOPEN_EV_SOURCES
    src/ev/event_loop.cpp
)

# Main library
set(CANOPEN_SOURCES
    ${CANOPEN_CAN_RAW_SOURCES}
    ${CANOPEN_CAN_FRAME_SOURCES}
    ${CANOPEN_CO_SOURCES}
    ${CANOPEN_RECOVERY_SOURCES}
    ${CANOPEN_DEVICE_SOURCES}
    ${CANOPEN_EV_SOURCES}
)

add_library(canopen ${CANOPEN_SOURCES})

target_include_directories(canopen
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # version.hpp sinh ra phải nằm trong include path PUBLIC vì frame.cpp
        # (thuộc thư viện) dùng nó; nhưng chỉ cần ở phía build.
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(canopen
    PUBLIC
        Threads::Threads
)

# Linux-specific libraries
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(canopen PUBLIC pthread)
endif()

# Generate headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ==================== Optional device driver libraries ====================
# Drivers are thin vendor layers built on top of the generic core library.
option(BUILD_KINEMATICS "Build differential drive kinematics library" ON)
option(BUILD_ZLAC_DRIVER "Build ZLAC8015D dual-axis servo driver library" ON)
option(BUILD_MBDV_DRIVER "Build Moons' MBDV dual-axis servo driver library" ON)

if(BUILD_KINEMATICS)
    add_subdirectory(drivers/kinematics)
endif()

if(BUILD_ZLAC_DRIVER)
    add_subdirectory(drivers/zlac8015)
endif()

if(BUILD_MBDV_DRIVER)
    add_subdirectory(drivers/mbdv)
endif()

# Subdirectories
if(BUILD_TESTING)
    # enable_testing() phải nằm ở thư mục GỐC, không phải trong test/.
    # Nếu chỉ bật trong test/, thì `ctest --test-dir build` chạy từ thư mục
    # build sẽ không thấy test nào — và ctest vẫn trả về mã 0, khiến CI
    # báo "thành công" dù chưa chạy kiểm thử gì.
    enable_testing()
    add_subdirectory(test)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Installation
include(GNUInstallDirs)

install(TARGETS canopen
    EXPORT canopenTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/canopen
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
# version.hpp sinh từ CMake — cài để bên ngoài dùng được macro
# CANOPEN_VERSION_STRING mà không phải parse canopen_version() lúc chạy
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/canopen/version.hpp"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/canopen
)

# Install driver libraries (optional)
if(BUILD_KINEMATICS)
    install(TARGETS canopen_kinematics
        EXPORT canopenTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

if(BUILD_ZLAC_DRIVER)
    install(TARGETS canopen_zlac
        EXPORT canopenTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

if(BUILD_MBDV_DRIVER)
    install(TARGETS canopen_mbdv
        EXPORT canopenTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

install(EXPORT canopenTargets
    FILE canopenTargets.cmake
    NAMESPACE canopen::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/canopen
)

# Sinh canopen/version.hpp từ project() — một nguồn version duy nhất cho cả
# find_package(canopen 0.2.0) lẫn canopen::canopen_version().
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/canopen/version.hpp"
    @ONLY
)

# File Config cho find_package(canopen) — không có nó thì find_package sẽ
# thất bại dù lib đã được cài đặt.
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/canopenConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/canopenConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/canopen
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/canopenConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/canopenConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/canopenConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/canopen
)

# Export package
export(TARGETS canopen
    FILE "${CMAKE_CURRENT_BINARY_DIR}/canopenTargets.cmake"
)

# pkg-config file
#
# Dùng ${pcfiledir} để file .pc tự biết mình nằm ở đâu, thay vì ghi cứng
# CMAKE_INSTALL_PREFIX lúc configure. Lý do: cùng một file .pc được đóng gói
# vào .deb (cài vào /usr) và vào bản .tar.gz (giải nén ở bất kỳ đâu) — nếu
# hard-code prefix thì bản .deb sẽ trỏ nhầm sang /usr/local và hỏng.
# ${pcfiledir} = thư mục chứa file .pc, nên ../.. đưa về gốc prefix.
set(PC_FILE "${CMAKE_CURRENT_BINARY_DIR}/canopen.pc")
file(WRITE ${PC_FILE}
"prefix=\${pcfiledir}/../..\n"
"libdir=\${prefix}/${CMAKE_INSTALL_LIBDIR}\n"
"includedir=\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}\n"
"\n"
"Name: canopen\n"
"Description: CANopen stack cho robot (SDO/NMT/PDO/EDS + driver ZLAC8015D)\n"
"Version: ${PROJECT_VERSION}\n"
"Requires:\n"
"Cflags: -I\${includedir}\n"
"Libs: -L\${libdir} -lcanopen\n"
)
install(FILES ${PC_FILE}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# =============================================================================
# Packaging — CPack
#
# Sinh 3 loại gói phục vụ 3 nhu cầu khác nhau:
#   cpack --config CPackSourceConfig.cmake  -> .tar.gz mã nguồn (mọi kiến trúc)
#   cpack -G TGZ                            -> cây cài đặt nén (copy tay sang robot)
#   cpack -G DEB                            -> .deb cài bằng apt/dpkg
#
# Vì sao cần cả ba: .deb tiện nhưng gắn với kiến trúc, còn robot và máy dev
# khác kiến trúc nên cần bản nén cây cài đặt để giải nén là chạy.
#
# ★ Mọi biến CPACK_* PHẢI đặt TRƯỚC include(CPack) ★
#   include(CPack) ghi file CPackConfig.cmake ngay tại chỗ nó được gọi, nên
#   biến đặt sau sẽ không tới được file đó. Đặt sai chỗ sẽ âm thầm rơi về
#   generator mặc định của CPack và tạo ra file .sh thay vì .deb.
# =============================================================================

# ---- Metadata dùng chung cho mọi generator ----
set(CPACK_PACKAGE_NAME "canopen")
set(CPACK_PACKAGE_VENDOR "base-canopen")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Thư viện CANopen C++20: SDO/NMT/PDO/EDS, driver ZLAC8015D, kinematics robot 2 bánh")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/NamPER221/base-canopen")
# Chỉ khai báo LICENSE nếu thật sự có — CPack báo lỗi nếu trỏ vào file không tồn tại
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
endif()

# ---- .deb ----
# dpkg-deb yêu cầu "Section"/"Priority"/"Maintainer"/"Description"; thiếu bất
# kỳ mục nào cũng làm lệnh `cpack -G DEB` dừng với lỗi chung chung.
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_DEBIAN_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "base-canopen <base-canopen@users.noreply.github.com>")
set(CPACK_PACKAGE_CONTACT "base-canopen")
set(CPACK_PACKAGE_DESCRIPTION_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/package-description.txt")
# Tắt shlibdeps: thư viện tĩnh không có SONAME, và shlibdeps cần đường dẫn
# cài đặt thật nên sẽ hỏng khi đóng gói ngoài môi trường đã cài.
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS "OFF")
set(CPACK_DEB_COMPONENT_INSTALL ON)

# CPack mặc định đặt tên gói theo CPACK_SYSTEM_NAME, nếu không khai báo thì ra
# "canopen-0.2.0-Linux.deb" — không có kiến trúc, kho Debian phân biệt các
# bản arm64/amd64 bằng chính cái tên đó. Suy ra từ CMAKE_SYSTEM_PROCESSOR để
# đúng cả khi cross-compile (đọc dpkg trên máy build sẽ ra sai khi đó).
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(CPACK_SYSTEM_NAME "arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(CPACK_SYSTEM_NAME "amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(armv7l|armhf)$")
    set(CPACK_SYSTEM_NAME "armhf")
else()
    set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_PROCESSOR}")
endif()
message(STATUS "  CPack system name (kiến trúc gói): ${CPACK_SYSTEM_NAME}")

# Chỉ dựng .deb khi thật sự có dpkg — để `cmake --build` không chết trên
# máy không cài dpkg (ví dụ macOS, container tối giản).
find_program(DPKG_DEB_EXECUTABLE dpkg-deb)
if(DPKG_DEB_EXECUTABLE)
    set(CPACK_GENERATOR "DEB;TGZ")
else()
    set(CPACK_GENERATOR "TGZ")
    message(STATUS "  dpkg-deb không có — chỉ đóng gói TGZ, bỏ qua .deb")
endif()

# ---- Nén cây cài đặt ----
set(CPACK_TGZ_GENERATOR_FORMAT "gz")
set(CPACK_ARCHIVE_COMPONENT_INSTALL OFF)

# ---- Gói mã nguồn ----
# Mặc định CMake sinh .tar.Z (nén kiểu cũ). Ép .tar.gz vì đó là định dạng
# mà mọi người và mọi công cụ giải nén đều biết xử lý.
# Tên file KHÔNG kèm đuôi — CPack tự thêm, nếu thêm sẽ ra ".tar.gz.tar.gz".
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "canopen-${PROJECT_VERSION}-Source")
# Không nhét build/ và .git/ vào gói nguồn
set(CPACK_SOURCE_IGNORE_FILES
    "/build/" "/\\\\.git/" "/\\\\.github/" "\\\\.tar\\\\.gz$" "\\\\.deb$")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)

# Ghi file CPackConfig.cmake + CPackSourceConfig.cmake — phải sau cùng
include(CPack)

# Print configuration
message(STATUS "CANopen Library Configuration")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Build shared libraries: ${BUILD_SHARED_LIBS}")
message(STATUS "  Build tests: ${BUILD_TESTING}")
message(STATUS "  Build examples: ${BUILD_EXAMPLES}")
