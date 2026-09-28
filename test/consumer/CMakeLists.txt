cmake_minimum_required(VERSION 3.16)
project(canopen_consumer LANGUAGES CXX)

# =============================================================================
# Project mẫu kiểm chứng bản cài đặt của thư viện.
#
# Nếu file này build được, nghĩa là:
#   - canopenConfig.cmake tìm thấy và parse được
#   - header đã cài đúng chỗ và target truyền đúng include dir
#   - lib đã cài và liên kết được
#   - các class public dùng được từ bên ngoài
#
# Đây là bài kiểm tra quan trọng nhất cho một thư viện C++ vì lỗi đóng gói
# (thiếu file Config, sai include path, thiếu phụ thu) chỉ lộ ra khi có
# project BÊN NGOÀI tiêu thụ — build tại chỗ thì không bao giờ thấy.
# =============================================================================

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(canopen REQUIRED)

message(STATUS "canopen ${canopen_VERSION} (config: ${canopen_DIR})")
message(STATUS "  kinematics: ${CANOPEN_HAS_KINEMATICS}")
message(STATUS "  zlac driver: ${CANOPEN_HAS_ZLAC_DRIVER}")

add_executable(consumer consumer.cpp)
target_link_libraries(consumer PRIVATE canopen::canopen)

# Driver và kinematics là target tuỳ chọn — chỉ kiểm tra nếu bản cài có
if(CANOPEN_HAS_KINEMATICS)
    add_executable(consumer_kin consumer_kin.cpp)
    target_link_libraries(consumer_kin PRIVATE canopen::canopen_kinematics)
endif()

enable_testing()
add_test(NAME consumer_runs COMMAND consumer)
if(CANOPEN_HAS_KINEMATICS)
    add_test(NAME consumer_kin_runs COMMAND consumer_kin)
endif()
