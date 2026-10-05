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

# Generate headers
include(GNUInstallDirs)
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/canopen/version.hpp"
    @ONLY
)

# Thư viện lõi
add_subdirectory(canopen_core)

# ==================== Optional device driver libraries ====================
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
    enable_testing()
    add_subdirectory(test)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Installation
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/canopen/version.hpp"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/canopen
)

install(EXPORT canopenTargets
    FILE canopenTargets.cmake
    NAMESPACE canopen::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/canopen
)

# Config file
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

export(TARGETS canopen
    FILE "${CMAKE_CURRENT_BINARY_DIR}/canopenTargets.cmake"
)

# pkg-config file
set(PC_FILE "${CMAKE_CURRENT_BINARY_DIR}/canopen.pc")
file(WRITE ${PC_FILE}
"prefix=\${pcfiledir}/../..\n"
"libdir=\${prefix}/${CMAKE_INSTALL_LIBDIR}\n"
"includedir=\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}\n"
"\n"
"Name: canopen\n"
"Description: CANopen stack cho robot\n"
"Version: ${PROJECT_VERSION}\n"
"Requires:\n"
"Cflags: -I\${includedir}\n"
"Libs: -L\${libdir} -lcanopen\n"
)
install(FILES ${PC_FILE}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# =============================================================================
# Packaging — CPack
# =============================================================================
set(CPACK_PACKAGE_NAME "canopen")
set(CPACK_PACKAGE_VENDOR "base-canopen")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Thư viện CANopen C++20")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/NamPER221/base-canopen")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
endif()

set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_DEBIAN_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "base-canopen <base-canopen@users.noreply.github.com>")
set(CPACK_PACKAGE_CONTACT "base-canopen")
set(CPACK_PACKAGE_DESCRIPTION_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/package-description.txt")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS "OFF")
set(CPACK_DEB_COMPONENT_INSTALL ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(CPACK_SYSTEM_NAME "arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(CPACK_SYSTEM_NAME "amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(armv7l|armhf)$")
    set(CPACK_SYSTEM_NAME "armhf")
else()
    set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_program(DPKG_DEB_EXECUTABLE dpkg-deb)
if(DPKG_DEB_EXECUTABLE)
    set(CPACK_GENERATOR "DEB;TGZ")
else()
    set(CPACK_GENERATOR "TGZ")
endif()

set(CPACK_TGZ_GENERATOR_FORMAT "gz")
set(CPACK_ARCHIVE_COMPONENT_INSTALL OFF)

set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "canopen-${PROJECT_VERSION}-Source")
set(CPACK_SOURCE_IGNORE_FILES
    "/build/" "/\\\\.git/" "/\\\\.github/" "\\\\.tar\\\\.gz$" "\\\\.deb$")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)

include(CPack)
