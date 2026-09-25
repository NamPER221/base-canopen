cmake_minimum_required(VERSION 3.16)
project(canopen VERSION 1.0.0 LANGUAGES CXX)

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

if(BUILD_KINEMATICS)
    add_subdirectory(drivers/kinematics)
endif()

if(BUILD_ZLAC_DRIVER)
    add_subdirectory(drivers/zlac8015)
endif()

# Subdirectories
if(BUILD_TESTING)
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

install(EXPORT canopenTargets
    FILE canopenTargets.cmake
    NAMESPACE canopen::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/canopen
)

# Export package
export(TARGETS canopen
    FILE "${CMAKE_CURRENT_BINARY_DIR}/canopenTargets.cmake"
)

# pkg-config file
set(PC_FILE "${CMAKE_CURRENT_BINARY_DIR}/canopen.pc")
file(WRITE ${PC_FILE}
"prefix=${CMAKE_INSTALL_PREFIX}\n"
"exec_prefix=${CMAKE_INSTALL_PREFIX}/bin\n"
"libdir=${CMAKE_INSTALL_LIBDIR}\n"
"includedir=${CMAKE_INSTALL_INCLUDEDIR}\n"
"\n"
"Name: canopen\n"
"Description: CANopen Stack Library\n"
"Version: ${PROJECT_VERSION}\n"
"Requires:\n"
"Cflags: -I\${includedir}\n"
"Libs: -L\${libdir} -lcanopen\n"
)
install(FILES ${PC_FILE}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# Print configuration
message(STATUS "CANopen Library Configuration")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Build shared libraries: ${BUILD_SHARED_LIBS}")
message(STATUS "  Build tests: ${BUILD_TESTING}")
message(STATUS "  Build examples: ${BUILD_EXAMPLES}")
