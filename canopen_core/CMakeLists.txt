# CANopen Core Library
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
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../generated>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(canopen
    PUBLIC
        Threads::Threads
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(canopen PUBLIC pthread)
endif()

install(TARGETS canopen
    EXPORT canopenTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/canopen
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
