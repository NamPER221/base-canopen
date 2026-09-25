# Differential Drive Kinematics Library (CANopen-independent)

set(KINEMATICS_SOURCES
    ../../src/kinematics/differential_drive.cpp
)

add_library(canopen_kinematics ${KINEMATICS_SOURCES})

target_include_directories(canopen_kinematics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(canopen_kinematics PUBLIC cxx_std_20)
