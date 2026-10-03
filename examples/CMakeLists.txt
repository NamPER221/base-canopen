# Find YAML library for config
find_library(YAMLCPP_LIB yaml-cpp)
if(YAMLCPP_LIB)
    set(YAML_LIBS ${YAMLCPP_LIB})
    set(HAVE_YAML 1)
    message(STATUS "yaml-cpp found - YAML config enabled")
else()
    set(YAML_LIBS "")
    set(HAVE_YAML 0)
    message(STATUS "yaml-cpp not found - using hardcoded config")
endif()

add_executable(master_example master_example.cpp)
target_link_libraries(master_example canopen pthread)

add_executable(dual_motor_control dual_motor_control.cpp)
target_link_libraries(dual_motor_control canopen canopen_kinematics pthread)

add_executable(dual_motor_keyboard dual_motor_keyboard.cpp)
target_link_libraries(dual_motor_keyboard canopen canopen_kinematics pthread)

add_executable(robot_move robot_move.cpp)
target_link_libraries(robot_move canopen_zlac canopen_kinematics ${YAML_LIBS} pthread)
target_compile_definitions(robot_move PRIVATE
    HAVE_YAML=${HAVE_YAML}
    CANOPEN_DEFAULT_CONFIG="${CMAKE_SOURCE_DIR}/config/zlac8015_config.yaml")

add_executable(keyboard_control keyboard_control.cpp)
target_link_libraries(keyboard_control canopen_zlac canopen_kinematics ${YAML_LIBS} pthread)
target_compile_definitions(keyboard_control PRIVATE
    HAVE_YAML=${HAVE_YAML}
    CANOPEN_DEFAULT_CONFIG="${CMAKE_SOURCE_DIR}/config/zlac8015_config.yaml")

# Demo lớp cấp cao MotorDevice (EDS-driven, không cần driver riêng)
add_executable(motor_device_demo motor_device_demo.cpp)
target_link_libraries(motor_device_demo canopen Threads::Threads)
target_compile_definitions(motor_device_demo PRIVATE
    CANOPEN_DEFAULT_EDS="${CMAKE_SOURCE_DIR}/ZLAC8015D.eds")

add_executable(can_probe can_probe.cpp)
target_link_libraries(can_probe canopen ${YAML_LIBS} Threads::Threads)
target_compile_definitions(can_probe PRIVATE
    HAVE_YAML=${HAVE_YAML}
    CANOPEN_DEFAULT_CONFIG="${CMAKE_SOURCE_DIR}/config/zlac8015_config.yaml")

add_executable(pdo_test pdo_test.cpp)
target_link_libraries(pdo_test canopen_zlac canopen_kinematics ${YAML_LIBS} Threads::Threads)

if(BUILD_MBDV_DRIVER)
    add_executable(mbdv_diff_drive mbdv_diff_drive.cpp)
    target_link_libraries(mbdv_diff_drive canopen_mbdv canopen_kinematics canopen Threads::Threads)

    add_executable(mbdv_teleop_keyboard mbdv_teleop_keyboard.cpp)
    target_link_libraries(mbdv_teleop_keyboard canopen_mbdv canopen_kinematics canopen Threads::Threads)
endif()

