# ============================================================
# Test structure:
#   test/src   - Core library unit tests (FakeBus, no hardware)
#   test/zlac  - Hardware tests with ZLAC8015D motor (can0/vcan0)
# ============================================================

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/test/include
)

enable_testing()

# ==================== Core Unit Tests (no hardware) ====================

# Object Dictionary Tests
add_executable(test_object_dictionary src/test_object_dictionary.cpp)
target_link_libraries(test_object_dictionary canopen Threads::Threads)
add_test(NAME test_object_dictionary COMMAND test_object_dictionary)

# Message Factory Tests
add_executable(test_message_factory src/test_message_factory.cpp)
target_link_libraries(test_message_factory canopen Threads::Threads)
add_test(NAME test_message_factory COMMAND test_message_factory)

# Frame Tests
add_executable(test_frame src/test_frame.cpp)
target_link_libraries(test_frame canopen Threads::Threads)
add_test(NAME test_frame COMMAND test_frame)

# SDO Protocol Tests (client/server over FakeBus)
add_executable(test_sdo src/test_sdo.cpp)
target_link_libraries(test_sdo canopen Threads::Threads)
add_test(NAME test_sdo COMMAND test_sdo)

# EDS Parser Tests (real ZLAC8015D.eds file)
add_executable(test_eds src/test_eds.cpp)
target_link_libraries(test_eds canopen Threads::Threads)
add_test(NAME test_eds COMMAND test_eds
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# NMT Service Tests (over FakeBus)
add_executable(test_nmt src/test_nmt.cpp)
target_link_libraries(test_nmt canopen Threads::Threads)
add_test(NAME test_nmt COMMAND test_nmt)

# PDO Tests (over FakeBus)
add_executable(test_pdo src/test_pdo.cpp)
target_link_libraries(test_pdo canopen Threads::Threads)
add_test(NAME test_pdo COMMAND test_pdo)

# Kinematics Tests (uses canopen_kinematics library, no hardware)
if(BUILD_KINEMATICS)
    add_executable(test_kinematics src/test_kinematics.cpp)
    target_link_libraries(test_kinematics canopen_kinematics Threads::Threads)
    add_test(NAME test_kinematics COMMAND test_kinematics)
endif()

# ==================== Hardware Tests (ZLAC8015D motor) ====================
# These require a real CAN interface (can0) or vcan0 with the drive attached.

# YAML config support for hardware tests
find_library(YAMLCPP_LIB yaml-cpp)
if(YAMLCPP_LIB)
    set(YAML_LIBS ${YAMLCPP_LIB})
    set(HAVE_YAML 1)
else()
    set(YAML_LIBS "")
    set(HAVE_YAML 0)
endif()

if(BUILD_ZLAC_DRIVER)
    add_executable(test_motor_control zlac/test_motor_control.cpp)
    target_link_libraries(test_motor_control canopen_zlac canopen_kinematics ${YAML_LIBS} Threads::Threads)
    target_compile_definitions(test_motor_control PRIVATE HAVE_YAML=${HAVE_YAML})
    # NOT registered with CTest by default (requires hardware);
    # run manually: ./test_zlac/test_motor_control can0 1
endif()

# ==================== Optional GTest based tests ====================

find_package(GTest QUIET)
if(GTest_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/test_object_dictionary_gtest.cpp")
    message(STATUS "GTest found - building GTest tests")
    add_executable(canopen_gtest_tests src/test_object_dictionary_gtest.cpp)
    target_link_libraries(canopen_gtest_tests
        canopen
        GTest::GTest
        GTest::Main
        Threads::Threads
    )
    add_test(NAME canopen_gtest_tests COMMAND canopen_gtest_tests)
endif()
