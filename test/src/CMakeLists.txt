# Test source CMakeLists.txt
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/test/include
)

# Enable testing
enable_testing()

# Object Dictionary Tests
add_executable(test_object_dictionary test_object_dictionary.cpp)
target_link_libraries(test_object_dictionary canopen Threads::Threads)
add_test(NAME test_object_dictionary COMMAND test_object_dictionary)

# Message Factory Tests
add_executable(test_message_factory test_message_factory.cpp)
target_link_libraries(test_message_factory canopen Threads::Threads)
add_test(NAME test_message_factory COMMAND test_message_factory)

# Frame Tests
add_executable(test_frame test_frame.cpp)
target_link_libraries(test_frame canopen Threads::Threads)
add_test(NAME test_frame COMMAND test_frame)

# Motor Control Tests (ZLAC8015D)
add_executable(test_motor_control test_motor_control.cpp)
target_link_libraries(test_motor_control canopen Threads::Threads)
add_test(NAME test_motor_control COMMAND test_motor_control)

# Kinematics Tests (No hardware required)
add_executable(test_kinematics test_kinematics.cpp)
target_link_libraries(test_kinematics Threads::Threads)
add_test(NAME test_kinematics COMMAND test_kinematics)

# Optional GTest based tests
find_package(GTest QUIET)
if(GTest_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_object_dictionary_gtest.cpp")
    message(STATUS "GTest found - building GTest tests")
    add_executable(canopen_gtest_tests test_object_dictionary_gtest.cpp)
    target_link_libraries(canopen_gtest_tests
        canopen
        GTest::GTest
        GTest::Main
        Threads::Threads
    )
    add_test(NAME canopen_gtest_tests COMMAND canopen_gtest_tests)
endif()
