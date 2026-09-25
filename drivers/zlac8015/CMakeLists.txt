# ZLAC8015D Dual-Axis Servo Driver Library (built on canopen core)

set(ZLAC_SOURCES
    ../../src/drivers/zlac8015/zlac8015_driver.cpp
)

add_library(canopen_zlac ${ZLAC_SOURCES})

target_include_directories(canopen_zlac
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(canopen_zlac
    PUBLIC
        canopen
)

target_compile_features(canopen_zlac PUBLIC cxx_std_20)
