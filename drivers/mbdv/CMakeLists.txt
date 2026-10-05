# Moons' MBDV Dual-Axis Servo Driver Library (built on canopen core)

set(MBDV_SOURCES
    src/drivers/mbdv/mbdv_driver.cpp
)

add_library(canopen_mbdv ${MBDV_SOURCES})

target_include_directories(canopen_mbdv
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(canopen_mbdv
    PUBLIC
        canopen
        canopen_kinematics
)

target_compile_features(canopen_mbdv PUBLIC cxx_std_20)

install(TARGETS canopen_mbdv
    EXPORT canopenTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/canopen
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
