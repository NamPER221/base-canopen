# ZLAC8015D Dual-Axis Servo Driver Library (built on canopen core)

set(ZLAC_SOURCES
    src/drivers/zlac8015/zlac8015_driver.cpp
)

add_library(canopen_zlac ${ZLAC_SOURCES})

target_include_directories(canopen_zlac
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(canopen_zlac
    PUBLIC
        canopen
        canopen_kinematics
)

target_compile_features(canopen_zlac PUBLIC cxx_std_20)

install(TARGETS canopen_zlac
    EXPORT canopenTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/canopen
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
