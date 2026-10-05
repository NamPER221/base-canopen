# Differential Drive Kinematics Library

set(KINEMATICS_SOURCES
    src/kinematics/differential_drive.cpp
)

add_library(canopen_kinematics ${KINEMATICS_SOURCES})

target_include_directories(canopen_kinematics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(canopen_kinematics
    PUBLIC
        canopen
)

target_compile_features(canopen_kinematics PUBLIC cxx_std_20)

install(TARGETS canopen_kinematics
    EXPORT canopenTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/canopen
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
