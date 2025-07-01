cmake_minimum_required(VERSION 3.15)
project(upgrade_gimbal)
# Specify at least C++11, better C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MAVLINK_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/mavlink/include/mavlink/v2.0")

set(MAVLINK_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src")


include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/src/parsers
                    ${MAVLINK_INCLUDES}
                    ${MAVLINK_SRC}
                    )


set(UPGRADE_GIMBAL_SRC
    upgrade_gimbal.cpp
    ${MAVLINK_SRC}/parsers/utils.cpp
    ${MAVLINK_SRC}/bootloader.cpp
    ${MAVLINK_SRC}/gimbal_interface.cpp
    ${MAVLINK_SRC}/gimbal_protocol.cpp
    ${MAVLINK_SRC}/gimbal_protocol_v1.cpp
    ${MAVLINK_SRC}/gimbal_protocol_v2.cpp
    ${MAVLINK_SRC}/serial_port.cpp
)
add_executable(upgrade_gimbal ${UPGRADE_GIMBAL_SRC})

find_package( Threads )
target_link_libraries(upgrade_gimbal
                    ${CMAKE_THREAD_LIBS_INIT}
                    )