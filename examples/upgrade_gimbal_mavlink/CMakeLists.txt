project(upgrade_gimbal_mavlink)
cmake_minimum_required(VERSION 3.15)

# Specify at least C++11, better C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MAVLINK_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src/mavlink/include/mavlink/v2.0")

set(MAVLINK_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/parsers
                    ${MAVLINK_INCLUDES}
                    ${MAVLINK_SRC}
                    )



set(UPGRADE_GIMBAL_MAVLINK_SRC
    upgrade_gimbal_mavlink.cpp
    ${MAVLINK_SRC}/parsers/utils.cpp
    ${MAVLINK_SRC}/parsers/binary.c
    ${MAVLINK_SRC}/parsers/crc32.c
    ${MAVLINK_SRC}/parsers/parse.cpp
    ${MAVLINK_SRC}/bootloader_mavlink.cpp
    ${MAVLINK_SRC}/serial_port.cpp
)
add_executable(upgrade_gimbal_mavlink ${UPGRADE_GIMBAL_MAVLINK_SRC})

find_package( Threads )
find_package(jsoncpp CONFIG REQUIRED)

target_link_libraries(upgrade_gimbal_mavlink
                    jsoncpp_lib
                    ${CMAKE_THREAD_LIBS_INIT}
                    )