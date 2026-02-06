cmake_minimum_required(VERSION 3.10)
project(px4_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MAVSDK
find_package(MAVSDK REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

# PX4 MAVLink Interface executable
add_executable(px4_mavlink_interface
    src/px4_mavlink_interface.cpp
)

target_link_libraries(px4_mavlink_interface
    MAVSDK::mavsdk
    pthread
    rt  # Required for POSIX shared memory and semaphores
)

# Install targets
install(TARGETS px4_mavlink_interface
    RUNTIME DESTINATION bin
)
