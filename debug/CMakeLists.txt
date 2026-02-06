cmake_minimum_required(VERSION 3.10)
project(debug)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MAVSDK
find_package(MAVSDK REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

# State Reader utility
add_executable(state_reader
    src/state_reader.cpp
)

target_link_libraries(state_reader
    pthread
    rt
)

# Install targets
install(TARGETS state_reader
    RUNTIME DESTINATION bin
)
