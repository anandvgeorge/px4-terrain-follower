cmake_minimum_required(VERSION 3.10)

# Find BehaviorTree.CPP
set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/third_party/behaviortree" ${CMAKE_PREFIX_PATH})
find_package(behaviortree_cpp REQUIRED)

# Find MAVSDK
find_package(MAVSDK REQUIRED)

# Control library with basic modes and offboard mode
add_library(control_node
    src/basic_modes.cpp
    src/offboard_mode.cpp
)

target_include_directories(control_node PUBLIC
    include
    ${CMAKE_SOURCE_DIR}/common
)

target_link_libraries(control_node
    pthread
    rt
)

# BT Main executable
add_executable(bt_main
    src/bt_main.cpp
)

target_include_directories(bt_main PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/third_party/behaviortree/include
)

target_link_libraries(bt_main
    control_node
    ${CMAKE_SOURCE_DIR}/third_party/behaviortree/lib/libbehaviortree_cpp.so
    pthread
)

set_target_properties(bt_main PROPERTIES
    INSTALL_RPATH "${CMAKE_SOURCE_DIR}/third_party/behaviortree/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# Copy BT XML file to build directory
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/autonomy.xml
    ${CMAKE_CURRENT_BINARY_DIR}/behavior_trees/autonomy.xml
    COPYONLY
)
