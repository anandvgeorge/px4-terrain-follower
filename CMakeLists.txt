cmake_minimum_required(VERSION 3.10)
project(terrain_follower)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add subdirectories
add_subdirectory(px4_interface)
add_subdirectory(control)
add_subdirectory(debug)
