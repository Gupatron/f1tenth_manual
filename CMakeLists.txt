cmake_minimum_required(VERSION 3.10)
project(RoverControl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Zenoh C and C++
# zenoh-cpp requires zenoh-c to be installed
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

# Find VRPN for Vicon communication
find_package(PkgConfig)
pkg_check_modules(VRPN REQUIRED vrpn)

include_directories(${VRPN_INCLUDE_DIRS})

# Include directories
include_directories(${CMAKE_SOURCE_DIR})

# Base station executable
add_executable(base
    base.cpp
)

target_link_libraries(base
    zenohcxx::zenohcxx
    zenohc::lib
    pthread
)

# Pure Pursuit base station executable
add_executable(base_purepursuit
    base_purepursuit.cpp
)

target_link_libraries(base_purepursuit
    zenohcxx::zenohcxx
    zenohc::lib
    pthread
)

# Vicon-based Pure Pursuit with PID controller
add_executable(base_vicon_purepursuit
    base_vicon_purepursuit.cpp
)

target_link_libraries(base_vicon_purepursuit
    zenohcxx::zenohcxx
    zenohc::lib
    ${VRPN_LIBRARIES}
    pthread
)

target_include_directories(base_vicon_purepursuit PRIVATE ${VRPN_INCLUDE_DIRS})

# Rover executable
add_executable(rover
    rover.cpp
)

target_link_libraries(rover
    zenohcxx::zenohcxx
    zenohc::lib
    pthread
)

# Optional: Add install targets
install(TARGETS base base_purepursuit base_vicon_purepursuit rover
    RUNTIME DESTINATION bin
)

# Print build information
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")