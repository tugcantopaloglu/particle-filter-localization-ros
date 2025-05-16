cmake_minimum_required(VERSION 3.0.2)
project(particle_pf)
add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf
    gazebo_msgs
    visualization_msgs
    roslib
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs tf gazebo_msgs visualization_msgs roslib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${roslib_INCLUDE_DIRS}
)

file(GLOB SRC src/*.cpp)

add_executable(particle_filter_node ${SRC})

target_link_libraries(particle_filter_node
    ${catkin_LIBRARIES}
    ${roslib_INCLUDE_DIRS}
)
