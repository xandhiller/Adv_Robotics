cmake_minimum_required(VERSION 2.8.12)

project(astar_tutorial)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    roslib
    nav_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/astar_tutorial.cpp
    src/astar.cpp
    src/occupancy_graph.cpp
    src/open_list.cpp
    src/extended_list.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

