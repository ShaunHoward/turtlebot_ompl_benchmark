cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_rrt)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  occupancy_grid_utils
  costmap_2d
  nav_core
  base_local_planner
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OMPL REQUIRED)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    include
)

add_library(rrt_lib src/rrt.cpp)

target_link_libraries(rrt_lib ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
