cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_ompl_benchmark)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
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
    include
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

add_library(omplbenchmarkplanner_lib src/ompl_benchmark.cpp)

target_link_libraries(omplbenchmarkplanner_lib ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
