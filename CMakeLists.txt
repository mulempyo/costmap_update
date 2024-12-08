cmake_minimum_required(VERSION 3.0.2)
project(costmap_update)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  roscpp
  sensor_msgs
  tf2
  nav_msgs
  dwa_planner_ros
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES costmap_update
   CATKIN_DEPENDS costmap_2d geometry_msgs roscpp sensor_msgs tf2 nav_msgs dwa_planner_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/costmap_update.cpp
)

add_executable(update_map src/costmap_update.cpp)

target_link_libraries(update_map
  ${catkin_LIBRARIES}
  dwa_planner_ros
)

add_dependencies(update_map ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})

