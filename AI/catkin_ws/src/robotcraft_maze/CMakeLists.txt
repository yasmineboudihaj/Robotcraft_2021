cmake_minimum_required(VERSION 2.8.3)
project(robotcraft_maze)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(maze_wallFollower src/maze_wallFollower.cpp)
add_executable(maze_BASICsolver  src/maze_BASICsolver.cpp)
add_executable(maze_PROsolver    src/maze_PROsolver.cpp)
add_executable(maze_noOdom       src/maze_noOdom.cpp)

target_link_libraries(maze_wallFollower ${catkin_LIBRARIES})
target_link_libraries(maze_BASICsolver  ${catkin_LIBRARIES})
target_link_libraries(maze_PROsolver    ${catkin_LIBRARIES})
target_link_libraries(maze_noOdom       ${catkin_LIBRARIES})