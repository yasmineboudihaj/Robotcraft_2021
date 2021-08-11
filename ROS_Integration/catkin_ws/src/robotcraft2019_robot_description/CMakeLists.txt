cmake_minimum_required(VERSION 2.8.3)
project(robotcraft2019_robot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

