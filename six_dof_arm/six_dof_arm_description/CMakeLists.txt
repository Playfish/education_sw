cmake_minimum_required(VERSION 2.8.3)
project(six_dof_arm_description)

find_package(catkin REQUIRED COMPONENTS
  rviz
  urdf
  xacro
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


