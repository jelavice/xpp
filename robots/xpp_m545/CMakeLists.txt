cmake_minimum_required(VERSION 2.8.3)
project(xpp_m545)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  xpp_vis
  m545_description
)

###################################
## catkin specific configuration ##
###################################
catkin_package()


###########
## Build ##
###########
## Specify additional locations of header files
#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)
#
# Declare a C++ library
#add_library(${PROJECT_NAME}
#  src/hyqleg_inverse_kinematics.cc
#  src/inverse_kinematics_hyq1.cc
#  src/inverse_kinematics_hyq2.cc
#  src/inverse_kinematics_hyq4.cc
#)
#
## URDF visualizers for all HyQ variants
#add_executable(urdf_visualizer_hyq1 src/exe/urdf_visualizer_hyq1.cc)
#target_link_libraries(urdf_visualizer_hyq1
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)
#
#add_executable(urdf_visualizer_hyq2 src/exe/urdf_visualizer_hyq2.cc)
#target_link_libraries(urdf_visualizer_hyq2
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)
#
#add_executable(urdf_visualizer_hyq4 src/exe/urdf_visualizer_hyq4.cc)
#target_link_libraries(urdf_visualizer_hyq4
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)
#
#)
