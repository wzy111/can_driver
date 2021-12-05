cmake_minimum_required(VERSION 3.0.2)
project(urexo_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/urexo_control.cfg
)

catkin_package(

)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(urexo_control
  src/urexo_control.cpp
  src/RMDx_Motor.cpp
  src/canalystii.cpp

  include/RMD/controlcan.h
  include/RMD/canalystii.h
  include/RMD/RMDx_Motor.h
  include/RMD/can.h
)
add_dependencies(urexo_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(urexo_control
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so
)
