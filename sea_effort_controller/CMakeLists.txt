cmake_minimum_required(VERSION 2.8.3)
project(sea_effort_controller)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)



find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
  roscpp
  std_msgs
  sensor_msgs

)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sea_effort_controller
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface pluginlib roscpp sensor_msgs std_msgs
  DEPENDS
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/sea_effort_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
