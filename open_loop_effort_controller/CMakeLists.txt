cmake_minimum_required(VERSION 2.8.3)
project(open_loop_effort_controller)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)



find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  subscription_notifier
  pluginlib
  roscpp
  std_msgs

  sensor_msgs

)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES open_loop_effort_controller
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface  subscription_notifier pluginlib roscpp sensor_msgs std_msgs
  DEPENDS
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/open_loop_effort_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
