cmake_minimum_required(VERSION 2.8.3)
project(serial_port_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Boost 1.54.0 COMPONENTS system filesystem thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES seriral_port_driver
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/handle.cpp src/qserial.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES} )
