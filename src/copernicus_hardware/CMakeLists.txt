cmake_minimum_required(VERSION 2.8.3)
project(copernicus_hardware)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  copernicus_msgs
  tf
  controller_manager
  hardware_interface
  diagnostic_updater
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS hardware_interface copernicus_msgs roscpp sensor_msgs
DEPENDS Boost )

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

add_library(tableToRos
	src/tableToRos.cpp
)

add_library(queue
	src/queue.cpp
)

add_library(conversion
	src/conversion.cpp
)
add_library(copernicusDataStruct
	src/copernicusDataStruct.cpp
)


add_executable(serial_node src/serialMain.cpp)
target_link_libraries(serial_node ${catkin_LIBRARIES})
target_link_libraries(serial_node tableToRos queue copernicusDataStruct conversion ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(copernicus_hardware src/copernicus_hardware.cpp)
add_executable(copernicus_base src/copernicus_base.cpp)
target_link_libraries(copernicus_base copernicus_hardware ${catkin_LIBRARIES} ${Boost_LIBRARIES})
