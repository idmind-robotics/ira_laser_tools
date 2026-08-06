cmake_minimum_required(VERSION 3.5)
project(ira_laser_tools)

# Default to C99
set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  sensor_msgs
  laser_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  rcl_interfaces
  pcl_ros
  Eigen3
  PCL
)


# bugfix for g++-Warning
#   "<command-line>:0:0: warning: missing whitespace after the macro name"
remove_definitions(-DDISABLE_LIBUSB-1.0)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_library(ira_laser_tools_components SHARED
  src/scan_projection.cpp
  src/laserscan_multi_merger.cpp
  src/laserscan_virtualizer.cpp
)
target_include_directories(ira_laser_tools_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ira_laser_tools_components ${PCL_LIBRARIES})
ament_target_dependencies(ira_laser_tools_components ${dependencies})

rclcpp_components_register_node(ira_laser_tools_components
  PLUGIN "ira_laser_tools::LaserscanMerger"
  EXECUTABLE laserscan_multi_merger
)
rclcpp_components_register_node(ira_laser_tools_components
  PLUGIN "ira_laser_tools::LaserscanVirtualizer"
  EXECUTABLE laserscan_virtualizer
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ira_laser_tools_components
  EXPORT export_ira_laser_tools_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS laserscan_multi_merger laserscan_virtualizer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/ config/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(ira_laser_tools_components)
ament_export_dependencies(${dependencies})
ament_package()
