cmake_minimum_required(VERSION 3.22)
project(groundgrid)

######################################
## Find catkin macros and libraries ##
######################################

set (CMAKE_CXX_STANDARD 11)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(grid_map_visualization REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(filters REQUIRED)

################################
## include common CMake setup ##
################################
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

# Compiler flags
# Do not warn for unused returned values
add_compile_options(-Wno-unused-result)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

set(DEPS
  rclpy
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
  sensor_msgs      
  grid_map_msgs
  grid_map
  grid_map_core 
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_visualization
  cv_bridge
  image_transport
  visualization_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
  filters
  pluginlib
)


include_directories(include)

add_executable(${PROJECT_NAME}_node
    src/GroundGridNode.cpp
    src/GroundGrid.cpp
    src/GroundSegmentation.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
    ${DEPS}
    PCL
    Eigen3
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
    ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib/${PROJECT_NAME} 
    LIBRARY DESTINATION lib/${PROJECT_NAME} 
    RUNTIME DESTINATION lib/${PROJECT_NAME} 
)

install(PROGRAMS
    scripts/eval_groundpoint_classifier.py
    DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY param
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
