cmake_minimum_required(VERSION 2.8.3)
project(groundgrid)

# use c++17
SET(CMAKE_CXX_STANDARD 17)

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    nodelet
    roscpp
    geometry_msgs
    pcl_ros
    pcl_conversions    
    grid_map_core
    grid_map_ros
    grid_map_cv
    grid_map_filters
    grid_map_loader
    grid_map_msgs
    grid_map_rviz_plugin
    grid_map_visualization
    cv_bridge
    image_transport
    std_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_msgs
    message_generation
)

################################
## include common CMake setup ##
################################
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

#######################################
## dynamic_reconfigure configuration ##
#######################################

generate_dynamic_reconfigure_options(
    cfg/GroundGrid.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
    FILES
    NextCloud.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        dynamic_reconfigure
        nodelet
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    include
)


#####################################################
## Build library, node and nodelet of this package ##
#####################################################

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
    src/GroundGrid.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}_lib
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_groundsegmentation_lib
	src/GroundSegmentation.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}_groundsegmentation_lib
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_groundsegmentation_lib
    ${catkin_LIBRARIES}
)

## determines target name as lib[name].so, here: libfub_dummy_nodelet.so
## the library path in nodelet_plugins.xml refers to this file
add_library(${PROJECT_NAME}_nodelet
    src/GroundGridNodelet.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet
    ${PROJECT_NAME}_gencfg
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_groundsegmentation_lib
)

target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_groundsegmentation_lib
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
    src/GroundGridNode.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelet
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
    TARGETS
        ${PROJECT_NAME}_lib
        ${PROJECT_NAME}_groundsegmentation_lib
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

