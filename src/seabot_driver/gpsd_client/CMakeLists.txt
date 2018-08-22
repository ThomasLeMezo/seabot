cmake_minimum_required(VERSION 2.8.3)
project(gpsd_client CXX)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

############
## Catkin ##
############

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
)

find_package(PkgConfig)
pkg_check_modules (libgps REQUIRED libgps)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    GPSStatus.msg
    GPSFix.msg
)

generate_messages(DEPENDENCIES 
  std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    message_runtime
    ${DEPS}
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${libgps_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}
    src/client.cpp
    src/gpsdclient.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${libgps_LIBRARIES})

