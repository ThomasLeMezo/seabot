cmake_minimum_required(VERSION 2.8.3)
project(seabot_depth_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  rospy
  message_generation
  seabot_fusion
  std_srvs
  pressure_bme280_driver
)

find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${Eigen3_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

 add_message_files(
   FILES
   RegulationDebug.msg
 )

generate_messages(
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp std_srvs pressure_bme280_driver
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(depth_controller
    src/state_feedback.cpp
)

add_dependencies(depth_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(depth_controller
  ${catkin_LIBRARIES}
)
