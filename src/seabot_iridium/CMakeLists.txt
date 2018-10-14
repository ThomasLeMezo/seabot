cmake_minimum_required(VERSION 2.8.3)
project(seabot_iridium)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fopenmp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    seabot_fusion
    seabot_power_driver
    gpsd_client
    pressure_bme280_driver
    seabot_safety
    std_srvs
				message_generation
)

find_package(OpenMP REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
	add_message_files(
			FILES
			IridiumLog.msg
			IridiumSession.msg
			IridiumStatus.msg
	)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
	generate_messages(
#			DEPENDENCIES
#			std_msgs  # Or other packages containing msgs
	)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
    INCLUDE_DIRS include
    # LIBRARIES TIS
    CATKIN_DEPENDS roscpp seabot_fusion seabot_power_driver gpsd_client pressure_bme280_driver seabot_safety
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

set(SRC
 src/logData.cpp
 src/missionxml.cpp
 src/sbd.cpp
 src/AsyncSerial.cpp
 src/BufferedAsyncSerial.cpp
)

add_executable(iridium_node
    src/main.cpp
    ${SRC}
    )

add_dependencies(iridium_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(iridium_node
    ${catkin_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
    )

if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")


 add_executable(decode_iridium
     src/test.cpp
     ${SRC}
     )

 add_dependencies(decode_iridium ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(decode_iridium
     ${catkin_LIBRARIES}
     ${OpenMP_CXX_FLAGS}
     )
endif()
