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
			IridiumSession.msg
			IridiumStatus.msg
	)

	generate_messages()

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

#if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")

add_executable(test_decode_mission
  src/test_decode_mission.cpp
  src/logData.cpp
  src/missionxml.cpp
  )
add_dependencies(test_decode_mission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_decode_mission ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})

# add_executable(test_decode_log_state src/test_decode_log_state.cpp ${SRC})
# add_dependencies(test_decode_log_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_decode_log_state ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})

# add_executable(test_generate_cmd_sleep src/test_generate_cmd_sleep.cpp ${SRC})
# add_dependencies(test_generate_cmd_sleep ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_generate_cmd_sleep ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})

# add_executable(test_generate_cmd_parameters src/test_generate_cmd_parameters.cpp ${SRC})
# add_dependencies(test_generate_cmd_parameters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_generate_cmd_parameters ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})

#endif()

#add_executable(test_checksum src/test.cpp ${SRC})
#add_dependencies(test_checksum ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_checksum ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
