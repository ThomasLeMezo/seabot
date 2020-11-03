cmake_minimum_required(VERSION 2.8.3)
project(seabot_log)

#if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64" AND $ENV{USER} STREQUAL "lemezoth")
# message("SEABOT_LOG = " ${CMAKE_HOST_SYSTEM_PROCESSOR} " " $ENV{USER})

#add_compile_options(-std=c++11)

################ FIND PACKAGES ###############

#find_package(catkin REQUIRED
#    roscpp
#    rosbag
#    seabot_fusion
#    seabot_mission
#)

#find_package(ZLIB REQUIRED)

#find_package(ipegenerator REQUIRED)
#message(STATUS "IPEGENERATOR_INCLUDE_DIRS=" ${IPEGENERATOR_INCLUDE_DIRS})
#message(STATUS "IPEGENERATOR_LIBRARIES=" ${IPEGENERATOR_LIBRARIES})

#find_package(Boost REQUIRED COMPONENTS system thread filesystem)
#include_directories(${Boost_INCLUDE_DIR})

#find_package(OpenMP)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/)
#if(NOT IBEX_ROOT)
# set(IBEX_ROOT "$ENV{HOME}/ibex")
#endif()

#set(IbexLib_USE_STATIC 1)
#FIND_PACKAGE(IbexLib REQUIRED)
#include_directories(${IBEX_INCLUDE_DIRS})

#catkin_package(
##  INCLUDE_DIRS include
##  LIBRARIES seabot_log
##  CATKIN_DEPENDS
##  DEPENDS system_lib
#)

################ BUILD Targets ###############

#include_directories(
## include
#    ${catkin_INCLUDE_DIRS}
#    ${ZLIB_INCLUDE_DIRS}
#    src/cnpy
#    ${IPEGENERATOR_INCLUDE_DIRS}
#)

##include(${VTK_USE_FILE})

#add_executable(log_analyzer_node
#    src/log_analyzer.cpp
#    src/cnpy/cnpy.cpp
#)
#add_dependencies(log_analyzer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(log_analyzer_node
#  ${catkin_LIBRARIES}
#  ${PYTHON_LIBRARIES}
#  ${ZLIB_LIBRARIES}
#)

#add_executable(realtime_depth_data
#				src/realtime_depth_data.cpp
#)
#add_dependencies(realtime_depth_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(realtime_depth_data
#  ${catkin_LIBRARIES}
#  ${IPEGENERATOR_LIBRARIES}
#  ${IBEX_LIBRARIES}
#)

##add_executable(log_state_space
##				src/log_state_space.cpp
##)
##add_dependencies(log_state_space ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

##target_link_libraries(log_state_space
##  ${catkin_LIBRARIES}
##  ${IPEGENERATOR_LIBRARIES}
##  ${IBEX_LIBRARIES}
##)

#endif()
