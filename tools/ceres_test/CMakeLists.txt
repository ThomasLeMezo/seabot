# ==================================================================
#  Test - cmake configuration file
# ==================================================================

cmake_minimum_required(VERSION 3.2.2)

# Settings
set(CMAKE_CXX_STANDARD 11)
set(PACKAGE_NAME      ceres-test)
set(PACKAGE_VERSION "0.1")
project(${PACKAGE_NAME})

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(LAPACK QUIET)
# find_package(SuiteSparse)
# find_package(CXSparse)
# find_package(Gflags)
# find_package(Glog)
# find_package(OpenMP QUIET)
# find_package(TBB QUIET)

include_directories(${EIGEN_INCLUDE_DIRS})

  set(SRC
    src/main.cpp
  )

  set(HEADERS

  )

  add_executable(helloworld ${SRC})

  target_link_libraries(helloworld
      ceres
  )

# Tests

 #  if(BUILD_TESTS)
	# add_test(NAME ExampleArithmetics COMMAND ex_arithmetic 0)
 #  endif(BUILD_TESTS)
