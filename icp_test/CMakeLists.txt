cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp_test)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp_test icp_test.cpp)
target_link_libraries (icp_test ${PCL_LIBRARIES})
