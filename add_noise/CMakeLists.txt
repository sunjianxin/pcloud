cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(add_noise)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (add_noise add_noise.cpp)
target_link_libraries (add_noise ${PCL_LIBRARIES})
