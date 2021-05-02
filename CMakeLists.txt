cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(halepensis)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (alpha1 
    halepensis/alpha1.cpp 
    halepensis/utils.cpp 
    halepensis/object.cpp
    halepensis/scene.cpp 
    halepensis/viewer.cpp
    halepensis/feature_cloud.cpp
    halepensis/template_alignment.cpp)
target_link_libraries (alpha1 ${PCL_LIBRARIES})