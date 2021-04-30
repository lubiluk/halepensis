cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(halepensis)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (alpha1 
    src/alpha1.cpp 
    src/utils.cpp 
    src/object.cpp
    src/scene.cpp 
    src/viewer.cpp
    src/feature_cloud.cpp
    src/template_alignment.cpp)
target_link_libraries (alpha1 ${PCL_LIBRARIES})