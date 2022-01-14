cmake_minimum_required(VERSION 2.8.12)
project(student_proect)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
 lib
)

add_subdirectory(lib/clipper)

## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
  # interface
   src/student_interface.cpp
  # roadmap
   src/rm/inflate.cpp
   src/rm/maxClearance.cpp
   src/rm/RoadMap.cpp
   src/rm/geometry.cpp
   src/rm/visibility.cpp
  # dubins
   src/dubins/dubins.cpp
  # navigation
   src/nav/dijkstra.cpp
   src/nav/NavMap.cpp
   src/nav/pursuerEvader.cpp
  #utils
   src/utils/timer.cpp
   src/utils/MatlabPlot.cpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}  
  polyclipping
)

## Compile tests
add_subdirectory(test)
