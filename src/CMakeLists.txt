cmake_minimum_required (VERSION 2.6)
project (SDC)

ADD_DEFINITIONS(-std=c++11)

include_directories("${PROJECT_BINARY_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/ekf")
include_directories ("${PROJECT_SOURCE_DIR}/rrt")
include_directories ("${PROJECT_SOURCE_DIR}/car")
include_directories ("${PROJECT_SOURCE_DIR}/utilities")
add_subdirectory (ekf)
add_subdirectory (rrt) 
add_subdirectory (car) 
add_subdirectory (utilities) 

add_executable(ekf_observer ekf_observer.cpp)
target_link_libraries (ekf_observer ekf)
target_link_libraries (ekf_observer rrt)
target_link_libraries (ekf_observer car)
target_link_libraries (ekf_observer utilities)

set(CMAKE_BUILD_TYPE Debug)

install (TARGETS ekf_observer DESTINATION bin)
install (FILES "${PROJECT_BINARY_DIR}/ekf_observer"        
         DESTINATION include)
