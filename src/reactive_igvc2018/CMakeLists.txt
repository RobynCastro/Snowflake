cmake_minimum_required(VERSION 2.8.3)
project(reactive_igvc2018)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy)

## System dependencies are found with CMake's conventions
find_package(sb_utils REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ipm
   CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   ${catkin_INCLUDE_DIRS}
   ${sb_utils_INCLUDE_DIRS}
   ./include
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(hole_tracker_node
        src/hole_tracker_node.cpp
        src/HoleTrackerNode.cpp
        src/HoleTrackerDecision.cpp
        src/ObstacleManager.cpp
        src/LinearAlgebra.cpp
        src/LineAvoider.cpp
        src/ConeAvoider.cpp
        src/ConeLineAvoider.cpp
        include/HoleTrackerNode.h
        include/HoleTrackerDecision.h
        include/ObstacleManager.h
        include/LinearAlgebra.h
        include/LineAvoider.h
        include/ConeAvoider.h
        include/ConeLineAvoider.h)

## Specify libraries to link a library or executable target against
target_link_libraries(hole_tracker_node
        ${catkin_LIBRARIES}
        ${sb_utils_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_decision_igvc2018.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
