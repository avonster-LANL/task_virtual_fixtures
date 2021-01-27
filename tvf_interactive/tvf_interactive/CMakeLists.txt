cmake_minimum_required(VERSION 2.8.3)
project(tvf_interactive)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wno-enum-compare" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf
  tf_conversions
  eigen_conversions
  moveit_ros_move_group
  moveit_ros_planning_interface
  industrial_robot_simulator
  rviz_visual_tools
  interactive_markers
  tvf_construction
  tvf_nav_descartes
)
find_package(Boost REQUIRED COMPONENTS system graph)

## Uses openmp
find_package(OpenMP REQUIRED)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

###################################
## catkin specific configuration ##
###################################

# catkin_python_setup()

## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS sensor_msgs std_msgs
    tvf_nav_descartes
    LIBRARIES ${PROJECT_NAME}
    # DEPENDS 
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

####################################################################################################

add_library(${PROJECT_NAME}
    src/tvf_graph_imarker.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

####################################################################################################

add_executable(tvf_graph_imarker_node src/tvf_graph_imarker_node.cpp)
add_executable(interpolation_voxel src/interpolation_voxel.cpp)

target_link_libraries(tvf_graph_imarker_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(interpolation_voxel ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

