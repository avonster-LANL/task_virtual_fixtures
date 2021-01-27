cmake_minimum_required(VERSION 2.8.3)
project(tvf_construction)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wno-enum-compare" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  actionlib_msgs
  actionlib
  message_generation
  tf
  tf_conversions
  eigen_conversions
  pcl_ros
  pcl_conversions
  moveit_ros_move_group
  moveit_ros_planning_interface
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
  CATKIN_DEPENDS pcl_ros sensor_msgs std_msgs actionlib_msgs 
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost 
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

####################################################################################################

add_library(${PROJECT_NAME}
  src/stl_parser.cpp
  src/stl_pcn.cpp
  src/pcn_layers.cpp
  src/tvf_graph.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

####################################################################################################

add_executable(cloud_normals_viewer_node src/cloud_normals_viewer_node.cpp)

add_executable(tvf_construction_node src/tvf_construction_node.cpp)
add_executable(batch_tvf_construction_node src/batch_tvf_construction_node.cpp)
add_executable(parametric_tvf_construction_node src/parametric_tvf_construction_node.cpp)

## add_dependencies(parser ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(cloud_normals_viewer_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})

target_link_libraries(tvf_construction_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(batch_tvf_construction_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(parametric_tvf_construction_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

