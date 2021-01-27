cmake_minimum_required(VERSION 2.8.3)
project(tvf_shape_primitive)

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
  actionlib_msgs
  actionlib
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system graph)

###################################
## catkin specific configuration ##
###################################

catkin_python_setup()
## Generate messages in the 'msg' folder
add_action_files(DIRECTORY action FILES SurfaceVF.action)
## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs actionlib_msgs)

catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS pcl_ros sensor_msgs std_msgs actionlib_msgs 
    # LIBRARIES vf_construction
    # DEPENDS 
)
#include_directories(
#    include
#    ${catkin_INCLUDE_DIRS}
#)

####################################################################################################



# target_link_libraries(vf_construction ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

####################################################################################################



## add_dependencies(parser ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(PROGRAMS
  scripts/SurfaceVFClientFile.py
  scripts/SurfaceVFGeneratorFile.py
  scripts/SurfaceVFServerFile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
