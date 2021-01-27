cmake_minimum_required(VERSION 2.8.3)
project(tvf_nav_descartes)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  move_interface
  message_generation
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  descartes_utilities
  std_msgs
  actionlib
  actionlib_msgs
  trajectory_msgs
  geometry_msgs
  tf
  tf_conversions
  move_interface
)

find_package(Boost REQUIRED COMPONENTS system thread)

## Generate messages in the 'msg' folder
add_action_files(DIRECTORY action FILES TVFNavDescartes.action)
## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs actionlib_msgs)

catkin_package( CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  actionlib
  DEPENDS Boost   
          descartes_core
          descartes_moveit
          descartes_trajectory
          descartes_planner
          descartes_utilities
          move_interface
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_executable(tvf_nav_descartes_server src/tvf_nav_descartes.cpp)
target_link_libraries(tvf_nav_descartes_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(tvf_nav_descartes
    src/tvf_nav_descartes.cpp
)
add_dependencies(tvf_nav_descartes_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tvf_nav_descartes tvf_nav_descartes_generate_messages_cpp)

#############
## Install ##
#############

install(TARGETS tvf_nav_descartes_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
