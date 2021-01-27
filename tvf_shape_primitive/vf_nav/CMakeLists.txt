cmake_minimum_required(VERSION 2.8.3)
project(vf_nav)

## Catkin dependencies
find_package(catkin REQUIRED COMPONENTS 
  rviz
  roscpp 
  rqt_gui_cpp
  message_generation
  actionlib
  actionlib_msgs
  geometry_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)

# Qt Stuff
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

set(QT_INCLUDE_DIRS 
  ${Qt5Core_INCLUDE_DIRS} 
  ${Qt5Widgets_INCLUDE_DIRS}
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_action_files(DIRECTORY action FILES VFNav.action)
## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

qt5_wrap_cpp(MOC_FILES
  include/vf_nav_rviz/vf_nav_rviz.hpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/vf_nav_rviz.cpp
  ${MOC_FILES}
)

add_dependencies(${PROJECT_NAME} vf_nav_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/VFNavClient.py
  scripts/VFNavigation.py
  scripts/VFNavServer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

