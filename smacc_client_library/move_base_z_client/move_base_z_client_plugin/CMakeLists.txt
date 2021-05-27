cmake_minimum_required(VERSION 2.8.7)
project(move_base_z_client_plugin)

find_package(catkin REQUIRED smacc pluginlib tf dynamic_reconfigure)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Needed so catkin can package the thread library dependency
set(THREADS_LIBRARIES "${CMAKE_THREAD_LIBS_INIT}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


add_action_files(
   FILES
   OdomTracker.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/OdomTracker.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES move_base_z_client_plugin odom_tracker waypoints_navigator planner_switcher move_base_z_client_behaviors costmap_switch pose
   CATKIN_DEPENDS smacc tf
   DEPENDS THREADS YAML_CPP
)

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 17)
#add_compile_options(-std=c++11) #workaround for ubuntu 16.04, to extinguish

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/move_base_z_client_plugin.cpp
 )

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#----------------------------------

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
#----------------------------------
## Declare a C++ library
add_library(odom_tracker
   src/components/odom_tracker/odom_tracker.cpp
)

target_link_libraries(odom_tracker
   ${catkin_LIBRARIES})
#----------------------------------
add_library(planner_switcher
   src/components/planner_switcher/planner_switcher.cpp
)

target_link_libraries(planner_switcher
   ${catkin_LIBRARIES})

#----------------------------------
add_library(pose
   src/components/pose/cp_pose.cpp
)

target_link_libraries(pose
   ${catkin_LIBRARIES}
   Threads::Threads
)

#----------------------------------
add_library(costmap_switch
   src/components/costmap_switch/cp_costmap_switch.cpp
)

target_link_libraries(costmap_switch
   ${catkin_LIBRARIES})

#----------------------------------
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
 add_library(waypoints_navigator
   src/components/waypoints_navigator/waypoints_event_dispatcher.cpp
   src/components/waypoints_navigator/waypoints_navigator.cpp
 )

target_link_libraries(waypoints_navigator
   ${catkin_LIBRARIES}
   ${YAML_CPP_LIBRARIES}
   planner_switcher
   move_base_z_client_plugin
)
#----------------------------------
file(GLOB_RECURSE SRC_FILES "src/client_behaviors/*.cpp") # opcionalmente GLOB

add_library(move_base_z_client_behaviors
   ${SRC_FILES}
)

target_link_libraries(move_base_z_client_behaviors
   ${catkin_LIBRARIES}
   odom_tracker
   planner_switcher
   waypoints_navigator
   Threads::Threads
)


 add_dependencies(odom_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME} odom_tracker waypoints_navigator planner_switcher costmap_switch move_base_z_client_behaviors pose
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_move_base_z_client_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
