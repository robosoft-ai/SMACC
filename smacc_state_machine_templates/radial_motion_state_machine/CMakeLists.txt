cmake_minimum_required(VERSION 2.8.7)
project(radial_motion_state_machine)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp smacc_core smacc_navigation_plugin smacc_odom_tracker smacc_interface_components smacc_planner_switcher)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES smacc
  #CATKIN_DEPENDS actionlib roscpp
  #DEPENDS system_lib
)