cmake_minimum_required(VERSION 2.8.7)
project(sm_radial_motion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp smacc smacc_navigation_plugin smacc_odom_tracker smacc_interface_components smacc_planner_switcher tf nav_msgs smacc_action_client_generic)

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
  INCLUDE_DIRS include
  #LIBRARIES smacc
  #CATKIN_DEPENDS actionlib roscpp
  #DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11) #workaround for ubuntu 16.04, to extinguish

## Specify additional locations of header files
## Your package locations should be listed before other locations

#------------------------------------------------------------
include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
                src/radial_motion.cpp)


target_link_libraries(${PROJECT_NAME}_node
                      ${catkin_LIBRARIES}
                      ${boost_LIBRARIES})

add_executable(tool_action_server_node test/tool_action_server/src/tool_action_server_node.cpp)

 target_link_libraries(
   tool_action_server_node
   ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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

install(TARGETS
    ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}}
)

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
    launch/radial_motion.launch
    launch/ridgeback_simulation.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
    config/backward_local_planner.yaml
    config/forward_local_planner.yaml
    config/navigation.rviz
    config/radial_motion_sm_config.yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
    urdf/empty.xacro
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_radial_motion_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)