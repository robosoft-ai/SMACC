cmake_minimum_required(VERSION 2.8.7)
project(smacc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  pluginlib
  smacc_msgs
  controller_manager_msgs
)


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
  LIBRARIES smacc
  CATKIN_DEPENDS actionlib roscpp smacc_msgs controller_manager_msgs smacc_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11) #workaround for ubuntu 16.04, to extinguish

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ${PROJECT_NAME}_IncludeFilesPaths "*.h")
add_custom_target(${PROJECT_NAME}_IncludeFilesTarget SOURCES ${${PROJECT_NAME}_IncludeFilesPaths})

# sources
file(GLOB_RECURSE SRC_FILES "src/smacc/*.cpp" "src/smacc/introspection/*.cpp" "src/smacc/state_reactors/*.cpp") # opcionalmente GLOB

add_library(${PROJECT_NAME} ${SRC_FILES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
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

#############
## Testing ##
#############
