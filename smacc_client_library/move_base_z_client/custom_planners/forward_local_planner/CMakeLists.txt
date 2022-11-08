cmake_minimum_required(VERSION 2.8.7)
project(forward_local_planner)

find_package(catkin REQUIRED
  roscpp
  costmap_2d
  tf
  nav_core
  base_local_planner
  nav_msgs
  move_base_msgs
  geometry_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES forward_local_planner
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11) #workaround for ubuntu 16.04, to extinguish

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/forward_local_planner.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

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

install(FILES
   flp_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_forward_local_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
