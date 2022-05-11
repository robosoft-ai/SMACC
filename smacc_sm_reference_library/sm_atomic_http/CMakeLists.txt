cmake_minimum_required(VERSION 2.8.3)
project(sm_atomic_http)

## Find catkin macros and libraries
find_package(catkin REQUIRED smacc ros_timer_client)

include(FetchContent)
FetchContent_Declare(
  cpr 
  GIT_REPOSITORY https://github.com/libcpr/cpr.git
  GIT_TAG 6ea2dec23c3df14ac3b27b7d2d6bbff0cb7ba1b0
) # The commit hash for 1.8.1. Replace with the latest from: https://github.com/libcpr/cpr/releases
FetchContent_MakeAvailable(cpr)

###################################
## catkin specific configuration ##
###################################
catkin_package()

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

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/sm_atomic_http_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  cpr::cpr
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and config files, etc.)
install(FILES
  launch/sm_atomic.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
