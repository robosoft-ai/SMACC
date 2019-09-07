cmake_minimum_required(VERSION 2.8.3)

project(smacc_viewer)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS smacc_msgs
  )

# Install targets
#install(PROGRAMS scripts/smacc_viewer.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
