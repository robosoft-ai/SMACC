cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Get access to nasa_cmake
rosbuild_find_ros_package(nasa_cmake)
list(APPEND CMAKE_MODULE_PATH ${nasa_cmake_PACKAGE_PATH})
find_package(Log4cpp REQUIRED)
include_directories(${Log4cpp_INCLUDE_DIR})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/JointControlCommonInterface.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlActualFsmGripper.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlActualFsmSeriesElastic.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlActualFsmWrist.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlActualFsmFinger.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlCommandFsmGripper.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlCommandFsmSeriesElastic.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlCommandFsmWrist.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlCommandFsmFinger.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlManagerGripper.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlManagerSeriesElastic.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlManagerWrist.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlManagerFinger.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointControlManagerFactory.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointCommandFactory.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointCommandSeriesElastic.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointCommandRigid.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointCommandGripper.cpp
                                     ${PROJECT_SOURCE_DIR}/src/FingerKinematics.cpp
                                     ${PROJECT_SOURCE_DIR}/src/FingerController.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointCommandWrist.cpp
                                     ${PROJECT_SOURCE_DIR}/src/JointCommandFinger.cpp
                                     ${PROJECT_SOURCE_DIR}/src/WristMechanism.cpp
                                     ${PROJECT_SOURCE_DIR}/src/TubeTareMechanism.cpp
                                     ${PROJECT_SOURCE_DIR}/src/GripperKinematics.cpp
                                     ${PROJECT_SOURCE_DIR}/src/GripperPositionStateMachine.cpp
                                     ${PROJECT_SOURCE_DIR}/src/GripperSupervisor.cpp
                                     ${PROJECT_SOURCE_DIR}/src/EmbeddedSmoother.cpp
									 ${PROJECT_SOURCE_DIR}/src/GripperEnvironment.cpp
                                     )

rosbuild_link_boost(${PROJECT_NAME} regex)
target_link_libraries(${PROJECT_NAME} tinyxml)

# Code Coverage Determination and Add compiler flags
option(CAPTURE_CODE_COVERAGE "Do you want to capture code coverage data?" OFF)
if (CAPTURE_CODE_COVERAGE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lgcov")
    message(STATUS "Capture Code Coverage Data: ON")
else (CAPTURE_CODE_COVERAGE)
    message(STATUS "Capture Code Coverage Data: OFF")
endif (CAPTURE_CODE_COVERAGE)

# Set variables to get things to work
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") # TODO: Remove someday: To get gtests to compile on 12.04 Fuerte
rosbuild_add_compile_flags(${PROJECT_NAME} "-fPIC")

# Specify test source files
rosbuild_add_gtest(JointControlActualFsm_Test       ${PROJECT_SOURCE_DIR}/test/JointControlActualFsm_Test.cpp)
rosbuild_add_gtest(JointControlCommandFsm_Test      ${PROJECT_SOURCE_DIR}/test/JointControlCommandFsm_Test.cpp)
rosbuild_add_gtest(JointControlManager_Test         ${PROJECT_SOURCE_DIR}/test/JointControlManager_Test.cpp)
rosbuild_add_gtest(JointCommandSeriesElastic_Test   ${PROJECT_SOURCE_DIR}/test/JointCommandSeriesElastic_Test.cpp)
rosbuild_add_gtest(JointCommandRigid_Test           ${PROJECT_SOURCE_DIR}/test/JointCommandRigid_Test.cpp)
rosbuild_add_gtest(JointCommandGripper_Test         ${PROJECT_SOURCE_DIR}/test/JointCommandGripper_Test.cpp)
rosbuild_add_gtest(JointCommandFactory_Test         ${PROJECT_SOURCE_DIR}/test/JointCommandFactory_Test.cpp)
rosbuild_add_gtest(RightWristMechanism_Test         ${PROJECT_SOURCE_DIR}/test/RightWristMechanism_Test.cpp)
rosbuild_add_gtest(LeftWristMechanism_Test          ${PROJECT_SOURCE_DIR}/test/LeftWristMechanism_Test.cpp)
rosbuild_add_gtest(JointCommandWrist_Test           ${PROJECT_SOURCE_DIR}/test/JointCommandWrist_Test.cpp)
rosbuild_add_gtest(FingerController_Test            ${PROJECT_SOURCE_DIR}/test/FingerController_Test.cpp)
rosbuild_add_gtest(TubeTareMechanism_Test           ${PROJECT_SOURCE_DIR}/test/TubeTareMechanism_Test.cpp)
rosbuild_add_gtest(GripperKinematics_Test           ${PROJECT_SOURCE_DIR}/test/GripperKinematics_Test.cpp)
rosbuild_add_gtest(GripperPositionStateMachine_Test ${PROJECT_SOURCE_DIR}/test/GripperPositionStateMachine_Test.cpp)
rosbuild_add_gtest(GripperSupervisorTest            ${PROJECT_SOURCE_DIR}/test/GripperSupervisorTest.cpp)
rosbuild_add_gtest(GripperEnvironmentTest           ${PROJECT_SOURCE_DIR}/test/GripperEnvironmentTest.cpp)
rosbuild_add_gtest(JointCommandFinger_Test          ${PROJECT_SOURCE_DIR}/test/JointCommandFinger_Test.cpp)
rosbuild_add_gtest(EmbeddedSmoother_Test            ${PROJECT_SOURCE_DIR}/test/EmbeddedSmoother_Test.cpp)

# Link tests into executables
target_link_libraries(JointControlActualFsm_Test       ${PROJECT_NAME})
target_link_libraries(JointControlCommandFsm_Test      ${PROJECT_NAME})
target_link_libraries(JointControlManager_Test         ${PROJECT_NAME})
target_link_libraries(JointCommandSeriesElastic_Test   ${PROJECT_NAME})
target_link_libraries(JointCommandRigid_Test           ${PROJECT_NAME})
target_link_libraries(JointCommandGripper_Test         ${PROJECT_NAME})
target_link_libraries(JointCommandFactory_Test         ${PROJECT_NAME})
target_link_libraries(RightWristMechanism_Test         ${PROJECT_NAME})
target_link_libraries(LeftWristMechanism_Test          ${PROJECT_NAME})
target_link_libraries(JointCommandWrist_Test           ${PROJECT_NAME})
target_link_libraries(FingerController_Test            ${PROJECT_NAME})
target_link_libraries(TubeTareMechanism_Test           ${PROJECT_NAME})
target_link_libraries(GripperKinematics_Test           ${PROJECT_NAME})
target_link_libraries(GripperPositionStateMachine_Test ${PROJECT_NAME})
target_link_libraries(GripperSupervisorTest            ${PROJECT_NAME})
target_link_libraries(GripperEnvironmentTest            ${PROJECT_NAME})
target_link_libraries(JointCommandFinger_Test          ${PROJECT_NAME})
target_link_libraries(EmbeddedSmoother_Test            ${PROJECT_NAME})
