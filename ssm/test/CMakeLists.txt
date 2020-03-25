if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    
    message("Include Directories: ${INCLUDE_DIRECTORIES}")
    
    message("******************** include directories: ${INCLUDE_DIRECTORIES}")
    message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
    catkin_add_gtest(ssmtest ssm_test.cpp)
    # target_include_directories(ssmtest PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})
    include_directories (BEFORE ${PROJECT_SOURCE_DIR}/test/include)
    message("******************** include directories: ${INCLUDE_DIRECTORIES}")
    message("******************** CATKIN_ENABLE_TESTING: ${CATKIN_ENABLE_TESTING}")
    target_link_libraries(ssmtest ${catkin_LIBRARIES})

    # add_rostest_gtest(tests_myfirsttest my_first_test.test MyFirstTest.cpp)
    # target_link_libraries(tests_myfirsttest ${catkin_LIBRARIES})

endif()

