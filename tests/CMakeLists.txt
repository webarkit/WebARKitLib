cmake_minimum_required(VERSION 3.24)
project(Webarkit_tests)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

# Fetch googletest v1.13.0  commit b796f7d44681514f58a683a3a71ff17c94edb0c1
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/b796f7d44681514f58a683a3a71ff17c94edb0c1.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_subdirectory(../WebARKit ../WebARKit/build)

add_executable(
  hello_test
  hello_test.cc
)

message(STATUS "CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

get_filename_component(PARENT_DIR ../ ABSOLUTE)
get_filename_component(OPENCV_DIR ../../../opencv/ ABSOLUTE)
get_filename_component(OPENCV_JS_DIR ../../../opencv_js/ ABSOLUTE)
get_filename_component(OPENCV_CONTRIB_DIR ../../../opencv_contrib/ ABSOLUTE)

target_include_directories(hello_test PRIVATE "${PARENT_DIR}/WebARKit/include")
target_include_directories(hello_test PRIVATE "${PARENT_DIR}/WebARKit/WebARKitTrackers/WebARKitOpticalTracking/include")
target_include_directories(hello_test PRIVATE
        ${OPENCV_CONFIG_FILE_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
        ${OPENCV_MODULE_opencv_calib3d_LOCATION}/include
        ${OPENCV_MODULE_opencv_core_LOCATION}/include
        ${OPENCV_MODULE_opencv_features2d_LOCATION}/include
        ${OPENCV_MODULE_opencv_flann_LOCATION}/include
        ${OPENCV_MODULE_opencv_imgproc_LOCATION}/include
        ${OPENCV_MODULE_opencv_imgcodecs_LOCATION}/include
        ${OPENCV_MODULE_opencv_video_LOCATION}/include
        ${OPENCV_MODULE_opencv_xfeatures2d_LOCATION}/include
        )

set(hello_test 
${PARENT_DIR}/WebARKit/WebARKitManager.cpp
)

target_link_libraries(
  hello_test
  WebARKitLib
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hello_test)