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
target_include_directories(hello_test PRIVATE "${OPENCV_CONTRIB_DIR}/modules/xfeatures2d/include")
target_include_directories(hello_test PRIVATE "${OPENCV_JS_DIR}")
target_include_directories(hello_test PRIVATE "${OPENCV_DIR}/include")
target_include_directories(hello_test PRIVATE "${OPENCV_DIR}/modules/calib3d/include")
target_include_directories(hello_test PRIVATE "${OPENCV_DIR}/modules/core/include")
target_include_directories(hello_test PRIVATE "${OPENCV_DIR}/modules/features2d/include")
target_include_directories(hello_test PRIVATE "${OPENCV_DIR}/modules/flann/include")
target_include_directories(hello_test PRIVATE "${OPENCV_DIR}/modules/imgproc/include")
target_include_directories(hello_test PRIVATE "${OPENCV_DIR}/modules/imgcodecs/include")
target_include_directories(hello_test PRIVATE "${OPENCV_DIR}/modules/video/include")

target_link_libraries(
  hello_test
  GTest::gtest_main
)

target_link_libraries(
  hello_test
  WebARKitManager.cpp
  Private "${PARENT_DIR}/WebARKit/WebARKitManager.cpp"
)

include(GoogleTest)
gtest_discover_tests(hello_test)