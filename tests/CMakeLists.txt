cmake_minimum_required(VERSION 3.22)
project(Webarkit_tests)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

if(POLICY CMP0135)
  cmake_policy(SET CMP0026 NEW)
endif()

# Fetch googletest v1.13.0  commit b796f7d44681514f58a683a3a71ff17c94edb0c1
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/b796f7d44681514f58a683a3a71ff17c94edb0c1.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_Declare(
  build_opencv
 URL https://github.com/webarkit/opencv-em/releases/download/0.1.4/opencv-4.7.0.zip
)

FetchContent_MakeAvailable(googletest build_opencv)

enable_testing()

add_subdirectory(../WebARKit ../WebARKit/build)

add_executable(
  webarkit_test
  webarkit_test.cc
)

get_filename_component(PARENT_DIR ../ ABSOLUTE)

target_include_directories(webarkit_test PRIVATE "${PARENT_DIR}/WebARKit/include")
target_include_directories(webarkit_test PRIVATE "${PARENT_DIR}/WebARKit/WebARKitTrackers/WebARKitOpticalTracking/include")

target_include_directories(webarkit_test PRIVATE
        "${build_opencv_SOURCE_DIR}"
        "${build_opencv_SOURCE_DIR}/3rdparty/include/libjpeg"
        "${build_opencv_SOURCE_DIR}/libs/opencv/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/calib3d/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/core/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/features2d/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/flann/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/imgcodecs/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/imgproc/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/video/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv_contrib/modules/xfeatures2d/include"
        )

set(webarkit_test 
${PARENT_DIR}/WebARKit/WebARKitManager.cpp
)

# The order of the libs make the difference!

target_link_libraries(
  webarkit_test
  WebARKitLib
  "${build_opencv_SOURCE_DIR}/lib/libopencv_calib3d.a"
  "${build_opencv_SOURCE_DIR}/lib/libopencv_features2d.a"
  "${build_opencv_SOURCE_DIR}/lib/libopencv_flann.a"
  "${build_opencv_SOURCE_DIR}/lib/libopencv_imgcodecs.a"
  "${build_opencv_SOURCE_DIR}/lib/libopencv_video.a"
  "${build_opencv_SOURCE_DIR}/lib/libopencv_xfeatures2d.a"
  "${build_opencv_SOURCE_DIR}/lib/libopencv_imgproc.a"
  "${build_opencv_SOURCE_DIR}/lib/libopencv_core.a"
  "${build_opencv_SOURCE_DIR}/3rdparty/lib/liblibopenjp2.a"
  "${build_opencv_SOURCE_DIR}/3rdparty/lib/libzlib.a"
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(webarkit_test)