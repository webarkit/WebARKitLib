cmake_minimum_required(VERSION 3.16)

project(WebARKitLib)

set(CMAKE_CXX_STANDARD 14)

if(VERSION GREATER 3.24)
  cmake_policy(SET CMP0135 NEW)
endif()

include(FetchContent)

if(${EMSCRIPTEN_COMP} EQUAL 1)
  message("Fetching opencv for emscripten compilation from webarkit/opencv-em ...")
  FetchContent_Declare(
    build_opencv
    URL https://github.com/webarkit/opencv-em/releases/download/0.1.3/opencv-js-4.7.0-emcc-3.1.26.zip
  )
else()
  message("Fetching opencv from webarkit/opencv-em ...")
  FetchContent_Declare(
    build_opencv
    URL https://github.com/webarkit/opencv-em/releases/download/0.1.3/opencv-4.7.0.zip
  )
endif()

FetchContent_MakeAvailable(build_opencv)

get_filename_component(PARENT_DIR ./ ABSOLUTE)

set(WEBARKIT_HEADERS
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking//WebARKitEnums.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h
${PARENT_DIR}/include/WebARKitCamera.h
${PARENT_DIR}/include/WebARKitLog.h
${PARENT_DIR}/include/WebARKitManager.h
)

set(SOURCE 
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.cpp
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.cpp
${PARENT_DIR}/WebARKitCamera.cpp
${PARENT_DIR}/WebARKitLog.cpp
${PARENT_DIR}/WebARKitManager.cpp
)

add_library(
  WebARKitLib STATIC
  ${WEBARKIT_HEADERS}
  ${SOURCE}
)

target_include_directories(WebARKitLib PRIVATE "${PARENT_DIR}/include")
target_include_directories(WebARKitLib PRIVATE "${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include")

target_include_directories(WebARKitLib PRIVATE
        "${build_opencv_SOURCE_DIR}"
        "${build_opencv_SOURCE_DIR}/libs/opencv/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/calib3d/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/core/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/features2d/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/flann/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/imgproc/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv/modules/video/include"
        "${build_opencv_SOURCE_DIR}/libs/opencv_contrib/modules/xfeatures2d/include"
        )