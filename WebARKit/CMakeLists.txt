cmake_minimum_required(VERSION 3.16)

project(WebARKitLib)

set(CMAKE_CXX_STANDARD 14)

if(VERSION GREATER 3.24)
  cmake_policy(SET CMP0135 NEW)
endif()

include(FetchContent)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/OpenCVEm.cmake)

option(WEBARKIT_SIMD "Use the SIMD-enabled emscripten OpenCV build" OFF)

if(${EMSCRIPTEN_COMP} EQUAL 1)
  if(WEBARKIT_SIMD)
    set(OPENCV_FETCH_DESC "SIMD emscripten")
    set(OPENCV_FETCH_URL ${OPENCV_EMSCRIPTEN_SIMD_URL})
    set(OPENCV_FETCH_HASH ${OPENCV_EMSCRIPTEN_SIMD_HASH})
  else()
    set(OPENCV_FETCH_DESC "emscripten")
    set(OPENCV_FETCH_URL ${OPENCV_EMSCRIPTEN_URL})
    set(OPENCV_FETCH_HASH ${OPENCV_EMSCRIPTEN_HASH})
  endif()
else()
  set(OPENCV_FETCH_DESC "native")
  set(OPENCV_FETCH_URL ${OPENCV_NATIVE_URL})
  set(OPENCV_FETCH_HASH ${OPENCV_NATIVE_HASH})
endif()

message("Fetching ${OPENCV_FETCH_DESC} opencv from webarkit/opencv-em ${OPENCV_EM_RELEASE} ...")
FetchContent_Declare(
  build_opencv
  URL ${OPENCV_FETCH_URL}
  URL_HASH ${OPENCV_FETCH_HASH}
)

FetchContent_MakeAvailable(build_opencv)

get_filename_component(PARENT_DIR ./ ABSOLUTE)

set(WEBARKIT_HEADERS
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/TrackedPoint.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/TrackingPointSelector.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitHomographyInfo.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h
${PARENT_DIR}/include/WebARKitCamera.h
${PARENT_DIR}/include/WebARKitLog.h
${PARENT_DIR}/include/WebARKitGL.h
${PARENT_DIR}/include/WebARKitManager.h
${PARENT_DIR}/include/WebARKitPattern.h
)

set(SOURCE 
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/TrackedPoint.cpp
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/TrackingPointSelector.cpp
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.cpp
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/WebARKitHomographyInfo.cpp
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.cpp
${PARENT_DIR}/WebARKitCamera.cpp
${PARENT_DIR}/WebARKitLog.cpp
${PARENT_DIR}/WebARKitGL.cpp
${PARENT_DIR}/WebARKitManager.cpp
${PARENT_DIR}/WebARKitPattern.cpp
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

target_link_libraries(WebARKitLib
jpeg
"${build_opencv_SOURCE_DIR}/3rdparty/lib/liblibopenjp2.a"
)