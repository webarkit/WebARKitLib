cmake_minimum_required(VERSION 3.20)

project(WebARKitLib)

set(CMAKE_CXX_STANDARD 14)

message(STATUS "CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

include(FetchContent)

FetchContent_Declare(
  opencv
  GIT_REPOSITORY https://github.com/webarkit/opencv.git
  GIT_TAG 889dfa15f1cd60e571d33b23b87a1d506d381ea9
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" INTERNAL)
message("BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")

FetchContent_MakeAvailable(opencv)

get_filename_component(PARENT_DIR ./ ABSOLUTE)
get_filename_component(OPENCV_DIR ../../../opencv/ ABSOLUTE)
get_filename_component(OPENCV_JS_DIR ../../../opencv_js/ ABSOLUTE)
get_filename_component(OPENCV_CONTRIB_DIR ../../../opencv_contrib/ ABSOLUTE)


set(WEBARKIT_HEADERS
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking//WebARKitEnums.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/include/WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h
${PARENT_DIR}/include/WebARKitLog.h
${PARENT_DIR}/include/WebARKitManager.h
)

set(SOURCE 
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.cpp
${PARENT_DIR}/WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.cpp
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
target_include_directories(WebARKitLib PRIVATE "${OPENCV_CONTRIB_DIR}/modules/xfeatures2d/include")
target_include_directories(WebARKitLib PRIVATE "${OPENCV_JS_DIR}")

message(STATUS "OPENCV_CONFIG_FILE_INCLUDE_DIR: ${OPENCV_CONFIG_FILE_INCLUDE_DIR}")
message(STATUS "OPENCV_INCLUDE_DIR: ${OPENCV_INCLUDE_DIR}")

target_include_directories(WebARKitLib PRIVATE
        ${OPENCV_CONFIG_FILE_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
        ${OPENCV_MODULE_opencv_calib3d_LOCATION}/include
        ${OPENCV_MODULE_opencv_core_LOCATION}/include
        ${OPENCV_MODULE_opencv_features2d_LOCATION}/include
        ${OPENCV_MODULE_opencv_flann_LOCATION}/include
        ${OPENCV_MODULE_opencv_imgproc_LOCATION}/include
        ${OPENCV_MODULE_opencv_imgcodecs_LOCATION}/include
        ${OPENCV_MODULE_opencv_video_LOCATION}/include
        )

#target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/include")
#target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/calib3d/include")
#target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/core/include")
#target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/features2d/include")
#target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/flann/include")
#target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/imgproc/include")
#target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/imgcodecs/include")
#target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/video/include")

target_link_libraries(WebARKitLib STATIC opencv_calib3d opencv_core opencv_features2d opencv_flann opencv_imgproc opencv_imgcodecs opencv_video)

#target_link_libraries(WebARKitLib "${OPENCV_JS_DIR}/lib/*.a")