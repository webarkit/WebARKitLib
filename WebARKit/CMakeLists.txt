cmake_minimum_required(VERSION 3.20)

project(WebARKitLib)

set(CMAKE_CXX_STANDARD 14)

include(FetchContent)

set(BUILD_SHARED_LIBS OFF)
set(BUILD_opencv_dnn OFF)
set(BUILD_opencv_highgui OFF)
set(BUILD_opencv_ml OFF)
set(BUILD_opencv_objdetect OFF)
set(BUILD_opencv_gapi OFF)
set(BUILD_opencv_photo OFF)
set(BUILD_opencv_stitching OFF)
set(BUILD_opencv_ts OFF)
set(BUILD_opencv_videoio OFF)
set(BUILD_opencv_python2 OFF)
set(BUILD_opencv_python3 OFF)

FetchContent_Declare(
  opencv
  GIT_REPOSITORY https://github.com/webarkit/opencv.git
  GIT_TAG 889dfa15f1cd60e571d33b23b87a1d506d381ea9
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)

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

target_link_libraries(WebARKitLib STATIC opencv_calib3d opencv_core opencv_features2d opencv_flann opencv_imgproc opencv_imgcodecs opencv_video)