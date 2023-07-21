cmake_minimum_required(VERSION 3.20)

project(WebARKitLib)

set(CMAKE_CXX_STANDARD 14)

message(STATUS "CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

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
target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/include")
target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/calib3d/include")
target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/core/include")
target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/features2d/include")
target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/flann/include")
target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/imgproc/include")
target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/imgcodecs/include")
target_include_directories(WebARKitLib PRIVATE "${OPENCV_DIR}/modules/video/include")

target_link_libraries(WebARKitLib "${OPENCV_JS_DIR}/lib/*.a")