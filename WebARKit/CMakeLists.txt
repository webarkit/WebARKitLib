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

set(BUILD_opencv_dnn_plugins OFF)
set(BUILD_opencv_highgui_plugins OFF)
set(BUILD_opencv_videoio_plugins OFF)
set(BUILD_opencv_intensity_transform OFF)
set(BUILD_opencv_phase_unwrapping OFF)
set(BUILD_opencv_plot OFF)
set(BUILD_opencv_reg OFF)
set(BUILD_opencv_surface_matching OFF)
set(BUILD_opencv_bioinspired OFF)
set(BUILD_opencv_fuzzy OFF)
set(BUILD_opencv_hfs OFF)
set(BUILD_opencv_img_hash OFF)
set(BUILD_opencv_line_descriptor OFF)
set(BUILD_opencv_saliency OFF)
set(BUILD_opencv_rapid OFF)
set(BUILD_opencv_rgbd OFF)
set(BUILD_opencv_shape OFF)
set(BUILD_opencv_rapid OFF)
set(BUILD_opencv_structured_light OFF)
set(BUILD_opencv_java OFF)

#set(BUILD_opencv_xfeatures2d ON)
#set(BUILD_opencv_contrib ON)
#set(OPENCV_ENABLE_NONFREE ON)

set(BUILD_opencv_ximgproc OFF)
set(BUILD_opencv_bgsegm OFF)
set(BUILD_opencv_optflow OFF)
set(BUILD_opencv_opencv_superres OFF)
set(BUILD_opencv_tracking OFF)
set(BUILD_opencv_stereo OFF)
set(BUILD_opencv_version OFF)

set(OPENCV_EXTRA_MODULES_PATH ${opencv_contrib_SOURCE_DIR}/modules)

FetchContent_Declare(
  opencv
  GIT_REPOSITORY https://github.com/webarkit/opencv.git
  GIT_TAG 889dfa15f1cd60e571d33b23b87a1d506d381ea9
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)

FetchContent_Declare(
  opencv_contrib
  GIT_REPOSITORY https://github.com/webarkit/opencv_contrib.git
  GIT_TAG 5438f53d81e1c1137258e18308ebf3f4daa2f2ff
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(opencv opencv_contrib)
set(OPENCV_EXTRA_MODULES_PATH ${opencv_contrib_SOURCE_DIR}/modules)

#set(BUILD_opencv_xfeatures2d ON)

get_filename_component(PARENT_DIR ./ ABSOLUTE)

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
        #${OPENCV_MODULE_opencv_xfeatures2d_LOCATION}/include
        ${opencv_contrib_SOURCE_DIR}/modules/xfeatures2d/include
        )

target_link_libraries(WebARKitLib opencv_calib3d opencv_core opencv_features2d opencv_flann opencv_imgproc opencv_imgcodecs opencv_video)