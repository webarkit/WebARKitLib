# WebARKitNFT: NFT tracking helpers (trackingMod, markerDecompress, trackingSub)
# plus the ARToolKit sources they need (AR, ARICP, AR2, KPM, ARUtil).
# The source lists mirror jsartoolkitNFT/tools/makem.js. No OpenCV dependency.

option(WEBARKIT_NFT_THREADS "Build the threaded KPM worker (trackingSub) with pthreads" OFF)

get_filename_component(WEBARKITLIB_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ABSOLUTE)
set(ARTK_SRC ${WEBARKITLIB_ROOT}/lib/SRC)

file(GLOB AR_SOURCES
  ${ARTK_SRC}/AR/*.c
  ${ARTK_SRC}/AR/arLabelingSub/*.c
  ${ARTK_SRC}/ARICP/*.c
)

set(ARUTIL_SOURCES
  ${ARTK_SRC}/ARUtil/log.c
  ${ARTK_SRC}/ARUtil/file_utils.c
)

set(AR2_SOURCES
  handle.c imageSet.c jpeg.c marker.c featureMap.c featureSet.c
  selectTemplate.c surface.c tracking.c tracking2d.c matching.c matching2.c
  template.c searchPoint.c coord.c util.c
)
list(TRANSFORM AR2_SOURCES PREPEND ${ARTK_SRC}/AR2/)

set(KPM_SOURCES
  kpmHandle.cpp kpmRefDataSet.cpp kpmMatching.cpp kpmResult.cpp kpmUtil.cpp kpmFopen.c
  FreakMatcher/detectors/DoG_scale_invariant_detector.cpp
  FreakMatcher/detectors/gaussian_scale_space_pyramid.cpp
  FreakMatcher/detectors/gradients.cpp
  FreakMatcher/detectors/orientation_assignment.cpp
  FreakMatcher/detectors/pyramid.cpp
  FreakMatcher/facade/visual_database_facade.cpp
  FreakMatcher/matchers/hough_similarity_voting.cpp
  FreakMatcher/matchers/freak.cpp
  FreakMatcher/framework/date_time.cpp
  FreakMatcher/framework/image.cpp
  FreakMatcher/framework/logger.cpp
  FreakMatcher/framework/timers.cpp
)
list(TRANSFORM KPM_SOURCES PREPEND ${ARTK_SRC}/KPM/)

set(NFT_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/trackingMod.c
  ${CMAKE_CURRENT_SOURCE_DIR}/trackingMod2d.c
  ${CMAKE_CURRENT_SOURCE_DIR}/markerDecompress.c
)

if(WEBARKIT_NFT_THREADS)
  list(APPEND ARUTIL_SOURCES ${ARTK_SRC}/ARUtil/thread_sub.c)
  list(APPEND NFT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/trackingSub.c)
endif()

add_library(WebARKitNFT STATIC
  ${AR_SOURCES} ${ARUTIL_SOURCES} ${AR2_SOURCES} ${KPM_SOURCES} ${NFT_SOURCES}
)

target_include_directories(WebARKitNFT
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${WEBARKITLIB_ROOT}/include
  PRIVATE
    ${ARTK_SRC}/KPM/FreakMatcher
)

target_compile_definitions(WebARKitNFT PUBLIC HAVE_NFT)

if(EMSCRIPTEN)
  # libjpeg and zlib come from the Emscripten ports.
  target_compile_options(WebARKitNFT PUBLIC -sUSE_LIBJPEG=1 -sUSE_ZLIB=1)
  target_link_options(WebARKitNFT PUBLIC -sUSE_LIBJPEG=1 -sUSE_ZLIB=1)
  if(WEBARKIT_NFT_THREADS)
    target_compile_options(WebARKitNFT PUBLIC -pthread)
    target_link_options(WebARKitNFT PUBLIC -pthread)
  endif()
else()
  find_package(JPEG REQUIRED)
  find_package(ZLIB REQUIRED)
  target_link_libraries(WebARKitNFT PUBLIC JPEG::JPEG ZLIB::ZLIB)
  if(WEBARKIT_NFT_THREADS)
    find_package(Threads REQUIRED)
    target_link_libraries(WebARKitNFT PUBLIC Threads::Threads)
  endif()
endif()
