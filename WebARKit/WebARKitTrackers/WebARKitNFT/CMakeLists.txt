# WebARKitNFT: NFT tracking helpers (trackingMod, markerDecompress, trackingSub)
# plus the ARToolKit sources they need (AR, ARICP, AR2, KPM, ARUtil).
# The source lists mirror jsartoolkitNFT/tools/makem.js. No OpenCV dependency.

option(WEBARKIT_NFT_THREADS "Build the threaded KPM worker (trackingSub); on Emscripten also enables -pthread" OFF)

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
  # thread helpers: AR2 handle.c/tracking.c call them even without the KPM worker
  ${ARTK_SRC}/ARUtil/thread_sub.c
  # minizip, used by file_utils.c (zip/unzip helpers)
  ${ARTK_SRC}/ARUtil/crypt.c
  ${ARTK_SRC}/ARUtil/ioapi.c
  ${ARTK_SRC}/ARUtil/unzip.c
  ${ARTK_SRC}/ARUtil/zip.c
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
  ${CMAKE_CURRENT_SOURCE_DIR}/NFTClock.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SyncKpmDetector.cpp
  # the native logger (WebARKit/include/WebARKitLog.h) used by the tracking core
  ${WEBARKITLIB_ROOT}/WebARKit/WebARKitLog.cpp
)

if(WEBARKIT_NFT_THREADS)
  list(APPEND NFT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/trackingSub.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ThreadedKpmDetector.cpp)
endif()

add_library(WebARKitNFT STATIC
  ${AR_SOURCES} ${ARUTIL_SOURCES} ${AR2_SOURCES} ${KPM_SOURCES} ${NFT_SOURCES}
)

target_include_directories(WebARKitNFT
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${WEBARKITLIB_ROOT}/include
    ${WEBARKITLIB_ROOT}/WebARKit/include
  PRIVATE
    ${ARTK_SRC}/KPM/FreakMatcher
)

target_compile_definitions(WebARKitNFT PUBLIC HAVE_NFT)
if(WEBARKIT_NFT_THREADS)
  # Seen by everything that links the target, so the core selects the threaded detector.
  target_compile_definitions(WebARKitNFT PUBLIC WEBARKIT_NFT_THREADS)
endif()

if(EMSCRIPTEN)
  # Emscripten has no fopen64/ftello64/fseeko64: let minizip use the plain calls.
  set_source_files_properties(
    ${ARTK_SRC}/ARUtil/ioapi.c ${ARTK_SRC}/ARUtil/unzip.c ${ARTK_SRC}/ARUtil/zip.c
    PROPERTIES COMPILE_DEFINITIONS USE_FILE32API
  )
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
  find_package(Threads REQUIRED)
  target_link_libraries(WebARKitNFT PUBLIC JPEG::JPEG ZLIB::ZLIB Threads::Threads)
endif()
