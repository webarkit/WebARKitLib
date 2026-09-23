# Shared webarkit/opencv-em release coordinates.
# Bump OPENCV_EM_RELEASE (and the three hashes below) to upgrade opencv-em.
set(OPENCV_EM_RELEASE "0.2.0")
set(OPENCV_EM_BASE_URL "https://github.com/webarkit/opencv-em/releases/download/${OPENCV_EM_RELEASE}")

set(OPENCV_NATIVE_URL "${OPENCV_EM_BASE_URL}/opencv-4.12.0.zip")
set(OPENCV_NATIVE_HASH "SHA256=eb68b3c6cac2781f6bbbe747d9ac8f27c5d716471da82d6c4fd79f26a18263b4")

set(OPENCV_EMSCRIPTEN_URL "${OPENCV_EM_BASE_URL}/opencv-js-4.12.0-emcc-3.1.69.zip")
set(OPENCV_EMSCRIPTEN_HASH "SHA256=3a9509615bed922b058e3201007c8b9b29c1e5aa3dd0750676b7d847738ce2c7")

set(OPENCV_EMSCRIPTEN_SIMD_URL "${OPENCV_EM_BASE_URL}/opencv-js-4.12.0-emcc-3.1.69-simd.zip")
set(OPENCV_EMSCRIPTEN_SIMD_HASH "SHA256=3600fd9d0422cb1fc19306bb1f876a578dff970f207947b346180993dfa27026")
