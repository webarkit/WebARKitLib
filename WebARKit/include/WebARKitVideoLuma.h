#ifndef WEBARKITVIDEOLUMA_H
#define WEBARKITVIDEOLUMA_H

#include <stdint.h>  // For standard integer types like uint8_t, int32_t
#include <stdbool.h> // For boolean type
#include <stdlib.h>  // For memory allocation functions
#include <memory>    // For std::unique_ptr

#ifdef __EMSCRIPTEN_SIMD128__
#include <wasm_simd128.h> // For SIMD operations
#endif

namespace webarkit {

// CCIR 601 recommended values. See
// http://www.poynton.com/notes/colour_and_gamma/ColorFAQ.html#RTFToC11 .
const uint8_t R8_CCIR601 = 77;
const uint8_t G8_CCIR601 = 150;
const uint8_t B8_CCIR601 = 29;

struct ARVideoLumaInfo {
  int xsize;
  int ysize;
  int buffSize;
  bool simd128;
  std::unique_ptr<uint8_t[]> buff;
};

#ifdef __EMSCRIPTEN_SIMD128__
static void arVideoLumaRGBAtoL_Emscripten_simd128(uint8_t *__restrict dest,
    uint8_t *__restrict src,
    int32_t numPixels);
#endif

static void arVideoLuma_default(uint8_t *__restrict dest, uint8_t *__restrict src,
    int32_t numPixels);

ARVideoLumaInfo *arVideoLumaInit(int xsize, int ysize, bool simd128);

uint8_t *__restrict arVideoLuma(ARVideoLumaInfo *vli,
    const uint8_t *__restrict dataPtr);

int arVideoLumaFinal(ARVideoLumaInfo **vli_p);

} // namespace webarkit

#endif // WEBARKITVIDEOLUMA_H