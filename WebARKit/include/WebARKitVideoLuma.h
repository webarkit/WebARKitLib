#ifndef WEBARKITVIDEOLUMA_H
#define WEBARKITVIDEOLUMA_H

#include <cstdint>
#include <cstdbool>
#include <memory>
#include <WebARKit/WebARKitLog.h>

#ifdef __wasm_simd128__
#include <wasm_simd128.h> // For SIMD operations
#endif

namespace webarkit {

// CCIR 601 recommended values. See
// http://www.poynton.com/notes/colour_and_gamma/ColorFAQ.html#RTFToC11 .
const uint8_t R8_CCIR601 = 77;
const uint8_t G8_CCIR601 = 150;
const uint8_t B8_CCIR601 = 29;

struct WebARKitLumaInfo {
  int xsize;
  int ysize;
  int buffSize;
  bool simd128;
  std::unique_ptr<uint8_t[]> buff;
};

#ifdef __wasm_simd128__
static void webarkitVideoLumaRGBAtoLuma_Emscripten_simd128(uint8_t *__restrict dest,
    uint8_t *__restrict src,
    int32_t numPixels);
#endif

static void webarkitVideoLuma_default(uint8_t *__restrict dest, uint8_t *__restrict src,
    int32_t numPixels);

WebARKitLumaInfo *webarkitVideoLumaInit(int xsize, int ysize, bool simd128);

uint8_t *__restrict webarkitVideoLuma(WebARKitLumaInfo *vli,
    const uint8_t *__restrict dataPtr);

int webarkitVideoLumaFinal(WebARKitLumaInfo **vli_p);

} // namespace webarkit

#endif // WEBARKITVIDEOLUMA_H