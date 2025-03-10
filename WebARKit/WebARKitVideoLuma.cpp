#include <WebARKitVideoLuma.h>
#include <stdio.h> // Include this header for printf

namespace webarkit {

WebARKitLumaInfo *arVideoLumaInit(int xsize, int ysize, bool simd128) {
  WebARKitLumaInfo *vli = new WebARKitLumaInfo;

  if (!vli) {
    printf("Out of memory!!\n");
    return nullptr;
  }
  vli->xsize = xsize;
  vli->ysize = ysize;
  vli->buffSize = xsize * ysize;
  vli->simd128 = simd128;
  vli->buff = std::make_unique<uint8_t[]>(vli->buffSize);
  if (!vli->buff) {
    printf("Out of memory!!\n");
    delete vli;
    return nullptr;
  }

  return vli;
}

uint8_t *__restrict arVideoLuma(WebARKitLumaInfo *vli,
                                const uint8_t *__restrict dataPtr) {
  unsigned int p, q;

  if (vli->simd128 == true) {
    printf("With simd128!!!\n");
#ifdef __EMSCRIPTEN_SIMD128__
    arVideoLumaRGBAtoL_Emscripten_simd128(
        vli->buff.get(), (unsigned char *__restrict)dataPtr, vli->buffSize);
    return vli->buff.get();
#else
    printf("SIMD128 not supported!!!\n");
    arVideoLuma_default(vli->buff.get(), (unsigned char *__restrict)dataPtr, vli->buffSize);
    return vli->buff.get();
#endif
  } else {
    printf("Without simd128!!!\n");
    arVideoLuma_default(vli->buff.get(), (unsigned char *__restrict)dataPtr, vli->buffSize);
    return vli->buff.get();
  }
}

int arVideoLumaFinal(WebARKitLumaInfo **vli_p) {
  if (!vli_p)
    return -1;
  if (!*vli_p)
    return 0;

  delete *vli_p;
  *vli_p = nullptr;

  return 0;
}

static void arVideoLuma_default(uint8_t *__restrict dest,
                                 uint8_t *__restrict src, int32_t numPixels) {
  unsigned int p, q;
  printf("default luma conversion!!!\n");
  q = 0;
  for (p = 0; p < numPixels; p++) {
    dest[p] = (R8_CCIR601 * src[q + 0] + G8_CCIR601 * src[q + 1] +
                    B8_CCIR601 * src[q + 2]) >>
                   8;
    q += 4;
  }
}

#ifdef __EMSCRIPTEN_SIMD128__
static void arVideoLumaRGBAtoL_Emscripten_simd128(uint8_t *__restrict dest,
                                                       uint8_t *__restrict src,
                                                       int32_t numPixels) {

  printf("using arVideoLumaRGBAtoL_Emscripten_simd128_fast !!!\n");

  v128_t *pin = (v128_t *)src;
  int64_t *pout = (int64_t *)dest;
  int numPixelsDiv8 = numPixels / 8;

  v128_t maskRedBlue = wasm_i32x4_splat(0x00FF00FF);
  v128_t scaleRedBlue =
      wasm_i32x4_splat((uint32_t)B8_CCIR601 << 16 | R8_CCIR601);
  v128_t scaleGreen = wasm_i32x4_splat(G8_CCIR601);
  do {
    v128_t pixels1 = wasm_v128_load(pin); // Load 16 bytes (4 pixels) from src
    v128_t pixels2 = wasm_v128_load(pin + 1);
    pin += 2;

    v128_t g1 = wasm_u16x8_shr(pixels1, 8);
    v128_t g2 = wasm_u16x8_shr(pixels2, 8);

    v128_t rb1 = wasm_v128_and(pixels1, maskRedBlue);
    v128_t rb2 = wasm_v128_and(pixels2, maskRedBlue);

    g1 = wasm_i32x4_dot_i16x8(g1, scaleGreen);
    g2 = wasm_i32x4_dot_i16x8(g2, scaleGreen);
    rb1 = wasm_i32x4_dot_i16x8(rb1, scaleRedBlue);
    rb2 = wasm_i32x4_dot_i16x8(rb2, scaleRedBlue);

    v128_t y1 = wasm_i32x4_add(g1, rb1);
    v128_t y2 = wasm_i32x4_add(g2, rb2);

    y1 = wasm_u32x4_shr(y1, 8);
    y2 = wasm_u32x4_shr(y2, 8);

    v128_t y = wasm_i16x8_narrow_i32x4(y1, y2);
    y = wasm_u8x16_narrow_i16x8(y, y);

    *pout = wasm_i64x2_extract_lane(y, 0);

    pout++;
    numPixelsDiv8--;
  } while (numPixelsDiv8);
}
#endif

} // namespace webarkit