#include <WebARKitTrackers/WebARKitNFT/NFTTrackingConfig.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#else
#include <chrono>
#endif

double nftDefaultClockMs() {
#ifdef __EMSCRIPTEN__
  return emscripten_get_now();
#else
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
}
