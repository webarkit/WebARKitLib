#include <gtest/gtest.h>

#include <AR/ar.h>
#include <AR2/imageFormat.h>
#include <KPM/kpm.h>
#include <WebARKitLog.h>
#include <WebARKitTrackers/WebARKitNFT/NFTDetector.h>
#include <WebARKitTrackers/WebARKitNFT/NFTTrackingConfig.h>
#include <WebARKitTrackers/WebARKitNFT/SyncKpmDetector.h>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <vector>

namespace {

/** A KPM handle with the pinball marker loaded (page 0), plus what it needs to live. */
struct PinballKpm {
  ARParamLT *paramLT = nullptr;
  KpmHandle *handle = nullptr;
  int width = 0;
  int height = 0;

  PinballKpm() = default;
  PinballKpm(const PinballKpm &) = delete;
  PinballKpm &operator=(const PinballKpm &) = delete;
  ~PinballKpm() {
    if (handle) kpmDeleteHandle(&handle);
    if (paramLT) arParamLTFree(&paramLT);
  }
};

/**
 * Builds a KPM handle for a width x height frame with the pinball marker as page 0,
 * the way ARToolKitNFTCore::addNFTMarkers does for one marker. Returns false on failure.
 */
bool loadPinballKpm(PinballKpm &kpm, int width, int height) {
  ARParam param;
  if (arParamLoad("data/camera_para.dat", 1, &param) < 0) return false;
  if (param.xsize != width || param.ysize != height) {
    arParamChangeSize(&param, width, height, &param);
  }
  kpm.paramLT = arParamLTCreate(&param, AR_PARAM_LT_DEFAULT_OFFSET);
  if (!kpm.paramLT) return false;
  kpm.handle = kpmCreateHandle(kpm.paramLT);
  if (!kpm.handle) return false;

  KpmRefDataSet *refDataSet = nullptr;
  if (kpmLoadRefDataSet("data/pinball", "fset3", &refDataSet) < 0) return false;
  if (kpmChangePageNoOfRefDataSet(refDataSet, KpmChangePageNoAllPages, 0) < 0) {
    kpmDeleteRefDataSet(&refDataSet);
    return false;
  }
  const int result = kpmSetRefDataSet(kpm.handle, refDataSet);
  kpmDeleteRefDataSet(&refDataSet);  // KPM keeps its own copy
  if (result < 0) return false;
  kpm.width = width;
  kpm.height = height;
  return true;
}

/** Reads a JPEG into 8-bit luma, (77*R + 150*G + 29*B) >> 8 per pixel. Empty on failure. */
std::vector<ARUint8> loadLuma(const char *path, int &width, int &height) {
  std::vector<ARUint8> luma;
  FILE *fp = std::fopen(path, "rb");
  if (!fp) return luma;
  AR2JpegImageT *jpeg = ar2ReadJpegImage2(fp);
  std::fclose(fp);
  if (!jpeg) return luma;
  width = jpeg->xsize;
  height = jpeg->ysize;
  luma.resize(static_cast<size_t>(width) * height);
  if (jpeg->nc == 1) {
    std::copy(jpeg->image, jpeg->image + luma.size(), luma.begin());
  } else {
    for (size_t i = 0; i < luma.size(); i++) {
      const ARUint8 *rgb = jpeg->image + i * jpeg->nc;
      luma[i] = static_cast<ARUint8>((77 * rgb[0] + 150 * rgb[1] + 29 * rgb[2]) >> 8);
    }
  }
  ar2FreeJpegImage(&jpeg);
  return luma;
}

bool containsPage(const std::vector<NFTDetection> &detections, int page) {
  return std::any_of(detections.begin(), detections.end(),
                     [page](const NFTDetection &d) { return d.page == page; });
}

}  // namespace

TEST(NFTTrackingConfigTest, SingleThreadPresetMatchesTheSingleThreadBinding) {
  const NFTTrackingConfig config = singleThreadPreset();
  EXPECT_EQ(config.detector, NFTTrackingConfig::Detector::Sync);
  EXPECT_EQ(config.ar2Variant, NFTTrackingConfig::AR2Variant::SingleThread);
  EXPECT_FALSE(config.cpuDependentSearchSize);
  EXPECT_EQ(config.defaultDetectionIntervalMs, 300.0);
  EXPECT_TRUE(config.poseFilteringSupported);
  EXPECT_EQ(config.clock, &nftDefaultClockMs);
}

TEST(NFTTrackingConfigTest, ThreadedPresetMatchesTheThreadedBinding) {
  const NFTTrackingConfig config = threadedPreset();
  EXPECT_EQ(config.detector, NFTTrackingConfig::Detector::Threaded);
  EXPECT_EQ(config.ar2Variant, NFTTrackingConfig::AR2Variant::Threaded);
  EXPECT_TRUE(config.cpuDependentSearchSize);
  EXPECT_EQ(config.defaultDetectionIntervalMs, 0.0);
  EXPECT_FALSE(config.poseFilteringSupported);
  EXPECT_EQ(config.clock, &nftDefaultClockMs);
}

TEST(NFTTrackingConfigTest, LoggerIsNative) {
  WEBARKIT_LOGi("core logger test");
  SUCCEED();
}

TEST(SyncKpmDetectorTest, FindsPinballInTheSameCall) {
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  PinballKpm kpm;
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));

  SyncKpmDetector detector(kpm.handle);
  EXPECT_TRUE(detector.start(luma.data(), nullptr, 0));
  EXPECT_TRUE(detector.idle());

  std::vector<NFTDetection> out;
  int resultNum = 0;
  EXPECT_TRUE(detector.collect(out, resultNum));
  EXPECT_GE(resultNum, 1);
  EXPECT_TRUE(containsPage(out, 0));
  for (const NFTDetection &d : out) {
    if (d.page != 0) continue;
    // The marker is in front of the camera: a non-zero translation.
    EXPECT_NE(d.trans[2][3], 0.0f);
  }

  // The result is handed over once.
  EXPECT_FALSE(detector.collect(out, resultNum));
}

TEST(SyncKpmDetectorTest, SkippedPageIsNotReported) {
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  PinballKpm kpm;
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));

  SyncKpmDetector detector(kpm.handle);
  const int skipPages[] = {0};
  EXPECT_TRUE(detector.start(luma.data(), skipPages, 1));

  std::vector<NFTDetection> out;
  int resultNum = 0;
  EXPECT_TRUE(detector.collect(out, resultNum));
  EXPECT_FALSE(containsPage(out, 0));
}

TEST(SyncKpmDetectorTest, BlankFrameFindsNothing) {
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  PinballKpm kpm;
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));
  std::fill(luma.begin(), luma.end(), 0);

  SyncKpmDetector detector(kpm.handle);
  EXPECT_TRUE(detector.start(luma.data(), nullptr, 0));

  std::vector<NFTDetection> out;
  int resultNum = 0;
  EXPECT_TRUE(detector.collect(out, resultNum));
  EXPECT_TRUE(out.empty());
}
