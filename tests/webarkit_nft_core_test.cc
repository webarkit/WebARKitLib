#include <gtest/gtest.h>

#include <AR/ar.h>
#include <AR/paramGL.h>
#include <AR2/imageFormat.h>
#include <KPM/kpm.h>
#include <WebARKitLog.h>
// The Emscripten logger that the jsartoolkitNFT bindings use, after the native one: see
// LogHeadersTest.
#include <WebARKit/WebARKitLog.h>
#include <WebARKitTrackers/WebARKitNFT/ARToolKitNFTCore.h>
#include <WebARKitTrackers/WebARKitNFT/NFTDetector.h>
#include <WebARKitTrackers/WebARKitNFT/NFTRuntimeSettings.h>
#include <WebARKitTrackers/WebARKitNFT/NFTTrackingConfig.h>
#include <WebARKitTrackers/WebARKitNFT/SyncKpmDetector.h>
#ifdef WEBARKIT_NFT_THREADS
#include <WebARKitTrackers/WebARKitNFT/ThreadedKpmDetector.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

// The native logger (WebARKit/include/WebARKitLog.h, used by the core) and the Emscripten
// one (include/WebARKit/WebARKitLog.h, used by the bindings) are both included above. They
// shared an include guard, so the second one was skipped and its functions were undeclared
// (#84). This file compiles only if each declares its API.
TEST(LogHeadersTest, BothLoggersAreDeclaredInOneFile) {
  static_assert(std::is_same<decltype(webarkitLOGi(std::string())), void>::value,
                "include/WebARKit/WebARKitLog.h declares webarkitLOGi()");
  EXPECT_EQ(WEBARKIT_LOG_LEVEL_DEBUG, 0);
}

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
  EXPECT_TRUE(config.poseFilteringSupported);  // #692: the threaded build used to never filter
  EXPECT_EQ(config.clock, &nftDefaultClockMs);
}

TEST(NFTTrackingConfigTest, ValueInitialisedConfigIsTheSingleThreadPreset) {
  const NFTTrackingConfig config{};
  const NFTTrackingConfig preset = singleThreadPreset();
  EXPECT_EQ(config.detector, preset.detector);
  EXPECT_EQ(config.ar2Variant, preset.ar2Variant);
  EXPECT_EQ(config.cpuDependentSearchSize, preset.cpuDependentSearchSize);
  EXPECT_EQ(config.defaultDetectionIntervalMs, preset.defaultDetectionIntervalMs);
  EXPECT_EQ(config.poseFilteringSupported, preset.poseFilteringSupported);
  EXPECT_EQ(config.clock, preset.clock);
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

#ifdef WEBARKIT_NFT_THREADS

namespace {

/** Polls collect() every 10 ms until it returns a pass or 10 s have gone by. */
bool pollCollect(NFTDetector &detector, std::vector<NFTDetection> &out, int &resultNum) {
  for (int i = 0; i < 1000; i++) {
    if (detector.collect(out, resultNum)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace

TEST(ThreadedKpmDetectorTest, FindsPinballOnALaterCollect) {
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  PinballKpm kpm;
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));

  std::unique_ptr<ThreadedKpmDetector> detector = ThreadedKpmDetector::create(kpm.handle);
  ASSERT_NE(detector, nullptr);
  EXPECT_TRUE(detector->idle());

  // A threaded pass never finishes inside start().
  EXPECT_FALSE(detector->start(luma.data(), nullptr, 0));
  EXPECT_FALSE(detector->idle());

  std::vector<NFTDetection> out;
  int resultNum = 0;
  ASSERT_TRUE(pollCollect(*detector, out, resultNum));
  EXPECT_GE(resultNum, 1);
  EXPECT_TRUE(containsPage(out, 0));
  for (const NFTDetection &d : out) {
    if (d.page != 0) continue;
    EXPECT_NE(d.trans[2][3], 0.0f);
  }
  EXPECT_TRUE(detector->idle());

  // The result is handed over once.
  EXPECT_FALSE(detector->collect(out, resultNum));
}

TEST(ThreadedKpmDetectorTest, SkippedPageIsNotReported) {
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  PinballKpm kpm;
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));

  std::unique_ptr<ThreadedKpmDetector> detector = ThreadedKpmDetector::create(kpm.handle);
  ASSERT_NE(detector, nullptr);
  const int skipPages[] = {0};
  EXPECT_FALSE(detector->start(luma.data(), skipPages, 1));

  std::vector<NFTDetection> out;
  int resultNum = 0;
  ASSERT_TRUE(pollCollect(*detector, out, resultNum));
  EXPECT_FALSE(containsPage(out, 0));
}

TEST(ThreadedKpmDetectorTest, BlankFrameFinishesWithNothing) {
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  PinballKpm kpm;
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));
  std::fill(luma.begin(), luma.end(), 0);

  std::unique_ptr<ThreadedKpmDetector> detector = ThreadedKpmDetector::create(kpm.handle);
  ASSERT_NE(detector, nullptr);
  EXPECT_FALSE(detector->start(luma.data(), nullptr, 0));

  std::vector<NFTDetection> out;
  int resultNum = 0;
  ASSERT_TRUE(pollCollect(*detector, out, resultNum));
  EXPECT_TRUE(out.empty());
  EXPECT_TRUE(detector->idle());
}

TEST(ThreadedKpmDetectorTest, CollectWithoutAStartFindsNothing) {
  PinballKpm kpm;
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));

  std::unique_ptr<ThreadedKpmDetector> detector = ThreadedKpmDetector::create(kpm.handle);
  ASSERT_NE(detector, nullptr);
  std::vector<NFTDetection> out;
  int resultNum = 0;
  EXPECT_FALSE(detector->collect(out, resultNum));
}

TEST(ThreadedKpmDetectorTest, CreateWithoutAHandleReturnsNull) {
  EXPECT_EQ(ThreadedKpmDetector::create(nullptr), nullptr);
}

TEST(ThreadedKpmDetectorTest, WaitIdleDropsTheRunningSearch) {
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  PinballKpm kpm;
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));

  std::unique_ptr<ThreadedKpmDetector> detector = ThreadedKpmDetector::create(kpm.handle);
  ASSERT_NE(detector, nullptr);
  EXPECT_FALSE(detector->start(luma.data(), nullptr, 0));
  detector->waitIdle();
  EXPECT_TRUE(detector->idle());

  std::vector<NFTDetection> out;
  int resultNum = 0;
  EXPECT_FALSE(detector->collect(out, resultNum));

  // The worker is free again: a new search can start and finish.
  EXPECT_FALSE(detector->start(luma.data(), nullptr, 0));
  ASSERT_TRUE(pollCollect(*detector, out, resultNum));
  EXPECT_TRUE(containsPage(out, 0));
}

TEST(ThreadedKpmDetectorTest, StartWhileSearchingIsRefused) {
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  PinballKpm kpm;
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));

  std::unique_ptr<ThreadedKpmDetector> detector = ThreadedKpmDetector::create(kpm.handle);
  ASSERT_NE(detector, nullptr);
  EXPECT_FALSE(detector->start(luma.data(), nullptr, 0));
  EXPECT_FALSE(detector->idle());
  EXPECT_FALSE(detector->start(luma.data(), nullptr, 0));
  EXPECT_FALSE(detector->idle());

  std::vector<NFTDetection> out;
  int resultNum = 0;
  ASSERT_TRUE(pollCollect(*detector, out, resultNum));
  EXPECT_TRUE(containsPage(out, 0));
}

TEST(ThreadedKpmDetectorTest, DestroyWhileSearching) {
  int width = 0, height = 0;
  std::vector<ARUint8> luma = loadLuma("data/pinball-demo.jpg", width, height);
  ASSERT_FALSE(luma.empty());
  PinballKpm kpm;
  ASSERT_TRUE(loadPinballKpm(kpm, width, height));

  std::unique_ptr<ThreadedKpmDetector> detector = ThreadedKpmDetector::create(kpm.handle);
  ASSERT_NE(detector, nullptr);
  EXPECT_FALSE(detector->start(luma.data(), nullptr, 0));
  detector.reset();  // waits for the search, then stops the worker

  // The KPM handle is free to go: the worker no longer touches it (PinballKpm's destructor).
  SUCCEED();
}

#endif  // WEBARKIT_NFT_THREADS

namespace {

const int kFrameWidth = 2000;   // pinball-demo.jpg
const int kFrameHeight = 1500;

/**
 * Makes a core ready for markers the way the bindings do: load the camera, setup() for the
 * test frame, then setupAR2(). Returns false on failure.
 */
bool makeReady(ARToolKitNFTCore &core) {
  const int cameraID = ARToolKitNFTCore::loadCamera("data/camera_para.dat");
  if (cameraID < 0) return false;
  if (core.setup(kFrameWidth, kFrameHeight, cameraID) < 0) return false;
  return core.setupAR2() == 0;
}

}  // namespace

TEST(CoreCameraTest, LoadSetupAndLens) {
  ARToolKitNFTCore core(singleThreadPreset());
  const int cameraID = ARToolKitNFTCore::loadCamera("data/camera_para.dat");
  ASSERT_GE(cameraID, 0);
  EXPECT_GE(core.setup(kFrameWidth, kFrameHeight, cameraID), 0);
  EXPECT_EQ(core.cameraParam().xsize, kFrameWidth);
  EXPECT_EQ(core.cameraParam().ysize, kFrameHeight);
  EXPECT_NE(core.cameraParamLT(), nullptr);

  const ARdouble *lens = core.cameraLens();
  ASSERT_NE(lens, nullptr);
  EXPECT_TRUE(std::any_of(lens, lens + 16, [](ARdouble v) { return v != 0.0; }));
}

TEST(CoreCameraTest, MissingCameraFileFails) {
  EXPECT_EQ(ARToolKitNFTCore::loadCamera("data/does-not-exist.dat"), -1);
}

TEST(CoreCameraTest, UnknownCameraIdFails) {
  ARToolKitNFTCore core(singleThreadPreset());
  EXPECT_EQ(core.setCamera(0, 9999), -1);
}

TEST(CoreCameraTest, SetupRejectsInvalidFrameSizes) {
  ARToolKitNFTCore core(singleThreadPreset());
  const int cameraID = ARToolKitNFTCore::loadCamera("data/camera_para.dat");
  ASSERT_GE(cameraID, 0);

  // Rejected before anything changes: no frame buffers and no camera.
  EXPECT_EQ(core.setup(0, kFrameHeight, cameraID), -1);
  EXPECT_EQ(core.setup(kFrameWidth, -1, cameraID), -1);
  // The RGBA frame, 4 * 30000 * 30000 bytes, does not fit an int.
  EXPECT_EQ(core.setup(30000, 30000, cameraID), -1);
  EXPECT_EQ(core.cameraParamLT(), nullptr);

  // After a valid setup() an invalid one keeps the camera.
  ASSERT_GE(core.setup(kFrameWidth, kFrameHeight, cameraID), 0);
  ARParamLT *paramLT = core.cameraParamLT();
  ASSERT_NE(paramLT, nullptr);
  EXPECT_EQ(core.setup(-kFrameWidth, kFrameHeight, cameraID), -1);
  EXPECT_EQ(core.cameraParamLT(), paramLT);
}

TEST(CoreCameraTest, SetCameraTwice) {
  ARToolKitNFTCore core(singleThreadPreset());
  const int cameraID = ARToolKitNFTCore::loadCamera("data/camera_para.dat");
  ASSERT_GE(cameraID, 0);
  const int id = core.setup(kFrameWidth, kFrameHeight, cameraID);
  ASSERT_GE(id, 0);
  ASSERT_EQ(core.setupAR2(), 0);

  // A second setCamera frees paramLT and the AR2 handle and rebuilds them; setupAR2 recreates
  // the handles from the new paramLT, and markers load against them.
  EXPECT_EQ(core.setCamera(id, cameraID), 0);
  EXPECT_NE(core.cameraParamLT(), nullptr);
  EXPECT_EQ(core.setupAR2(), 0);
  EXPECT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
}

TEST(CoreCameraTest, ProjectionPlanes) {
  ARToolKitNFTCore core(singleThreadPreset());
  EXPECT_EQ(core.getProjectionNearPlane(), 0.1);
  EXPECT_EQ(core.getProjectionFarPlane(), 1000.0);

  ASSERT_TRUE(makeReady(core));
  std::vector<ARdouble> before(core.cameraLens(), core.cameraLens() + 16);
  core.setProjectionNearPlane(1.0);
  core.setProjectionFarPlane(500.0);
  EXPECT_EQ(core.getProjectionNearPlane(), 1.0);
  EXPECT_EQ(core.getProjectionFarPlane(), 500.0);

  // The plane setters rebuild the lens (D5); recalculating again changes nothing.
  EXPECT_NE(std::vector<ARdouble>(core.cameraLens(), core.cameraLens() + 16), before);
  const std::vector<ARdouble> after(core.cameraLens(), core.cameraLens() + 16);
  core.recalculateCameraLens();
  EXPECT_EQ(std::vector<ARdouble>(core.cameraLens(), core.cameraLens() + 16), after);
}

TEST(CoreMarkersTest, LoadsTwoMarkersInOneCall) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  EXPECT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));
  EXPECT_EQ(core.markerCount(), 2);
  const nftMarker marker = core.getNFTData(0);
  EXPECT_EQ(marker.id_NFT, 0);
  EXPECT_GT(marker.width_NFT, 0);
  EXPECT_GT(marker.height_NFT, 0);
  EXPECT_GT(marker.dpi_NFT, 0);
  EXPECT_EQ(core.getNFTData(1).id_NFT, 1);
}

TEST(CoreMarkersTest, LoadsMarkersIncrementally) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  EXPECT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  EXPECT_EQ(core.addNFTMarkers({"data/kuva"}), std::vector<int>({1}));
  EXPECT_EQ(core.markerCount(), 2);
}

TEST(CoreMarkersTest, MissingMarkerReturnsEmpty) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));

  EXPECT_TRUE(core.addNFTMarkers({"data/does-not-exist"}).empty());
  EXPECT_EQ(core.markerCount(), 1);

  // A failed batch leaves the earlier markers and the id sequence untouched.
  EXPECT_TRUE(core.addNFTMarkers({"data/kuva", "data/does-not-exist"}).empty());
  EXPECT_EQ(core.markerCount(), 1);
  EXPECT_EQ(core.addNFTMarkers({"data/kuva"}), std::vector<int>({1}));
}

TEST(CoreMarkersTest, TooManyMarkersReturnsEmpty) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  const std::vector<std::string> paths(PAGES_MAX + 1, "data/pinball");
  EXPECT_TRUE(core.addNFTMarkers(paths).empty());
  EXPECT_EQ(core.markerCount(), 0);
}

TEST(CoreMarkersTest, MarkerStateOutOfRange) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  EXPECT_EQ(core.markerState(0), nullptr);
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));

  EXPECT_EQ(core.markerState(-1), nullptr);
  EXPECT_EQ(core.markerState(core.markerCount()), nullptr);
  ASSERT_NE(core.markerState(0), nullptr);
  EXPECT_FALSE(core.markerState(0)->tracking);
}

TEST(CoreMarkersTest, DecompressMissingArchiveFails) {
  ARToolKitNFTCore core(singleThreadPreset());
  EXPECT_EQ(core.decompressZFT("data/does-not-exist.zft", "data/zft-temp"), -1);
}

TEST(CoreLifecycleTest, TeardownThenDestroy) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));

  EXPECT_EQ(core.teardown(), 0);
  EXPECT_EQ(core.markerCount(), 0);
  EXPECT_EQ(core.cameraParamLT(), nullptr);
  EXPECT_EQ(core.markerState(0), nullptr);
  // A second teardown (the destructor's) finds nothing left to free.
  EXPECT_EQ(core.teardown(), 0);
}

TEST(CoreLifecycleTest, TeardownKeepsMarkerData) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));
  const nftMarker before = core.getNFTData(0);

  // As in the bindings, getNFTData() still answers after teardown() (it does not abort).
  EXPECT_EQ(core.teardown(), 0);
  const nftMarker after = core.getNFTData(0);
  EXPECT_EQ(after.id_NFT, before.id_NFT);
  EXPECT_EQ(after.width_NFT, before.width_NFT);
  EXPECT_EQ(after.height_NFT, before.height_NFT);
  EXPECT_EQ(after.dpi_NFT, before.dpi_NFT);
}

TEST(CoreLifecycleTest, MarkersLoadedAfterTeardownReplaceTheOldData) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));
  const nftMarker pinball = core.getNFTData(0);
  const nftMarker kuva = core.getNFTData(1);
  ASSERT_TRUE(pinball.width_NFT != kuva.width_NFT || pinball.height_NFT != kuva.height_NFT);

  // After teardown() the ids restart at 0: getNFTData(0) is then the new marker, not the old
  // one still stored at that index.
  ASSERT_EQ(core.teardown(), 0);
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/kuva"}), std::vector<int>({0}));
  EXPECT_EQ(core.markerCount(), 1);
  const nftMarker reloaded = core.getNFTData(0);
  EXPECT_EQ(reloaded.id_NFT, 0);
  EXPECT_EQ(reloaded.width_NFT, kuva.width_NFT);
  EXPECT_EQ(reloaded.height_NFT, kuva.height_NFT);
  EXPECT_EQ(reloaded.dpi_NFT, kuva.dpi_NFT);
}

#ifdef WEBARKIT_NFT_THREADS

TEST(CoreMarkersTest, ThreadedPresetLoadsMarkersIncrementally) {
  ARToolKitNFTCore core(threadedPreset());
  ASSERT_TRUE(makeReady(core));
  EXPECT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  EXPECT_EQ(core.addNFTMarkers({"data/kuva"}), std::vector<int>({1}));
  EXPECT_EQ(core.markerCount(), 2);
  EXPECT_GT(core.getNFTData(1).width_NFT, 0);
}

TEST(CoreLifecycleTest, ThreadedTeardownThenDestroy) {
  ARToolKitNFTCore core(threadedPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  EXPECT_EQ(core.teardown(), 0);
  EXPECT_EQ(core.markerCount(), 0);
}

#endif  // WEBARKIT_NFT_THREADS

namespace {

/** One camera frame as the core takes it: RGBA and its 8-bit luma. */
struct Frame {
  std::vector<ARUint8> rgba;
  std::vector<ARUint8> luma;
};

/** Recomputes the luma of a frame from its RGBA, (77*R + 150*G + 29*B) >> 8 per pixel. */
void computeLuma(Frame &frame) {
  frame.luma.resize(frame.rgba.size() / 4);
  for (size_t i = 0; i < frame.luma.size(); i++) {
    const ARUint8 *rgba = &frame.rgba[i * 4];
    frame.luma[i] = static_cast<ARUint8>((77 * rgba[0] + 150 * rgba[1] + 29 * rgba[2]) >> 8);
  }
}

/** Reads a kFrameWidth x kFrameHeight JPEG into an RGBA frame (alpha 255). Empty on failure. */
Frame loadFrame(const char *path) {
  Frame frame;
  // As loadLuma(): ar2ReadJpegImage(path, nullptr) crashes, so open the file here.
  FILE *fp = std::fopen(path, "rb");
  if (!fp) return frame;
  AR2JpegImageT *jpeg = ar2ReadJpegImage2(fp);
  std::fclose(fp);
  if (!jpeg) return frame;
  if (jpeg->xsize == kFrameWidth && jpeg->ysize == kFrameHeight && (jpeg->nc == 3 || jpeg->nc == 1)) {
    const size_t pixels = static_cast<size_t>(kFrameWidth) * kFrameHeight;
    frame.rgba.resize(pixels * 4);
    for (size_t i = 0; i < pixels; i++) {
      const ARUint8 *src = jpeg->image + i * jpeg->nc;
      ARUint8 *dst = &frame.rgba[i * 4];
      dst[0] = src[0];
      dst[1] = src[jpeg->nc == 3 ? 1 : 0];
      dst[2] = src[jpeg->nc == 3 ? 2 : 0];
      dst[3] = 255;
    }
    computeLuma(frame);
  }
  ar2FreeJpegImage(&jpeg);
  return frame;
}

/**
 * The photo with the kuva print painted over: its bounding box, x in [1096, 1705] and
 * y in [380, 1165] (from the quad in jsartoolkitNFT's tests/node/multi-marker.test.js),
 * filled with the paper colour RGB (217, 212, 202).
 */
Frame paintOverKuva(const Frame &photo) {
  Frame frame = photo;
  for (int y = 380; y <= 1165; y++) {
    for (int x = 1096; x <= 1705; x++) {
      ARUint8 *rgba = &frame.rgba[(static_cast<size_t>(y) * kFrameWidth + x) * 4];
      rgba[0] = 217;
      rgba[1] = 212;
      rgba[2] = 202;
      rgba[3] = 255;
    }
  }
  computeLuma(frame);
  return frame;
}

struct TestFrames {
  Frame both;         // pinball-demo.jpg: the pinball print on the left, kuva on the right
  Frame pinballOnly;  // the same photo with the kuva print painted over
  Frame blank;        // all zeros
};

/** The test frames, decoded once. both and pinballOnly are empty when the photo cannot be read. */
const TestFrames &testFrames() {
  static const TestFrames frames = [] {
    TestFrames f;
    f.both = loadFrame("data/pinball-demo.jpg");
    if (!f.both.rgba.empty()) f.pinballOnly = paintOverKuva(f.both);
    const size_t pixels = static_cast<size_t>(kFrameWidth) * kFrameHeight;
    f.blank.rgba.assign(pixels * 4, 0);
    f.blank.luma.assign(pixels, 0);
    return f;
  }();
  return frames;
}

// A clock the tests move by hand, for the detection interval.
double gFakeNowMs = 0.0;
double fakeClockMs() { return gFakeNowMs; }

/** config on the fake clock, which is reset to 0. */
NFTTrackingConfig withFakeClock(NFTTrackingConfig config) {
  gFakeNowMs = 0.0;
  config.clock = &fakeClockMs;
  return config;
}

/** Passes one frame, then detects and tracks on it, as the bindings' process() does. */
int feed(ARToolKitNFTCore &core, const Frame &frame) {
  core.setVideoFrame(frame.rgba.data(), frame.luma.data());
  return core.detectNFTMarker();
}

bool isTracking(const ARToolKitNFTCore &core, int index) {
  const NFTMarkerState *state = core.markerState(index);
  return state != nullptr && state->tracking;
}

/**
 * Single-thread: feeds the frame until done() holds, at most maxFrames times. The fake clock
 * advances 100 ms before each frame (nothing changes for a core on the default clock).
 */
template <typename Done>
bool feedUntil(ARToolKitNFTCore &core, const Frame &frame, Done done, int maxFrames = 60) {
  for (int i = 0; i < maxFrames; i++) {
    gFakeNowMs += 100.0;
    feed(core, frame);
    if (done()) return true;
  }
  return false;
}

}  // namespace

TEST(CoreFrameTest, DetectBeforeMarkersReturnsMinusOne) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  EXPECT_EQ(core.detectNFTMarker(), -1);

  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  EXPECT_EQ(feed(core, frames.both), -1);
}

TEST(CoreFrameTest, DetectBeforeAnyFrame) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));

  core.detectNFTMarker();
  ASSERT_NE(core.markerState(0), nullptr);
  EXPECT_FALSE(core.markerState(0)->tracking);
}

TEST(CoreFrameTest, DetectWithoutDetectorStillTracks) {
  ARToolKitNFTCore core(singleThreadPreset());
  const int cameraID = ARToolKitNFTCore::loadCamera("data/camera_para.dat");
  ASSERT_GE(cameraID, 0);
  const int id = core.setup(kFrameWidth, kFrameHeight, cameraID);
  ASSERT_EQ(core.setupAR2(), 0);
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));

  // A new camera and setupAR2() stop the detector until the next addNFTMarkers(), and the
  // markers stay loaded. Detection is skipped but tracking still runs: a blank frame loses pinball.
  ASSERT_EQ(core.setCamera(id, cameraID), 0);
  ASSERT_EQ(core.setupAR2(), 0);
  EXPECT_EQ(feed(core, frames.blank), -1);
  EXPECT_FALSE(isTracking(core, 0));
}

TEST(CoreFrameTest, DetectBetweenSetCameraAndSetupAR2) {
  ARToolKitNFTCore core(withFakeClock(singleThreadPreset()));
  const int cameraID = ARToolKitNFTCore::loadCamera("data/camera_para.dat");
  ASSERT_GE(cameraID, 0);
  const int id = core.setup(kFrameWidth, kFrameHeight, cameraID);
  ASSERT_EQ(core.setupAR2(), 0);
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));

  // setCamera() frees the detector and the KPM and AR2 handles with the old paramLT. Until
  // setupAR2() nothing is detected, and the marker being tracked is lost (no AR2 handle).
  ASSERT_EQ(core.setCamera(id, cameraID), 0);
  for (int i = 0; i < 3; i++) {
    gFakeNowMs += 100.0;
    EXPECT_EQ(feed(core, frames.both), -1) << "KPM ran on frame " << i;
    EXPECT_FALSE(isTracking(core, 0));
  }
  // Markers cannot be added without a KPM handle either.
  EXPECT_TRUE(core.addNFTMarkers({"data/kuva"}).empty());
  EXPECT_EQ(core.markerCount(), 1);

  // setupAR2() creates the handles; the next addNFTMarkers() hands KPM every loaded marker.
  ASSERT_EQ(core.setupAR2(), 0);
  gFakeNowMs += 100.0;
  EXPECT_EQ(feed(core, frames.both), -1);  // no detector before addNFTMarkers()
  EXPECT_EQ(core.addNFTMarkers({"data/kuva"}), std::vector<int>({1}));
  EXPECT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0) && isTracking(core, 1); }));
}

TEST(CoreFrameTest, SetupWithUnknownCameraDropsTheCamera) {
  ARToolKitNFTCore core(withFakeClock(singleThreadPreset()));
  const int cameraID = ARToolKitNFTCore::loadCamera("data/camera_para.dat");
  ASSERT_GE(cameraID, 0);
  ASSERT_GE(core.setup(kFrameWidth, kFrameHeight, cameraID), 0);
  ASSERT_EQ(core.setupAR2(), 0);
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));

  // A second setup() for a smaller frame resizes the frame buffers, but cannot apply an
  // unknown camera: paramLT and everything built on it, sized for the old frame, are freed
  // so that nothing reads the new buffers with the old size.
  const int width = kFrameWidth / 2;
  const int height = kFrameHeight / 2;
  const int id = core.setup(width, height, 9999);
  EXPECT_GE(id, 0);
  ASSERT_EQ(core.cameraParamLT(), nullptr);

  // Nothing is detected or tracked until a camera is applied and setupAR2() is called again.
  const size_t pixels = static_cast<size_t>(width) * height;
  const Frame small{std::vector<ARUint8>(pixels * 4, 0), std::vector<ARUint8>(pixels, 0)};
  gFakeNowMs += 100.0;
  EXPECT_EQ(feed(core, small), -1);
  EXPECT_FALSE(isTracking(core, 0));
  EXPECT_EQ(core.setupAR2(), -1);
  EXPECT_EQ(core.setCamera(id, cameraID), 0);
  EXPECT_EQ(core.setupAR2(), 0);
}

TEST(CoreFrameTest, ValueInitialisedConfigDetects) {
  ARToolKitNFTCore core(NFTTrackingConfig{});
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  EXPECT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));
}

TEST(CoreFrameTest, NullClockFallsBackToTheDefault) {
  NFTTrackingConfig config{};
  config.clock = nullptr;
  ARToolKitNFTCore core(config);
  ASSERT_TRUE(makeReady(core));
  // detectNFTMarker() reads the clock on every call, markers or not.
  EXPECT_EQ(core.detectNFTMarker(), -1);
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  EXPECT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));
}

TEST(CoreTrackingTest, SyncFindsBothMarkers) {
  // The fake clock moves 100 ms per frame, so a marker one KPM pass misses is searched for
  // again 300 ms later, however fast the machine runs the frames.
  ARToolKitNFTCore core(withFakeClock(singleThreadPreset()));
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());

  ASSERT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0) && isTracking(core, 1); }));
  // pose is [R|t], so [0][3] is the x translation: pinball is left of kuva.
  EXPECT_LT(core.markerState(0)->pose[0][3], core.markerState(1)->pose[0][3]);
}

TEST(CoreTrackingTest, BlankFrameLosesTracking) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));
  EXPECT_NE(core.markerState(0)->err, -1.0f);

  feed(core, frames.blank);
  EXPECT_FALSE(core.markerState(0)->tracking);
  EXPECT_EQ(core.markerState(0)->err, -1.0f);
}

TEST(CorePolicyTest, IntervalThrottlesWhileSomethingIsTracked) {
  ARToolKitNFTCore core(withFakeClock(singleThreadPreset()));
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));
  core.setDetectionInterval(300.0);
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.pinballOnly.rgba.empty());

  // KPM runs on every frame while nothing is tracked, so the pass that finds pinball is the
  // last one: it ends at the time of the frame where pinball is acquired, t0.
  ASSERT_TRUE(feedUntil(core, frames.pinballOnly, [&] { return isTracking(core, 0); }));
  ASSERT_FALSE(isTracking(core, 1));

  // Then one frame every 100 ms, at t0 + 100, t0 + 200, ..., t0 + 900. Kuva stays untracked
  // (it is painted over), so KPM is due exactly when 300 ms have passed since the previous
  // pass: at t0 + 300, t0 + 600 and t0 + 900, and on no frame in between. A frame where KPM
  // runs returns its result count (one per loaded page) instead of -1.
  const std::vector<bool> expected = {false, false, true, false, false, true, false, false, true};
  std::vector<bool> ran;
  for (size_t i = 0; i < expected.size(); i++) {
    gFakeNowMs += 100.0;
    ran.push_back(feed(core, frames.pinballOnly) != -1);
    ASSERT_TRUE(isTracking(core, 0)) << "pinball lost on frame " << i;
  }
  EXPECT_EQ(ran, expected);
}

TEST(CorePolicyTest, NaNIntervalMeansEveryFrame) {
  ARToolKitNFTCore core(withFakeClock(singleThreadPreset()));
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.pinballOnly.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.pinballOnly, [&] { return isTracking(core, 0); }));

  // NaN is taken as 0, like a negative value: with the clock stopped, KPM still runs on every
  // frame while kuva is untracked (an interval of NaN itself would never be due).
  core.setDetectionInterval(std::numeric_limits<double>::quiet_NaN());
  for (int i = 0; i < 3; i++) {
    EXPECT_NE(feed(core, frames.pinballOnly), -1) << "KPM did not run on frame " << i;
  }
}

TEST(CorePolicyTest, NaNDefaultIntervalMeansEveryFrame) {
  // The config's default interval is taken as setDetectionInterval() takes an interval.
  NFTTrackingConfig config = singleThreadPreset();
  config.defaultDetectionIntervalMs = std::numeric_limits<double>::quiet_NaN();
  ARToolKitNFTCore core(withFakeClock(config));
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.pinballOnly.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.pinballOnly, [&] { return isTracking(core, 0); }));

  for (int i = 0; i < 3; i++) {
    EXPECT_NE(feed(core, frames.pinballOnly), -1) << "KPM did not run on frame " << i;
  }
}

TEST(CorePolicyTest, ContinuousDetectionOff) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));
  // Every frame is due with an interval of 0: only continuous detection holds KPM back.
  core.setDetectionInterval(0.0);
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.pinballOnly.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.pinballOnly, [&] { return isTracking(core, 0); }));
  ASSERT_FALSE(isTracking(core, 1));

  core.setContinuousDetection(false);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(feed(core, frames.both), -1) << "KPM ran on frame " << i;
  }
  EXPECT_TRUE(isTracking(core, 0));
  EXPECT_FALSE(isTracking(core, 1));

  core.setContinuousDetection(true);
  EXPECT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 1); }));
}

TEST(CoreFilterTest, SingleThreadFilters) {
  ARToolKitNFTCore core(singleThreadPreset(), true);
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));
  EXPECT_NE(core.markerState(0)->ftmi, nullptr);
}

TEST(CoreFilterTest, SetFilteringTurnsItOnInTheSingleThreadPreset) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));
  EXPECT_EQ(core.markerState(0)->ftmi, nullptr);  // withFiltering defaults to false

  core.setFiltering(true);
  feed(core, frames.both);
  ASSERT_TRUE(isTracking(core, 0));
  EXPECT_NE(core.markerState(0)->ftmi, nullptr);
}

namespace {

/** The frame moved dx pixels to the right (edge pixels repeated); a second, different view. */
Frame shiftedRight(const Frame &src, int dx) {
  Frame out = src;
  for (int y = 0; y < kFrameHeight; y++) {
    for (int x = 0; x < kFrameWidth; x++) {
      const int sx = std::max(0, x - dx);
      const ARUint8 *from = &src.rgba[(static_cast<size_t>(y) * kFrameWidth + sx) * 4];
      std::copy(from, from + 4, &out.rgba[(static_cast<size_t>(y) * kFrameWidth + x) * 4]);
    }
  }
  computeLuma(out);
  return out;
}

double poseDistance(const ARdouble a[3][4], const ARdouble b[3][4]) {
  double sum = 0.0;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 4; c++) sum += std::fabs(a[r][c] - b[r][c]);
  }
  return sum;
}

}  // namespace

// Re-enabling filtering must restart each marker's filter (review of #106). The marker keeps
// its filter state while filtering is off, so without a reset the first filtered pose is pulled
// toward the sample from before it was switched off. A marker that moved while filtering was
// off is simulated by overwriting that stale sample with a position far from the real one: a
// reset leaves the pose on the raw one, a stale filter drags it most of the way to the sample.
// (Two real frames cannot tell them apart: AR2's pose jitters by more than the filter moves it.)
namespace {

/** Distance from the raw translation, and from the stale sample, of the pose after re-enabling. */
struct FilterRestart {
  bool ok = false;
  double fromRaw = 0.0;
  double staleOffset = 0.0;
};

template <typename Enable>
FilterRestart filterRestart(Enable enable) {
  FilterRestart out;
  const TestFrames &frames = testFrames();
  if (frames.both.rgba.empty()) return out;
  ARToolKitNFTCore core(singleThreadPreset());
  if (!makeReady(core)) return out;
  if (core.addNFTMarkers({"data/pinball"}) != std::vector<int>({0})) return out;
  if (!feedUntil(core, frames.both, [&] { return isTracking(core, 0); })) return out;

  core.setFiltering(true);
  for (int i = 0; i < 3; i++) feed(core, frames.both);
  core.setFiltering(false);
  feed(core, frames.both);  // unfiltered: markerState(0)->pose is the raw pose
  if (!isTracking(core, 0) || core.markerState(0)->ftmi == nullptr) return out;
  const ARdouble rawX = core.markerState(0)->pose[0][3];

  // Overwrite the stale sample through the public API: a reset call seeds it from the matrix.
  ARdouble far[3][4];
  std::copy(&core.markerState(0)->pose[0][0], &core.markerState(0)->pose[0][0] + 12, &far[0][0]);
  const ARdouble stale = rawX + 10000.0;
  far[0][3] = stale;
  if (arFilterTransMat(core.markerState(0)->ftmi, far, 1) < 0) return out;

  enable(core);
  feed(core, frames.both);
  if (!isTracking(core, 0)) return out;
  out.ok = true;
  out.fromRaw = std::fabs(core.markerState(0)->pose[0][3] - rawX);
  out.staleOffset = std::fabs(stale - rawX);
  return out;
}

template <typename Enable>
void expectFilterRestarts(Enable enable) {
  const FilterRestart r = filterRestart(enable);
  ASSERT_TRUE(r.ok);
  // Tracking jitter between two frames is a few units; the stale sample would pull ~85% of 10000.
  EXPECT_LT(r.fromRaw, r.staleOffset * 0.05);
}

}  // namespace

TEST(CoreFilterTest, SetFilteringRestartsTheFilter) {
  expectFilterRestarts([](ARToolKitNFTCore &core) { core.setFiltering(true); });
}

TEST(CoreFilterTest, ConfigureRestartsTheFilter) {
  expectFilterRestarts([](ARToolKitNFTCore &core) {
    NFTRuntimeSettings s = core.getRuntimeSettings();
    s.filtering = true;
    ASSERT_EQ(core.configure(s), 0);
  });
}

#ifdef WEBARKIT_NFT_THREADS

namespace {

/** Threaded: feeds the frame every 10 ms until done() holds or 10 s have gone by. */
template <typename Done>
bool feedUntilThreaded(ARToolKitNFTCore &core, const Frame &frame, Done done) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    feed(core, frame);
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace

TEST(CoreTrackingTest, ThreadedFindsBothMarkers) {
  ARToolKitNFTCore core(threadedPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball", "data/kuva"}), std::vector<int>({0, 1}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());

  ASSERT_TRUE(feedUntilThreaded(core, frames.both, [&] { return isTracking(core, 0) && isTracking(core, 1); }));
  EXPECT_LT(core.markerState(0)->pose[0][3], core.markerState(1)->pose[0][3]);
}

namespace {

/**
 * Tracks pinball on `base` until the pose settles, then feeds `moved` once and returns how far
 * the pose of marker 0 travelled. The filter low-passes this step; the raw pose follows it.
 * Returns a negative value when tracking failed.
 */
double poseStepOnAChangedFrame(ARToolKitNFTCore &core, const Frame &base, const Frame &moved) {
  if (!makeReady(core)) return -1.0;
  if (core.addNFTMarkers({"data/pinball"}) != std::vector<int>({0})) return -1.0;
  if (!feedUntilThreaded(core, base, [&] { return isTracking(core, 0); })) return -1.0;
  for (int i = 0; i < 5; i++) feed(core, base);
  if (!isTracking(core, 0)) return -1.0;
  ARdouble before[3][4];
  std::copy(&core.markerState(0)->pose[0][0], &core.markerState(0)->pose[0][0] + 12, &before[0][0]);
  feed(core, moved);
  if (!isTracking(core, 0)) return -1.0;
  return poseDistance(before, core.markerState(0)->pose);
}

}  // namespace

// #692: the threaded build never filtered poses; the core makes it a preset flag. Frames of a
// static image give the same raw pose, so filtered == raw proves nothing. The test feeds a
// second, shifted view and compares the pose step with a core that does not filter.
TEST(NFTRuntimeSettingsTest, ThreadedPresetFilters) {
  EXPECT_TRUE(threadedPreset().poseFilteringSupported);
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  const Frame moved = shiftedRight(frames.both, 6);

  ARToolKitNFTCore raw(threadedPreset());
  const double rawStep = poseStepOnAChangedFrame(raw, frames.both, moved);
  ASSERT_GT(rawStep, 0.0) << "the unfiltered core must follow the shifted view";
  EXPECT_EQ(raw.markerState(0)->ftmi, nullptr);

  ARToolKitNFTCore filtered(threadedPreset(), true);
  const double filteredStep = poseStepOnAChangedFrame(filtered, frames.both, moved);
  ASSERT_GE(filteredStep, 0.0);
  EXPECT_NE(filtered.markerState(0)->ftmi, nullptr);
  EXPECT_LT(filteredStep, rawStep * 0.95);
}

TEST(NFTRuntimeSettingsTest, ThreadedPresetFilteringFollowsSetFiltering) {
  ARToolKitNFTCore core(threadedPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntilThreaded(core, frames.both, [&] { return isTracking(core, 0); }));
  feed(core, frames.both);
  ASSERT_TRUE(isTracking(core, 0));
  EXPECT_EQ(core.markerState(0)->ftmi, nullptr);  // off by default

  core.setFiltering(true);
  feed(core, frames.both);
  ASSERT_TRUE(isTracking(core, 0));
  EXPECT_NE(core.markerState(0)->ftmi, nullptr);
}

TEST(CoreLifecycleTest, AddMarkersWhileSearching) {
  ARToolKitNFTCore core(threadedPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());

  // Nothing to collect yet: the frame starts a search on the worker.
  EXPECT_EQ(feed(core, frames.both), -1);
  // The search is awaited and its result dropped; the next frame searches both markers.
  EXPECT_EQ(core.addNFTMarkers({"data/kuva"}), std::vector<int>({1}));
  // Nothing to collect: the dropped search does not report. This frame starts the new one.
  EXPECT_EQ(feed(core, frames.both), -1);
  EXPECT_TRUE(feedUntilThreaded(core, frames.both, [&] { return isTracking(core, 0) && isTracking(core, 1); }));
}

TEST(CoreLifecycleTest, ThreadedSetCameraWhileSearching) {
  ARToolKitNFTCore core(threadedPreset());
  const int cameraID = ARToolKitNFTCore::loadCamera("data/camera_para.dat");
  ASSERT_GE(cameraID, 0);
  const int id = core.setup(kFrameWidth, kFrameHeight, cameraID);
  ASSERT_EQ(core.setupAR2(), 0);
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());

  EXPECT_EQ(feed(core, frames.both), -1);  // starts a search on the worker
  // Waits for the search and drops it, then frees the detector and the KPM handle with the
  // old paramLT: nothing is detected until setupAR2().
  ASSERT_EQ(core.setCamera(id, cameraID), 0);
  EXPECT_EQ(feed(core, frames.both), -1);
  EXPECT_FALSE(isTracking(core, 0));

  // The new KPM handle has no reference data: an empty batch hands it the loaded markers
  // (and returns an empty vector, as a failure does).
  ASSERT_EQ(core.setupAR2(), 0);
  EXPECT_TRUE(core.addNFTMarkers({}).empty());
  EXPECT_EQ(core.markerCount(), 1);
  EXPECT_TRUE(feedUntilThreaded(core, frames.both, [&] { return isTracking(core, 0); }));
}

TEST(CoreLifecycleTest, TeardownWhileSearching) {
  ARToolKitNFTCore core(threadedPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());

  EXPECT_EQ(feed(core, frames.both), -1);  // starts a search on the worker
  // Waits for the search before freeing the KPM handle it uses; the destructor follows.
  EXPECT_EQ(core.teardown(), 0);
  EXPECT_EQ(core.detectNFTMarker(), -1);
}

#endif  // WEBARKIT_NFT_THREADS

// Runtime settings (#720).

namespace {

std::vector<ARdouble> lensOf(const ARToolKitNFTCore &core) {
  return std::vector<ARdouble>(core.cameraLens(), core.cameraLens() + 16);
}

std::vector<ARdouble> frustumFor(const ARToolKitNFTCore &core, ARdouble nearPlane, ARdouble farPlane) {
  std::vector<ARdouble> lens(16);
  arglCameraFrustumRH(&core.cameraParamLT()->param, nearPlane, farPlane, lens.data());
  return lens;
}

bool sameSettings(const NFTRuntimeSettings &a, const NFTRuntimeSettings &b) {
  return a.nearPlane == b.nearPlane && a.farPlane == b.farPlane &&
         a.continuousDetection == b.continuousDetection &&
         a.detectionIntervalMs == b.detectionIntervalMs && a.filtering == b.filtering &&
         a.logLevel == b.logLevel && a.ar2.trackingThresh == b.ar2.trackingThresh &&
         a.ar2.simThresh == b.ar2.simThresh && a.ar2.searchFeatureNum == b.ar2.searchFeatureNum &&
         a.ar2.templateSize1 == b.ar2.templateSize1 && a.ar2.templateSize2 == b.ar2.templateSize2 &&
         a.ar2.searchSize == b.ar2.searchSize;
}

}  // namespace

TEST(NFTRuntimeSettingsTest, DefaultsMatchTheOldConstants) {
  const NFTRuntimeSettings s;
  EXPECT_EQ(s.nearPlane, 0.1);  // was 0.0001 (D4)
  EXPECT_EQ(s.farPlane, 1000.0);
  EXPECT_TRUE(s.continuousDetection);
  EXPECT_EQ(s.detectionIntervalMs, -1);
  EXPECT_FALSE(s.filtering);
  EXPECT_EQ(s.logLevel, -1);  // leave the level unchanged (D9)
  EXPECT_EQ(s.ar2.trackingThresh, 5.0f);
  EXPECT_EQ(s.ar2.simThresh, 0.5f);
  EXPECT_EQ(s.ar2.searchFeatureNum, 16);
  EXPECT_EQ(s.ar2.templateSize1, 6);
  EXPECT_EQ(s.ar2.templateSize2, 6);
  EXPECT_EQ(s.ar2.searchSize, 0);
  EXPECT_TRUE(nftRuntimeSettingsValid(s));
}

TEST(NFTRuntimeSettingsTest, GetResolvesSentinels) {
  ARToolKitNFTCore core(singleThreadPreset());
  EXPECT_EQ(core.getRuntimeSettings().detectionIntervalMs, 300.0);
  // configure() with the -1 sentinel resolves it to the preset's interval too.
  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.detectionIntervalMs = -1;
  ASSERT_EQ(core.configure(s), 0);
  EXPECT_EQ(core.getRuntimeSettings().detectionIntervalMs, 300.0);
}

TEST(NFTRuntimeSettingsTest, InvalidSettingsChangeNothing) {
  ARToolKitNFTCore core(singleThreadPreset());
  const NFTRuntimeSettings before = core.getRuntimeSettings();

  std::vector<NFTRuntimeSettings> bad;
  auto add = [&](void (*mutate)(NFTRuntimeSettings &)) {
    NFTRuntimeSettings s = before;
    // A valid change first, so a half-applied patch would be visible.
    s.nearPlane = 2.0;
    s.farPlane = 50.0;
    s.continuousDetection = !before.continuousDetection;
    mutate(s);
    bad.push_back(s);
  };
  add([](NFTRuntimeSettings &s) { s.nearPlane = 0; });
  add([](NFTRuntimeSettings &s) { s.farPlane = s.nearPlane; });
  add([](NFTRuntimeSettings &s) { s.farPlane = 1.0; });
  add([](NFTRuntimeSettings &s) { s.detectionIntervalMs = -2; });
  add([](NFTRuntimeSettings &s) { s.logLevel = 99; });
  add([](NFTRuntimeSettings &s) { s.logLevel = -2; });
  add([](NFTRuntimeSettings &s) { s.ar2.searchFeatureNum = 2; });
  add([](NFTRuntimeSettings &s) { s.ar2.searchFeatureNum = 41; });
  add([](NFTRuntimeSettings &s) { s.ar2.simThresh = 1.5f; });
  add([](NFTRuntimeSettings &s) { s.ar2.templateSize1 = 0; });
  add([](NFTRuntimeSettings &s) { s.ar2.searchSize = 65; });

  for (size_t i = 0; i < bad.size(); i++) {
    SCOPED_TRACE(i);
    EXPECT_EQ(core.configure(bad[i]), -1);
    EXPECT_TRUE(sameSettings(core.getRuntimeSettings(), before));
  }
}

TEST(NFTRuntimeSettingsTest, ConfigureRoundTrip) {
  ARToolKitNFTCore core(singleThreadPreset());
  const NFTRuntimeSettings before = core.getRuntimeSettings();
  EXPECT_EQ(core.configure(before), 0);
  EXPECT_TRUE(sameSettings(core.getRuntimeSettings(), before));

  NFTRuntimeSettings s = before;
  s.nearPlane = 1;
  s.farPlane = 500;
  ASSERT_EQ(core.configure(s), 0);
  EXPECT_TRUE(sameSettings(core.getRuntimeSettings(), s));
}

TEST(NFTRuntimeSettingsTest, PlaneSetterRebuildsLensAfterSetCamera) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  // The first lens already uses the default near plane (D4).
  EXPECT_EQ(lensOf(core), frustumFor(core, 0.1, 1000.0));

  core.setProjectionNearPlane(5.0);
  EXPECT_EQ(lensOf(core), frustumFor(core, 5.0, 1000.0));
  core.setProjectionFarPlane(200.0);
  EXPECT_EQ(lensOf(core), frustumFor(core, 5.0, 200.0));

  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.nearPlane = 2.0;
  s.farPlane = 300.0;
  ASSERT_EQ(core.configure(s), 0);
  EXPECT_EQ(lensOf(core), frustumFor(core, 2.0, 300.0));
}

TEST(NFTRuntimeSettingsTest, PlanesBeforeCameraOnlyStore) {
  ARToolKitNFTCore core(singleThreadPreset());
  core.setProjectionNearPlane(5.0);  // no paramLT yet: must not crash
  core.setProjectionFarPlane(400.0);
  EXPECT_EQ(core.getProjectionNearPlane(), 5.0);
  ASSERT_TRUE(makeReady(core));
  EXPECT_EQ(lensOf(core), frustumFor(core, 5.0, 400.0));
}

TEST(NFTRuntimeSettingsTest, LogLevelIsTheGlobal) {
  const int savedAr = arLogLevel;
  const int savedWebarkit = webarkitLogLevel;
  {
    ARToolKitNFTCore core(singleThreadPreset());
    NFTRuntimeSettings s = core.getRuntimeSettings();
    s.logLevel = AR_LOG_LEVEL_ERROR;
    ASSERT_EQ(core.configure(s), 0);
    EXPECT_EQ(arLogLevel, AR_LOG_LEVEL_ERROR);
    EXPECT_EQ(webarkitLogLevel, AR_LOG_LEVEL_ERROR);
    EXPECT_EQ(core.getRuntimeSettings().logLevel, AR_LOG_LEVEL_ERROR);

    // A second core does not reset the level (D9).
    ARToolKitNFTCore other(singleThreadPreset());
    EXPECT_EQ(arLogLevel, AR_LOG_LEVEL_ERROR);
    EXPECT_EQ(webarkitLogLevel, AR_LOG_LEVEL_ERROR);
    EXPECT_EQ(other.getRuntimeSettings().logLevel, AR_LOG_LEVEL_ERROR);
  }
  arLogLevel = savedAr;
  webarkitLogLevel = savedWebarkit;
}

namespace {

/** Restores the process-wide log levels, which these tests change on purpose. */
struct LogLevelGuard {
  int ar = arLogLevel;
  int webarkit = webarkitLogLevel;
  ~LogLevelGuard() {
    arLogLevel = ar;
    webarkitLogLevel = webarkit;
  }
};

}  // namespace

// logLevel -1 leaves the level alone (review of #106).

TEST(NFTRuntimeSettingsTest, LogLevelMinusOneIsValidAndMinusTwoIsNot) {
  NFTRuntimeSettings s;
  s.logLevel = -1;
  EXPECT_TRUE(nftRuntimeSettingsValid(s));
  s.logLevel = -2;
  EXPECT_FALSE(nftRuntimeSettingsValid(s));
  s.logLevel = AR_LOG_LEVEL_DEBUG;
  EXPECT_TRUE(nftRuntimeSettingsValid(s));
  s.logLevel = AR_LOG_LEVEL_REL_INFO;
  EXPECT_TRUE(nftRuntimeSettingsValid(s));
}

TEST(NFTRuntimeSettingsTest, FreshStructChangingThePlanesLeavesTheLevelAlone) {
  LogLevelGuard guard;
  ARToolKitNFTCore core(singleThreadPreset());
  arLogLevel = AR_LOG_LEVEL_ERROR;
  webarkitLogLevel = AR_LOG_LEVEL_ERROR;

  NFTRuntimeSettings fresh;  // logLevel is the default, -1
  fresh.nearPlane = 2.0;
  fresh.farPlane = 300.0;
  ASSERT_EQ(core.configure(fresh), 0);
  EXPECT_EQ(core.getProjectionNearPlane(), 2.0);
  EXPECT_EQ(arLogLevel, AR_LOG_LEVEL_ERROR);
  EXPECT_EQ(webarkitLogLevel, AR_LOG_LEVEL_ERROR);
  EXPECT_EQ(core.getRuntimeSettings().logLevel, AR_LOG_LEVEL_ERROR);  // the effective level, never -1
}

TEST(NFTRuntimeSettingsTest, RoundTripKeepsAnIndependentWebarkitLogLevel) {
  LogLevelGuard guard;
  ARToolKitNFTCore core(singleThreadPreset());
  arLogLevel = AR_LOG_LEVEL_INFO;
  webarkitLogLevel = AR_LOG_LEVEL_WARN;  // as WebARKitManager::setLogLevel() would leave it

  ASSERT_EQ(core.configure(core.getRuntimeSettings()), 0);
  EXPECT_EQ(arLogLevel, AR_LOG_LEVEL_INFO);
  EXPECT_EQ(webarkitLogLevel, AR_LOG_LEVEL_WARN);
}

TEST(NFTRuntimeSettingsTest, AnExplicitLevelIsWrittenToBothGlobals) {
  LogLevelGuard guard;
  ARToolKitNFTCore core(singleThreadPreset());
  arLogLevel = AR_LOG_LEVEL_INFO;
  webarkitLogLevel = AR_LOG_LEVEL_WARN;

  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.logLevel = AR_LOG_LEVEL_ERROR;
  ASSERT_EQ(core.configure(s), 0);
  EXPECT_EQ(arLogLevel, AR_LOG_LEVEL_ERROR);
  EXPECT_EQ(webarkitLogLevel, AR_LOG_LEVEL_ERROR);
}

// The AR2 parameters come from the settings (D6), are applied live, and a template-size
// change recreates only the AR2 handle (D12).

TEST(NFTRuntimeSettingsAR2Test, SetupAR2UsesStoredSettings) {
  ARToolKitNFTCore core(singleThreadPreset());
  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.ar2.trackingThresh = 7.0f;
  s.ar2.simThresh = 0.6f;
  s.ar2.searchFeatureNum = 20;
  ASSERT_EQ(core.configure(s), 0);  // no handle yet: only stored
  EXPECT_EQ(core.ar2HandleForTest(), nullptr);

  ASSERT_TRUE(makeReady(core));
  const AR2HandleT *handle = core.ar2HandleForTest();
  ASSERT_NE(handle, nullptr);
  EXPECT_FLOAT_EQ(handle->trackingThresh, 7.0f);
  EXPECT_FLOAT_EQ(handle->simThresh, 0.6f);
  EXPECT_EQ(handle->searchFeatureNum, 20);
  EXPECT_EQ(handle->templateSize1, 6);
  EXPECT_EQ(handle->templateSize2, 6);
}

TEST(NFTRuntimeSettingsAR2Test, ConfigureUpdatesLiveHandle) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  const AR2HandleT *handle = core.ar2HandleForTest();
  ASSERT_NE(handle, nullptr);

  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.ar2.trackingThresh = 9.0f;
  ASSERT_EQ(core.configure(s), 0);
  EXPECT_EQ(core.ar2HandleForTest(), handle);  // not recreated
  EXPECT_FLOAT_EQ(handle->trackingThresh, 9.0f);
}

TEST(NFTRuntimeSettingsAR2Test, TemplateSizeRecreatesOnlyAR2) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const AR2HandleT *oldHandle = core.ar2HandleForTest();
  const KpmHandle *kpm = core.kpmHandleForTest();
  ASSERT_NE(oldHandle, nullptr);
  ASSERT_NE(kpm, nullptr);

  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.ar2.templateSize1 = 8;
  ASSERT_EQ(core.configure(s), 0);
  // The new handle is created before the old one is freed, so the address differs.
  const AR2HandleT *newHandle = core.ar2HandleForTest();
  ASSERT_NE(newHandle, nullptr);
  EXPECT_NE(newHandle, oldHandle);
  EXPECT_EQ(newHandle->templateSize1, 8);
  EXPECT_EQ(newHandle->templateSize2, 6);
  EXPECT_FLOAT_EQ(newHandle->trackingThresh, 5.0f);  // the other settings are applied too
  EXPECT_EQ(core.kpmHandleForTest(), kpm);
  EXPECT_EQ(core.markerCount(), 1);
}

TEST(NFTRuntimeSettingsAR2Test, TrackingSurvivesTemplateChange) {
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));

  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.ar2.templateSize1 = 8;
  s.ar2.templateSize2 = 8;
  ASSERT_EQ(core.configure(s), 0);
  EXPECT_EQ(core.ar2HandleForTest()->templateSize1, 8);

  // The marker's tracking state is in its surface set, so it is tracked again on the same
  // image with the new handle within a few frames.
  EXPECT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }, 5));
  EXPECT_NE(core.markerState(0)->err, -1.0f);
}

TEST(NFTRuntimeSettingsAR2Test, TemplateChangeKeepsTrackingWithoutKpm) {
  // Stricter than TrackingSurvivesTemplateChange: the very next frame is tracked by the new
  // handle from the surface set's pose, with no KPM pass (feed() returns -1 when none ran;
  // KPM does not start while every marker is tracked).
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));

  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.ar2.templateSize1 = 8;
  s.ar2.templateSize2 = 8;
  ASSERT_EQ(core.configure(s), 0);
  EXPECT_EQ(feed(core, frames.both), -1);
  EXPECT_TRUE(isTracking(core, 0));
}

TEST(NFTRuntimeSettingsAR2Test, SearchSizeZeroFollowsPreset) {
  ARToolKitNFTCore core(singleThreadPreset());
  // The sentinel 0 is resolved through the preset: 6 without cpuDependentSearchSize.
  EXPECT_EQ(core.getRuntimeSettings().ar2.searchSize, 6);
  ASSERT_TRUE(makeReady(core));
  const AR2HandleT *handle = core.ar2HandleForTest();
  ASSERT_NE(handle, nullptr);
  EXPECT_EQ(handle->searchSize, 6);
  EXPECT_EQ(core.getRuntimeSettings().ar2.searchSize, 6);

  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.ar2.searchSize = 10;
  ASSERT_EQ(core.configure(s), 0);
  EXPECT_EQ(core.ar2HandleForTest(), handle);
  EXPECT_EQ(handle->searchSize, 10);
  EXPECT_EQ(core.getRuntimeSettings().ar2.searchSize, 10);
}

TEST(NFTRuntimeSettingsAR2Test, SetupAR2TwiceReplacesTheHandleAndKeepsTracking) {
  // setupAR2() is documented as callable again; the previous handle must be freed, not leaked.
  ARToolKitNFTCore core(singleThreadPreset());
  ASSERT_TRUE(makeReady(core));  // the first setupAR2()
  const AR2HandleT *first = core.ar2HandleForTest();
  ASSERT_NE(first, nullptr);

  ASSERT_EQ(core.setupAR2(), 0);
  ASSERT_NE(core.ar2HandleForTest(), nullptr);
  EXPECT_NE(core.ar2HandleForTest(), first);  // the new handle exists before the old is freed

  // setupAR2() drops the detector; addNFTMarkers() creates one for the new KPM handle.
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  EXPECT_TRUE(feedUntil(core, frames.both, [&] { return isTracking(core, 0); }));
}

#ifdef WEBARKIT_NFT_THREADS

TEST(NFTRuntimeSettingsAR2Test, ThreadedSetupAR2TwiceKeepsTracking) {
  ARToolKitNFTCore core(threadedPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.setupAR2(), 0);
  ASSERT_EQ(core.setupAR2(), 0);
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  EXPECT_TRUE(feedUntilThreaded(core, frames.both, [&] { return isTracking(core, 0); }));
}

TEST(NFTRuntimeSettingsAR2Test, ThreadedTemplateSizeRecreatesOnlyAR2) {
  // The threaded AR2 variant (ar2CreateHandle / ar2DeleteHandle, with tracking threads).
  ARToolKitNFTCore core(threadedPreset());
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const AR2HandleT *oldHandle = core.ar2HandleForTest();
  const KpmHandle *kpm = core.kpmHandleForTest();
  ASSERT_NE(oldHandle, nullptr);

  NFTRuntimeSettings s = core.getRuntimeSettings();
  s.ar2.templateSize2 = 9;
  ASSERT_EQ(core.configure(s), 0);
  const AR2HandleT *newHandle = core.ar2HandleForTest();
  ASSERT_NE(newHandle, nullptr);
  EXPECT_NE(newHandle, oldHandle);
  EXPECT_EQ(newHandle->templateSize2, 9);
  EXPECT_EQ(newHandle->searchSize, s.ar2.searchSize);  // 6 or 12, resolved by the preset
  EXPECT_EQ(core.kpmHandleForTest(), kpm);
}

#endif  // WEBARKIT_NFT_THREADS
