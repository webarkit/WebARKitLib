#include <gtest/gtest.h>

#include <AR/ar.h>
#include <AR2/imageFormat.h>
#include <KPM/kpm.h>
#include <WebARKitLog.h>
#include <WebARKitTrackers/WebARKitNFT/ARToolKitNFTCore.h>
#include <WebARKitTrackers/WebARKitNFT/NFTDetector.h>
#include <WebARKitTrackers/WebARKitNFT/NFTTrackingConfig.h>
#include <WebARKitTrackers/WebARKitNFT/SyncKpmDetector.h>
#ifdef WEBARKIT_NFT_THREADS
#include <WebARKitTrackers/WebARKitNFT/ThreadedKpmDetector.h>
#endif

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
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
  EXPECT_EQ(core.getProjectionNearPlane(), 0.0001);
  EXPECT_EQ(core.getProjectionFarPlane(), 1000.0);

  ASSERT_TRUE(makeReady(core));
  std::vector<ARdouble> before(core.cameraLens(), core.cameraLens() + 16);
  core.setProjectionNearPlane(1.0);
  core.setProjectionFarPlane(500.0);
  EXPECT_EQ(core.getProjectionNearPlane(), 1.0);
  EXPECT_EQ(core.getProjectionFarPlane(), 500.0);

  // The lens follows the planes only once recalculated.
  EXPECT_EQ(std::vector<ARdouble>(core.cameraLens(), core.cameraLens() + 16), before);
  core.recalculateCameraLens();
  EXPECT_NE(std::vector<ARdouble>(core.cameraLens(), core.cameraLens() + 16), before);
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
