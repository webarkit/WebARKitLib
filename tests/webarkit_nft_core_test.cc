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
#include <limits>
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

TEST(CoreFilterTest, ThreadedPresetNeverFilters) {
  ARToolKitNFTCore core(threadedPreset());
  core.setFiltering(true);
  ASSERT_TRUE(makeReady(core));
  ASSERT_EQ(core.addNFTMarkers({"data/pinball"}), std::vector<int>({0}));
  const TestFrames &frames = testFrames();
  ASSERT_FALSE(frames.both.rgba.empty());
  ASSERT_TRUE(feedUntilThreaded(core, frames.both, [&] { return isTracking(core, 0); }));
  feed(core, frames.both);
  ASSERT_TRUE(isTracking(core, 0));
  EXPECT_EQ(core.markerState(0)->ftmi, nullptr);
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
