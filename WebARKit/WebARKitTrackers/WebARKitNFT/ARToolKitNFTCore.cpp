#include <WebARKitLog.h>
#include <WebARKitTrackers/WebARKitNFT/ARToolKitNFTCore.h>
#include <WebARKitTrackers/WebARKitNFT/KpmRefDataSetCopy.h>
#include <WebARKitTrackers/WebARKitNFT/SyncKpmDetector.h>
#include <WebARKitTrackers/WebARKitNFT/markerDecompress.h>
#include <WebARKitTrackers/WebARKitNFT/trackingMod.h>
#ifdef WEBARKIT_NFT_THREADS
#include <WebARKitTrackers/WebARKitNFT/ThreadedKpmDetector.h>
#endif

#include <AR/paramGL.h>
#include <ARUtil/thread_sub.h>

#include <algorithm>
#include <unordered_map>
#include <utility>

// Process-wide, as in the bindings: a camera id from loadCamera() is valid for any controller.
static int gARControllerID = 0;
static int gCameraID = 0;
static std::unordered_map<int, ARParam> cameraParams;

ARToolKitNFTCore::ARToolKitNFTCore(const NFTTrackingConfig &config, bool withFiltering)
    : config(config),
      filterCutoffFrequency(60.0), filterSampleRate(120.0),
      id(0), param(), paramLT(nullptr), videoFrame(nullptr), videoFrameSize(0),
      videoLuma(nullptr), width(0), height(0),
      kpmHandle(nullptr), ar2Handle(nullptr), detector(nullptr),
      lastKpmEndMs(-std::numeric_limits<double>::infinity()),
      surfaceSetCount(0), // Running NFT marker id
      surfaceSet(), refDataSetAll(nullptr),
      cameraLens_(), pixFormat(AR_PIXEL_FORMAT_RGBA)
{
  // The settings start at their defaults (near 0.1: see NFTRuntimeSettings). The stored
  // interval is always the resolved one, so it starts as the preset's; as in
  // setDetectionInterval(), negative values and NaN mean 0.
  this->settings_.filtering = withFiltering;
  this->settings_.detectionIntervalMs =
      config.defaultDetectionIntervalMs > 0.0 ? config.defaultDetectionIntervalMs : 0.0;
  // The constructor does not touch arLogLevel / webarkitLogLevel (D9): they belong to the
  // process (the module, in JS), so constructing a core must not reset a level set earlier.
  // A config without a clock gets the default one, so detectNFTMarker() never calls null.
  if (this->config.clock == nullptr) {
    this->config.clock = &nftDefaultClockMs;
  }
  WEBARKIT_LOGi("init ARToolKitNFTCore...\n");
}

ARToolKitNFTCore::~ARToolKitNFTCore() {
  teardown();
}

/*********
 * Frames *
 *********/

void ARToolKitNFTCore::setVideoFrame(const ARUint8 *rgba, const ARUint8 *luma) {
  // Copy data instead of just assigning pointers
  if (this->videoFrame && rgba) {
    std::copy(rgba, rgba + this->videoFrameSize, this->videoFrame.get());
  }

  if (this->videoLuma && luma) {
    std::copy(luma, luma + (this->width * this->height), this->videoLuma.get());
  }
}

/*************************
 * Detection and tracking *
 *************************/

bool ARToolKitNFTCore::allMarkersTracked() const {
  for (int i = 0; i < this->surfaceSetCount; i++) {
    if (!markerStates[i].tracking) return false;
  }
  return true;
}

bool ARToolKitNFTCore::anyMarkerTracked() const {
  for (int i = 0; i < this->surfaceSetCount; i++) {
    if (markerStates[i].tracking) return true;
  }
  return false;
}

bool ARToolKitNFTCore::collectDetections(int &resultNum) {
  std::vector<NFTDetection> detections;
  int passResultNum = -1;
  if (!this->detector->collect(detections, passResultNum)) return false;
  // Finished, or failed (threaded, no detections): either way the pass is over.
  this->lastKpmEndMs = this->config.clock();
  resultNum = passResultNum;

  for (NFTDetection &detection : detections) {
    const int page = detection.page;
    if (page < 0 || page >= this->surfaceSetCount) {
      ARLOGe("KPM reported page %d, outside 0..%d.\n", page, this->surfaceSetCount - 1);
      continue;
    }
    NFTMarkerState &state = markerStates[page];
    if (state.tracking) continue;
    ar2SetInitTrans(this->surfaceSet[page], detection.trans);
    state.tracking = true;
    state.filterNeedsReset = true;
  }
  return true;
}

int ARToolKitNFTCore::detectNFTMarker() {
  int resultNum = -1;

  // Without a detector or a KPM handle (no addNFTMarkers() yet, or a setCamera() or
  // setupAR2() since the last one) nothing is detected this frame, but the markers being
  // tracked are still tracked.
  const bool canDetect = this->detector && this->kpmHandle;

  // Collect a pass that finished since the last frame (threaded: on the worker; a sync pass
  // is collected where it runs, below). Its markers then count as tracked when deciding
  // whether to start a new pass.
  if (canDetect) {
    collectDetections(resultNum);
  }

  // Detect every frame while nothing is tracked. Once something is, detect
  // at most once per detectionIntervalMs, counted from the END of the previous
  // pass, or not at all without continuous detection. A pass costs the full
  // KPM time on the frame where it runs; timing from its start would let a
  // pass slower than the interval run again on every frame.
  const double now = this->config.clock();
  const bool detectionDue =
      !anyMarkerTracked() ||
      (this->settings_.continuousDetection &&
       now - this->lastKpmEndMs >= this->settings_.detectionIntervalMs);

  if (canDetect && this->detector->idle() && this->surfaceSetCount > 0 &&
      !allMarkersTracked() && detectionDue) {

    // Pages already being tracked need no pose from KPM this pass.
    // kpmMatching() clears the skip flags again when it finishes.
    int skipPages[PAGES_MAX];
    int skipNum = 0;
    for (int i = 0; i < this->surfaceSetCount; i++) {
      if (markerStates[i].tracking) skipPages[skipNum++] = i;
    }

    // A sync pass has finished when start() returns true: apply it in this frame.
    if (this->detector->start(this->videoLuma.get(), skipPages, skipNum)) {
      collectDetections(resultNum);
    }
  }

  trackMarkers();
  return resultNum;
}

void ARToolKitNFTCore::trackMarkers() {
  for (int page = 0; page < this->surfaceSetCount; page++) {
    NFTMarkerState &state = markerStates[page];
    if (!state.tracking) continue;

    float trans[3][4];
    float err = -1.0f;
    int trackResult;
    if (this->config.ar2Variant == NFTTrackingConfig::AR2Variant::SingleThread) {
      trackResult = ar2TrackingMod(this->ar2Handle, this->surfaceSet[page],
                                   this->videoFrame.get(), trans, &err);
    } else {
      trackResult = ar2Tracking(this->ar2Handle, this->surfaceSet[page],
                                this->videoFrame.get(), trans, &err);
    }
    if (trackResult < 0) {
      ARLOGi("Tracking lost on page %d. %d\n", page, trackResult);
      state.tracking = false;
      state.err = -1.0f;
      continue;
    }

    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 4; c++) {
        state.pose[r][c] = trans[r][c];
      }
    }
    // Per-marker filter, reset on (re)acquisition. Both presets support it (#692: the threaded
    // build never filtered before the core); poseFilteringSupported is kept for a preset that
    // cannot filter.
    if (this->settings_.filtering && this->config.poseFilteringSupported) {
      if (!state.ftmi) {
        state.ftmi = arFilterTransMatInit(this->filterSampleRate, this->filterCutoffFrequency);
        state.filterNeedsReset = true;
      }
      if (arFilterTransMat(state.ftmi, state.pose, state.filterNeedsReset ? 1 : 0) < 0) {
        WEBARKIT_LOGe("arFilterTransMat error with marker %d.\n", page);
      }
      state.filterNeedsReset = false;
    }
    state.err = err;
    ARLOGi("Tracked page %d (max %d).\n", page, this->surfaceSetCount - 1);
  }
}

/*******************
 * KPM and AR2 setup *
 *******************/

void ARToolKitNFTCore::KpmHandleDeleter::operator()(KpmHandle *handle) const {
  if (handle) kpmDeleteHandle(&handle);
}

ARToolKitNFTCore::KpmHandlePtr ARToolKitNFTCore::createKpmHandle(ARParamLT *cparamLT) {
  KpmHandle *handle = kpmCreateHandle(cparamLT);
  if (!handle) {
    WEBARKIT_LOGe("Error: kpmCreateHandle returned NULL.\n");
    return KpmHandlePtr(nullptr);
  }
  return KpmHandlePtr(handle);
}

int ARToolKitNFTCore::setupAR2() {
  // Not in the bindings: without paramLT (no setup(), or a failed setCamera()) the AR2 and
  // KPM handles cannot be created, and ar2CreateHandle*() would dereference null.
  if (this->paramLT == nullptr) {
    WEBARKIT_LOGe("Error: setupAR2() needs a camera; call setup() first.\n");
    return -1;
  }

  AR2HandleT *tempHandle = createAR2Handle();
  if (tempHandle == nullptr) {
    WEBARKIT_LOGe("Error: ar2CreateHandle.\n");
    return -1;  // Return error code if handle creation failed
  }

  // A second setupAR2() replaces the handle: free the previous one only now that the new one
  // exists, so a failed creation leaves the old handle usable. Without this the old handle,
  // and on the threaded build its tracking threads, leaked.
  if (this->ar2Handle != nullptr) deleteAR2Handle();
  this->ar2Handle = tempHandle;

  // D6: the parameters come from the runtime settings (defaults: the constants this used to
  // hard-code), with searchSize 0 resolved to the preset's choice.
  if (this->settings_.ar2.searchSize == 0 && this->config.cpuDependentSearchSize) {
    if (threadGetCPU() <= 1) {
      WEBARKIT_LOGi("Using NFT tracking settings for a single CPU.\n");
    } else {
      WEBARKIT_LOGi("Using NFT tracking settings for more than one CPU.\n");
    }
  }
  applyAR2Settings(this->ar2Handle, effectiveAR2());

  // The detector searches with the KPM handle replaced below: drop a running search (as
  // addNFTMarkers() does, so the detection interval restarts), then stop the detector.
  // The next addNFTMarkers() creates one for the new handle.
  dropRunningSearch();
  this->detector.reset();

  // Create KPM handle
  this->kpmHandle = createKpmHandle(this->paramLT);
  if (!this->kpmHandle) {
    WEBARKIT_LOGe("Error creating KPM handle\n");
    return -1;
  }

  return 0;  // Success
}

AR2HandleT *ARToolKitNFTCore::createAR2Handle() {
  if (this->config.ar2Variant == NFTTrackingConfig::AR2Variant::SingleThread) {
    return ar2CreateHandleMod(this->paramLT, this->pixFormat);
  }
  return ar2CreateHandle(this->paramLT, this->pixFormat, AR2_TRACKING_DEFAULT_THREAD_NUM);
}

void ARToolKitNFTCore::applyAR2Settings(AR2HandleT *handle, const NFTAR2Settings &ar2) {
  ar2SetTrackingThresh(handle, ar2.trackingThresh);
  ar2SetSimThresh(handle, ar2.simThresh);
  ar2SetSearchFeatureNum(handle, ar2.searchFeatureNum);
  ar2SetSearchSize(handle, ar2.searchSize);
  ar2SetTemplateSize1(handle, ar2.templateSize1);
  ar2SetTemplateSize2(handle, ar2.templateSize2);
}

// D6: searchSize 0 means "the preset's choice": 6 (settings for devices with single-core
// CPUs), or 12 when config.cpuDependentSearchSize and more than one CPU, as setupAR2()
// hard-coded before. The sentinel exists because the threaded preset already varied the
// search size, and leaving it out of the settings would hard-code that choice again. It is
// stored unresolved, so every setupAR2() resolves it again.
NFTAR2Settings ARToolKitNFTCore::resolveAR2(const NFTAR2Settings &ar2) const {
  NFTAR2Settings resolved = ar2;
  if (resolved.searchSize == 0) {
    resolved.searchSize = (this->config.cpuDependentSearchSize && threadGetCPU() > 1) ? 12 : 6;
  }
  return resolved;
}

// D12: AR2 builds each tracking thread's template once, on the first tracked frame
// (lib/SRC/AR2/tracking2d.c:106, WebARKitTrackers/WebARKitNFT/trackingMod2d.c:81: `if (*templ == NULL) *templ =
// ar2GenTemplate(...)`), and keeps it in the handle. A later ar2SetTemplateSize1/2 only stores
// the number, with no effect. So a template-size change needs a new handle. Only the AR2
// handle is replaced: the per-marker tracking state (the last poses trans1..3, contNum,
// prevFeature) lives in each marker's AR2SurfaceSetT, and what the handle holds besides the
// templates (wtrans, candidates, usedFeature, pos) is per-frame scratch recomputed from the
// surface set on every ar2Tracking*() call. So the markers stay loaded and keep tracking. The
// KPM handle and the detector do not use the AR2 handle and are not touched. It is the only
// option that makes the change take effect live without reaching into AR2's internals.
int ARToolKitNFTCore::recreateAR2Handle() {
  if (this->ar2Handle == nullptr || this->paramLT == nullptr) return -1;
  AR2HandleT *newHandle = createAR2Handle();
  if (newHandle == nullptr) {
    WEBARKIT_LOGe("Error: ar2CreateHandle; the AR2 handle is unchanged.\n");
    return -1;
  }
  applyAR2Settings(newHandle, effectiveAR2());
  deleteAR2Handle();  // the delete call matching the variant
  this->ar2Handle = newHandle;
  return 0;
}

std::unique_ptr<NFTDetector> ARToolKitNFTCore::createDetector() {
  if (this->config.detector == NFTTrackingConfig::Detector::Sync) {
    return std::unique_ptr<NFTDetector>(new SyncKpmDetector(this->kpmHandle.get()));
  }
#ifdef WEBARKIT_NFT_THREADS
  std::unique_ptr<ThreadedKpmDetector> threaded = ThreadedKpmDetector::create(this->kpmHandle.get());
  if (!threaded) {
    WEBARKIT_LOGe("Error: could not start the detection worker.\n");
    return nullptr;
  }
  return std::unique_ptr<NFTDetector>(std::move(threaded));
#else
  WEBARKIT_LOGe("Error: threaded detection needs a build with WEBARKIT_NFT_THREADS.\n");
  return nullptr;
#endif
}

void ARToolKitNFTCore::dropRunningSearch() {
  // Its result is for the old KPM state, so it is dropped, and the next frame starts a
  // search over the new one.
  if (this->detector && !this->detector->idle()) {
    this->detector->waitIdle();
    this->lastKpmEndMs = this->config.clock();
  }
}

nftMarker ARToolKitNFTCore::getNFTData(int index) const {
  // get marker(s) nft data.
  return this->nftMarkers.at(index);
}

const NFTMarkerState *ARToolKitNFTCore::markerState(int index) const {
  if (index < 0 || index >= this->surfaceSetCount) return nullptr;
  return &this->markerStates[index];
}

/***********
 * Teardown *
 ***********/

void ARToolKitNFTCore::deleteAR2Handle() {
  if (this->ar2Handle != nullptr) {
    if (this->config.ar2Variant == NFTTrackingConfig::AR2Variant::SingleThread) {
      ar2DeleteHandleMod(&(this->ar2Handle));
    } else {
      ar2DeleteHandle(&(this->ar2Handle));
    }
    this->ar2Handle = nullptr;
  }
}

void ARToolKitNFTCore::deleteHandle() {
  if (this->paramLT != nullptr) {
    arParamLTFree(&(this->paramLT));
    this->paramLT = nullptr;
  }
  deleteAR2Handle();
}

void ARToolKitNFTCore::deleteCamera() {
  // A running KPM search reads paramLT through the KPM handle: let it end before freeing.
  // Not in the bindings: the detector and the KPM handle go too, as both point at paramLT;
  // until setupAR2() creates new ones nothing is detected.
  dropRunningSearch();
  this->detector.reset();
  this->kpmHandle.reset();
  deleteHandle();
}

int ARToolKitNFTCore::teardown() {
  // Stop the detector first: a threaded search uses the KPM handle (and, through it,
  // paramLT) until it ends. Its destructor waits for the search, then lets the worker exit.
  this->detector.reset();

  this->kpmHandle.reset();

  deleteAR2Handle();

  for (int i = 0; i < this->surfaceSetCount; i++) {
    ar2FreeSurfaceSet(&this->surfaceSet[i]);
  }
  this->surfaceSetCount = 0;
  // nftMarkers is kept, as in the bindings: getNFTData() still answers after teardown().

  if (this->paramLT != nullptr) {
    arParamLTFree(&(this->paramLT));
    this->paramLT = nullptr;
  }

  if (this->refDataSetAll) {
    kpmDeleteRefDataSet(&this->refDataSetAll);
  }

  for (auto &state : this->markerStates) {
    if (state.ftmi) {
      arFilterTransMatFinal(state.ftmi);
    }
    state = NFTMarkerState{};
  }

  this->videoFrame.reset();
  this->videoLuma.reset();
  this->videoFrameSize = 0;

  return 0;
}

/*********
 * Camera *
 *********/

// id is unused, as in the bindings: it keeps their setCamera(id, cameraID) signature.
int ARToolKitNFTCore::setCamera(int /*id*/, int cameraID) {

  if (cameraParams.find(cameraID) == cameraParams.end()) {
    return -1;
  }

  this->param = cameraParams[cameraID];

  if (this->param.xsize != this->width || this->param.ysize != this->height) {
    ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", this->param.xsize,
           this->param.ysize);
    arParamChangeSize(&(this->param), this->width, this->height,
                      &(this->param));
  }

  ARLOGi("*** Camera Parameter ***\n");
  arParamDisp(&(this->param));

  deleteCamera();

  this->paramLT = arParamLTCreate(&(this->param), AR_PARAM_LT_DEFAULT_OFFSET);
  if (!this->paramLT) {
    WEBARKIT_LOGe("setCamera(): Error: arParamLTCreate for cameraID %d.\n", cameraID);
    return -1;
  }

  ARLOGi("setCamera(): arParamLTCreated\n..%d, %d\n", (this->paramLT->param).xsize, (this->paramLT->param).ysize);

  arglCameraFrustumRH(&((this->paramLT)->param), this->settings_.nearPlane,
                      this->settings_.farPlane, this->cameraLens_);

  return 0;
}

void ARToolKitNFTCore::recalculateCameraLens() {
  // Not in the bindings: there is no lens without paramLT.
  if (this->paramLT == nullptr) return;
  arglCameraFrustumRH(&((this->paramLT)->param), this->settings_.nearPlane,
                      this->settings_.farPlane, this->cameraLens_);
}

int ARToolKitNFTCore::loadCamera(const std::string &cparam_name) {
  ARParam param;
  if (arParamLoad(cparam_name.c_str(), 1, &param) < 0) {
    WEBARKIT_LOGe("loadCamera(): Error loading parameter file %s for camera.\n",
                  cparam_name.c_str());
    return -1;
  }
  int cameraID = gCameraID++;
  cameraParams[cameraID] = param;

  return cameraID;
}

int ARToolKitNFTCore::decompressZFT(const std::string &datasetPathname, const std::string &tempPathname) {
  int response = decompressMarkers(datasetPathname.c_str(), tempPathname.c_str());

  // 1 on success, -1 if the archive is missing or malformed.
  return response == 0 ? 1 : -1;
}

/*****************
 * Marker loading *
 *****************/

std::vector<int>
ARToolKitNFTCore::addNFTMarkers(const std::vector<std::string> &datasetPathnames) {

  // Per-marker state lives in fixed arrays of PAGES_MAX entries (surfaceSet,
  // markerStates) indexed up to surfaceSetCount, so the running total across
  // every call must stay within PAGES_MAX. Refuse before any state changes.
  if (datasetPathnames.size() >
      static_cast<size_t>(PAGES_MAX - this->surfaceSetCount)) {
    WEBARKIT_LOGe("Error: exceeded maximum pages (%d).\n", PAGES_MAX);
    return {};
  }

  // No KPM handle before setupAR2(), or after a setCamera() until the next setupAR2(). The
  // bindings fail here too (kpmSetRefDataSet() refuses a null handle); refuse before a
  // detector is created over it.
  if (!this->kpmHandle) {
    WEBARKIT_LOGe("Error: addNFTMarkers() needs a KPM handle; call setupAR2() first.\n");
    return {};
  }

  // One detector for the KPM handle's lifetime, created by the first call.
  // teardown(), setCamera() and a new KPM handle from setupAR2() stop it.
  if (!this->detector) {
    this->detector = createDetector();
    if (!this->detector) {
      return {};
    }
  }

  // Markers loaded by earlier calls keep their ids, so this batch continues
  // the sequence: marker id = KPM page number = surfaceSet slot.
  const int firstId = this->surfaceSetCount;
  const int batchSize = static_cast<int>(datasetPathnames.size());

  // Load the whole batch before changing any state, so a marker that fails to
  // load leaves the markers from earlier calls untouched.
  KpmRefDataSet *batchRefDataSet = nullptr;
  auto discardBatch = [&](int loadedSurfaces) {
    for (int j = 0; j < loadedSurfaces; j++) {
      ar2FreeSurfaceSet(&this->surfaceSet[firstId + j]);
    }
    if (batchRefDataSet) {
      kpmDeleteRefDataSet(&batchRefDataSet);
    }
  };

  for (int i = 0; i < batchSize; i++) {
    const char *datasetPathname = datasetPathnames[i].c_str();
    const int id = firstId + i;
    WEBARKIT_LOGi("add NFT marker-> '%s'\n", datasetPathname);

    // Load KPM data.
    KpmRefDataSet *refDataSet2;
    WEBARKIT_LOGi("Reading %s.fset3\n", datasetPathname);
    if (kpmLoadRefDataSet(datasetPathname, "fset3", &refDataSet2) < 0) {
      WEBARKIT_LOGe("Error reading KPM data from %s.fset3\n", datasetPathname);
      discardBatch(i);
      return {};
    }
    WEBARKIT_LOGi("Assigned page no. %d.\n", id);
    if (kpmChangePageNoOfRefDataSet(refDataSet2, KpmChangePageNoAllPages, id) < 0) {
      WEBARKIT_LOGe("Error: kpmChangePageNoOfRefDataSet\n");
      kpmDeleteRefDataSet(&refDataSet2);
      discardBatch(i);
      return {};
    }
    if (kpmMergeRefDataSet(&batchRefDataSet, &refDataSet2) < 0) {
      WEBARKIT_LOGe("Error: kpmMergeRefDataSet\n");
      discardBatch(i);
      return {};
    }

    // Load AR2 data.
    WEBARKIT_LOGi("Reading %s.fset\n", datasetPathname);
    if ((this->surfaceSet[id] = ar2ReadSurfaceSet(datasetPathname, "fset", nullptr)) == nullptr) {
      WEBARKIT_LOGe("Error reading data from %s.fset\n", datasetPathname);
      discardBatch(i);
      return {};
    }
  }

  // Hand KPM every marker loaded so far: kpmSetRefDataSet() rebuilds the
  // matcher from the set it is given, so the new batch alone would drop the
  // markers from earlier calls. The batch is merged into a copy of the
  // accumulated set, which replaces it only once KPM accepts it; a rejected
  // batch (kpmSetRefDataSet() checks its image limit before changing
  // anything) leaves the earlier markers loaded and detectable.
  KpmRefDataSet *combined = nullptr;
  if (this->refDataSetAll &&
      (combined = kpmCopyRefDataSet(this->refDataSetAll)) == nullptr) {
    WEBARKIT_LOGe("Error: out of memory copying the KPM reference data.\n");
    discardBatch(batchSize);
    return {};
  }
  if (kpmMergeRefDataSet(&combined, &batchRefDataSet) < 0) {
    WEBARKIT_LOGe("Error: kpmMergeRefDataSet\n");
    kpmDeleteRefDataSet(&combined);
    discardBatch(batchSize);
    return {};
  }

  // kpmSetRefDataSet() replaces the matcher a threaded search runs with. Wait for
  // a running search to finish first; its result is for the old marker set,
  // so it is dropped, and the next frame starts a search over the new one.
  dropRunningSearch();

  if (kpmSetRefDataSet(this->kpmHandle.get(), combined) < 0) {
    WEBARKIT_LOGe("Error: kpmSetRefDataSet\n");
    kpmDeleteRefDataSet(&combined);
    discardBatch(batchSize);
    return {};
  }
  kpmDeleteRefDataSet(&this->refDataSetAll);
  this->refDataSetAll = combined;

  // nftMarkers[id] must hold marker id. teardown() keeps the old entries (getNFTData() still
  // answers after it) but restarts the ids at 0, so drop the entries the new ids replace.
  // Only ever shrinks: nftMarkers has at least surfaceSetCount (= firstId) entries.
  this->nftMarkers.resize(firstId);

  std::vector<int> markerIds;
  for (int i = 0; i < batchSize; i++) {
    const int id = firstId + i;
    AR2SurfaceSetT *surface = this->surfaceSet[id];
    nftMarker nft;
    nft.id_NFT = id;
    nft.width_NFT = surface->surface[0].imageSet->scale[0]->xsize;
    nft.height_NFT = surface->surface[0].imageSet->scale[0]->ysize;
    nft.dpi_NFT = surface->surface[0].imageSet->scale[0]->dpi;
    ARLOGi("NFT marker %d: %d x %d at %d dpi.\n", id, nft.width_NFT,
           nft.height_NFT, nft.dpi_NFT);
    this->nftMarkers.push_back(nft);
    this->markerStates[id] = NFTMarkerState{};
    markerIds.push_back(id);
  }

  this->surfaceSetCount += batchSize;
  WEBARKIT_LOGi("Loading of NFT data complete.\n");

  return markerIds;
}

/**********************
 * Setters and getters *
 **********************/

// D5: the plane setters rebuild the lens when a camera is loaded, so cameraLens() is always
// consistent with the planes and callers need no recalculateCameraLens(). Before setCamera()
// there is no lens yet, so the value is only stored (recalculateCameraLens() does nothing
// without paramLT). The setters stay lenient: unlike configure() they validate nothing.
void ARToolKitNFTCore::setProjectionNearPlane(const ARdouble projectionNearPlane) {
  this->settings_.nearPlane = projectionNearPlane;
  recalculateCameraLens();
}

ARdouble ARToolKitNFTCore::getProjectionNearPlane() const { return this->settings_.nearPlane; }

void ARToolKitNFTCore::setProjectionFarPlane(const ARdouble projectionFarPlane) {
  this->settings_.farPlane = projectionFarPlane;
  recalculateCameraLens();
}

ARdouble ARToolKitNFTCore::getProjectionFarPlane() const { return this->settings_.farPlane; }

/********
 * Setup *
 ********/

int ARToolKitNFTCore::setup(int width, int height, int cameraID) {
  // Not in the bindings: the frame sizes below are ints, so reject a frame they cannot hold
  // before changing anything. (max) in parentheses: on Windows <AR/config.h> includes
  // <windows.h>, whose max() macro would otherwise break this line (MSVC C2589) unless
  // NOMINMAX is defined (#101).
  if (width <= 0 || height <= 0 ||
      static_cast<long long>(width) * height * 4 > (std::numeric_limits<int>::max)()) {
    WEBARKIT_LOGe("setup(): Error: invalid frame size %d x %d.\n", width, height);
    return -1;
  }

  int id = gARControllerID++;
  this->id = id;

  this->width = width;
  this->height = height;

  this->videoFrameSize = width * height * 4 * sizeof(ARUint8);
  // unique_ptr owns the frame buffers: exclusive ownership and automatic deallocation.
  // Not in the bindings: the buffers start zeroed (a blank frame), so a detectNFTMarker()
  // before the first setVideoFrame() does not search leftover heap memory, which can hold
  // an earlier frame.
  this->videoFrame = std::unique_ptr<ARUint8[]>(new ARUint8[this->videoFrameSize]());
  this->videoLuma = std::unique_ptr<ARUint8[]>(new ARUint8[this->width * this->height]());

  if (setCamera(id, cameraID) != 0) {
    // Not in the bindings: paramLT and the handles built on it are still sized for the
    // previous frame, if any, and would read past the new buffers. Free them: nothing is
    // detected or tracked until setCamera() and setupAR2().
    deleteCamera();
  }

  WEBARKIT_LOGi("Allocated videoFrameSize %d\n", this->videoFrameSize);

  return this->id;
}

/*******************
 * Detection policy *
 *******************/

// A marker keeps its filter state (ftmi) while filtering is off. When filtering comes back on
// during continuous tracking, the first filtered pose would be pulled toward that stale
// sample. Marking every marker for a reset makes the next tracked frame re-seed the filter
// from the raw pose. One place, used by setFiltering() and configure() (review of #106).
void ARToolKitNFTCore::resetMarkerFilters() {
  for (NFTMarkerState &state : this->markerStates) state.filterNeedsReset = true;
}

void ARToolKitNFTCore::setFiltering(bool enableFiltering) {
  if (enableFiltering && !this->settings_.filtering) resetMarkerFilters();
  this->settings_.filtering = enableFiltering;
  WEBARKIT_LOGi("Filtering enabled with setFiltering: %s\n", enableFiltering ? "true" : "false");
}

void ARToolKitNFTCore::setContinuousDetection(bool enabled) {
  this->settings_.continuousDetection = enabled;
  WEBARKIT_LOGi("Continuous detection: %s\n", enabled ? "on" : "off");
}

void ARToolKitNFTCore::setDetectionInterval(double ms) {
  // Negative (or NaN) means "every frame", as 0 does.
  this->settings_.detectionIntervalMs = ms > 0.0 ? ms : 0.0;
  WEBARKIT_LOGi("Detection interval: %f ms\n", this->settings_.detectionIntervalMs);
}

/*******************
 * Runtime settings *
 *******************/

// D7: configure() validates everything before it changes anything, so a patch that is invalid
// in one field applies none of it. The setters above stay lenient (for example a negative
// interval means 0) so existing callers do not break; only the new entry point rejects.
//
// Threading: on the threaded build, configure() is called between frames on the thread that
// calls detectNFTMarker(). AR2 tracking runs inside that call (its tracking threads only work
// within ar2Tracking()), so changing or replacing the AR2 handle between frames is safe. The
// threaded KPM search, which may still be running on the detector's worker, does not use the
// AR2 handle.
int ARToolKitNFTCore::configure(const NFTRuntimeSettings &settings) {
  if (!nftRuntimeSettingsValid(settings)) {
    WEBARKIT_LOGe("configure(): invalid settings, nothing changed.\n");
    return -1;
  }

  const bool planesChanged = settings.nearPlane != this->settings_.nearPlane ||
                             settings.farPlane != this->settings_.farPlane;

  // The interval -1 follows the preset; stored resolved, as setDetectionInterval() does.
  const double interval = settings.detectionIntervalMs < 0 ? this->config.defaultDetectionIntervalMs
                                                           : settings.detectionIntervalMs;

  const NFTRuntimeSettings previous = this->settings_;
  this->settings_ = settings;
  this->settings_.detectionIntervalMs = previous.detectionIntervalMs;  // set below

  // D6 / D12: the AR2 fields go to the live handle. Done first, because recreating the handle
  // can fail, and then nothing may have changed (D7). Without a handle (before setupAR2(), or
  // after a setCamera()) they are only stored, and the next setupAR2() applies them.
  if (this->ar2Handle != nullptr) {
    const NFTAR2Settings before = resolveAR2(previous.ar2);
    const NFTAR2Settings after = effectiveAR2();
    if (after.templateSize1 != before.templateSize1 || after.templateSize2 != before.templateSize2) {
      // The templates are built once per handle (D12): a new handle, with every AR2 field.
      if (recreateAR2Handle() != 0) {
        this->settings_ = previous;
        WEBARKIT_LOGe("configure(): could not recreate the AR2 handle, nothing changed.\n");
        return -1;
      }
    } else {
      if (after.trackingThresh != before.trackingThresh) ar2SetTrackingThresh(this->ar2Handle, after.trackingThresh);
      if (after.simThresh != before.simThresh) ar2SetSimThresh(this->ar2Handle, after.simThresh);
      if (after.searchFeatureNum != before.searchFeatureNum) ar2SetSearchFeatureNum(this->ar2Handle, after.searchFeatureNum);
      if (after.searchSize != before.searchSize) ar2SetSearchSize(this->ar2Handle, after.searchSize);
    }
  }

  if (settings.filtering && !previous.filtering) resetMarkerFilters();  // off to on: see above

  setDetectionInterval(interval);

  if (planesChanged) recalculateCameraLens();  // a no-op before setCamera()

  // D9: the level is process-wide (module-wide in JS), not held by the core.
  arLogLevel = settings.logLevel;
  webarkitLogLevel = settings.logLevel;
  return 0;
}

NFTRuntimeSettings ARToolKitNFTCore::getRuntimeSettings() const {
  NFTRuntimeSettings settings = this->settings_;
  settings.ar2 = effectiveAR2();  // searchSize 0 resolved (D6, D7)
  settings.logLevel = arLogLevel;
  return settings;
}
