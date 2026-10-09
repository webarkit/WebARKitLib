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

#include <unordered_map>
#include <utility>

// Process-wide, as in the bindings: a camera id from loadCamera() is valid for any controller.
static int gARControllerID = 0;
static int gCameraID = 0;
static std::unordered_map<int, ARParam> cameraParams;

ARToolKitNFTCore::ARToolKitNFTCore(const NFTTrackingConfig &config, bool withFiltering)
    : config(config), withFiltering(withFiltering),
      filterCutoffFrequency(60.0), filterSampleRate(120.0),
      id(0), param(), paramLT(nullptr), videoFrame(nullptr), videoFrameSize(0),
      videoLuma(nullptr), width(0), height(0),
      kpmHandle(nullptr), ar2Handle(nullptr), detector(nullptr),
      continuousDetection(true),
      detectionIntervalMs(config.defaultDetectionIntervalMs),
      lastKpmEndMs(-std::numeric_limits<double>::infinity()),
      surfaceSetCount(0), // Running NFT marker id
      surfaceSet(), refDataSetAll(nullptr),
      nearPlane(0.0001), farPlane(1000.0),
      cameraLens_(), pixFormat(AR_PIXEL_FORMAT_RGBA)
{
  WEBARKIT_LOGi("init ARToolKitNFT constructor...\n");
}

ARToolKitNFTCore::~ARToolKitNFTCore() {
  teardown();
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

  AR2HandleT *tempHandle;
  if (this->config.ar2Variant == NFTTrackingConfig::AR2Variant::SingleThread) {
    tempHandle = ar2CreateHandleMod(this->paramLT, this->pixFormat);
  } else {
    tempHandle = ar2CreateHandle(this->paramLT, this->pixFormat, AR2_TRACKING_DEFAULT_THREAD_NUM);
  }
  if (tempHandle == nullptr) {
    WEBARKIT_LOGe("Error: ar2CreateHandle.\n");
    return -1;  // Return error code if handle creation failed
  }

  // Store the handle
  this->ar2Handle = tempHandle;

  // Settings for devices with single-core CPUs; the threaded build searches a larger
  // area when it has more than one CPU.
  int searchSize = 6;
  if (this->config.cpuDependentSearchSize) {
    if (threadGetCPU() <= 1) {
      WEBARKIT_LOGi("Using NFT tracking settings for a single CPU.\n");
    } else {
      WEBARKIT_LOGi("Using NFT tracking settings for more than one CPU.\n");
      searchSize = 12;
    }
  }
  ar2SetTrackingThresh(this->ar2Handle, 5.0);
  ar2SetSimThresh(this->ar2Handle, 0.50);
  ar2SetSearchFeatureNum(this->ar2Handle, 16);
  ar2SetSearchSize(this->ar2Handle, searchSize);
  ar2SetTemplateSize1(this->ar2Handle, 6);
  ar2SetTemplateSize2(this->ar2Handle, 6);

  // The detector searches with the KPM handle replaced below: stop it first (its destructor
  // waits for a running search). The next addNFTMarkers() creates one for the new handle.
  this->detector.reset();

  // Create KPM handle
  this->kpmHandle = createKpmHandle(this->paramLT);
  if (!this->kpmHandle) {
    WEBARKIT_LOGe("Error creating KPM handle\n");
    return -1;
  }

  return 0;  // Success
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
  this->nftMarkers.clear();

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

  // A running KPM search reads paramLT through the KPM handle: let it end before freeing.
  dropRunningSearch();
  deleteHandle();

  this->paramLT = arParamLTCreate(&(this->param), AR_PARAM_LT_DEFAULT_OFFSET);
  if (!this->paramLT) {
    WEBARKIT_LOGe("setCamera(): Error: arParamLTCreate for cameraID %d.\n", cameraID);
    return -1;
  }

  ARLOGi("setCamera(): arParamLTCreated\n..%d, %d\n", (this->paramLT->param).xsize, (this->paramLT->param).ysize);

  arglCameraFrustumRH(&((this->paramLT)->param), this->nearPlane,
                      this->farPlane, this->cameraLens_);

  return 0;
}

void ARToolKitNFTCore::recalculateCameraLens() {
  // Not in the bindings: there is no lens without paramLT.
  if (this->paramLT == nullptr) return;
  arglCameraFrustumRH(&((this->paramLT)->param), this->nearPlane,
                      this->farPlane, this->cameraLens_);
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

  // One detector for the KPM handle's lifetime, created by the first call.
  // teardown() (and a new KPM handle from setupAR2()) stops it.
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

void ARToolKitNFTCore::setProjectionNearPlane(const ARdouble projectionNearPlane) {
  this->nearPlane = projectionNearPlane;
}

ARdouble ARToolKitNFTCore::getProjectionNearPlane() const { return this->nearPlane; }

void ARToolKitNFTCore::setProjectionFarPlane(const ARdouble projectionFarPlane) {
  this->farPlane = projectionFarPlane;
}

ARdouble ARToolKitNFTCore::getProjectionFarPlane() const { return this->farPlane; }

/********
 * Setup *
 ********/

int ARToolKitNFTCore::setup(int width, int height, int cameraID) {
  int id = gARControllerID++;
  this->id = id;

  this->width = width;
  this->height = height;

  this->videoFrameSize = width * height * 4 * sizeof(ARUint8);
  // unique_ptr owns the frame buffers: exclusive ownership and automatic deallocation
  this->videoFrame = std::unique_ptr<ARUint8[]>(new ARUint8[this->videoFrameSize]);
  this->videoLuma = std::unique_ptr<ARUint8[]>(new ARUint8[this->width * this->height]);

  setCamera(id, cameraID);

  WEBARKIT_LOGi("Allocated videoFrameSize %d\n", this->videoFrameSize);

  return this->id;
}
