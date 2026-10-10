#ifndef ARTOOLKIT_NFT_CORE_H
#define ARTOOLKIT_NFT_CORE_H

#include <AR/ar.h>
#include <AR2/tracking.h>
#include <KPM/kpm.h>
#include <WebARKitTrackers/WebARKitNFT/NFTDetector.h>
#include <WebARKitTrackers/WebARKitNFT/NFTMarkerState.h>
#include <WebARKitTrackers/WebARKitNFT/NFTTrackingConfig.h>

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

const int PAGES_MAX = 20; // Maximum number of pages expected. You can change this down (to save memory) or up (to accomodate more pages.)

/** Size and resolution of a loaded NFT marker, from its first image set scale. */
struct nftMarker
{
    int id_NFT;
    int width_NFT;
    int height_NFT;
    int dpi_NFT;
};

/**
 * The NFT tracking core shared by the jsartoolkitNFT bindings and native consumers: camera,
 * markers, frame buffers, per-marker state, AR2 tracking and the KPM detection policy.
 *
 * NFT only: the ARHandle / AR3DHandle of the bindings are not part of it. They are created
 * from cameraParamLT() and cameraParam() by the caller, which must delete them before
 * setCamera() or teardown() frees paramLT.
 *
 * Not thread-safe: drive each core from one thread. The only other thread is the threaded KPM
 * detector's worker, which the core waits for before it changes or frees what the worker uses.
 * loadCamera(), setup() and setCamera() also use process-wide state shared by every core and
 * not synchronised (the camera registry, the camera and controller id counters): cores driven
 * from different threads must not call them concurrently.
 *
 * Typical use: loadCamera(), setup() (on the first setup, check cameraParamLT() != nullptr),
 * setupAR2(), then addNFTMarkers(); then, per frame, setVideoFrame() and detectNFTMarker().
 */
class ARToolKitNFTCore
{
public:
    /**
     * @param config variant-specific settings, e.g. singleThreadPreset() or threadedPreset()
     * @param withFiltering whether poses are filtered (only when config.poseFilteringSupported)
     */
    explicit ARToolKitNFTCore(const NFTTrackingConfig &config, bool withFiltering = false);
    ~ARToolKitNFTCore();

    ARToolKitNFTCore(const ARToolKitNFTCore &) = delete;
    ARToolKitNFTCore &operator=(const ARToolKitNFTCore &) = delete;

    // Camera

    /**
     * Loads a camera parameter file into the process-wide camera registry (not synchronised:
     * see the class comment).
     * @return the camera id, valid for any core, or -1 when the file cannot be loaded
     */
    static int loadCamera(const std::string &path);

    /**
     * Allocates the frame buffers for a width x height RGBA frame and applies the camera with
     * setCamera(). Takes the next id from a process-wide counter (see the class comment).
     * @return this controller's id, also when the camera could not be applied (as in the
     *         bindings). On the first setup(), cameraParamLT() != nullptr tells that it was.
     *         On a later setup() a failed camera leaves the previous paramLT in place, so that
     *         check proves nothing; call setCamera() and check its return value instead.
     */
    int setup(int width, int height, int cameraID);

    /**
     * Applies a camera from the process-wide registry (not synchronised: see the class
     * comment): resizes it to the frame, frees and recreates
     * paramLT and recomputes the lens. Everything built on the old paramLT is freed with it:
     * the AR2 handle, the KPM handle and the detector (a running search is waited for and
     * dropped). Call setupAR2() after it; until then nothing is detected and markers being
     * tracked are lost on the next frame. The markers stay loaded.
     * @return 0, or -1 when the camera id is unknown (nothing changes) or paramLT cannot be
     *         created
     */
    int setCamera(int id, int cameraID);

    /**
     * Creates the AR2 tracking handle (variant and settings from the config) and the KPM
     * handle from paramLT. Call after setup() and after every setCamera().
     *
     * A second call (or the first after a setCamera()) replaces the KPM handle: the markers
     * already loaded stay loaded and tracked, but are not detected again until the next
     * addNFTMarkers() hands the new handle the reference data. addNFTMarkers({}) does that
     * without adding markers, and returns an empty vector, as a failure does.
     * @return 0, or -1 on failure (no camera, or a handle cannot be created)
     */
    int setupAR2();

    /** The OpenGL projection matrix of the camera: 16 values, column-major. */
    const ARdouble *cameraLens() const { return cameraLens_; }
    const ARParam &cameraParam() const { return param; }
    ARParamLT *cameraParamLT() const { return paramLT; }

    void setProjectionNearPlane(ARdouble projectionNearPlane);
    ARdouble getProjectionNearPlane() const;
    void setProjectionFarPlane(ARdouble projectionFarPlane);
    ARdouble getProjectionFarPlane() const;
    /** Recomputes cameraLens() from paramLT and the projection planes. */
    void recalculateCameraLens();

    // Markers

    /**
     * Loads a batch of NFT markers (paths without extension) after the ones already loaded.
     * Needs setupAR2(); it also hands every loaded marker to the current KPM handle (see
     * setupAR2()).
     * @return the ids of the new markers, or an empty vector when any of them fails to load
     *         (the earlier markers stay loaded), when there is no KPM handle, or when paths
     *         is empty
     */
    std::vector<int> addNFTMarkers(const std::vector<std::string> &paths);

    /** Decompresses a .zft archive into tempPath. @return 1 on success, -1 on failure */
    int decompressZFT(const std::string &path, const std::string &tempPath);

    /**
     * Data of a loaded marker, still available after teardown(), as in the bindings, until
     * markers are loaded again (their ids restart at 0 and replace the old entries).
     * An out-of-range index aborts (std::vector::at).
     */
    nftMarker getNFTData(int index) const;

    /** The number of loaded markers. */
    int markerCount() const { return surfaceSetCount; }

    /** The tracking state of a loaded marker, or nullptr when index is out of range. */
    const NFTMarkerState *markerState(int index) const;

    // Frames, detection and tracking

    /**
     * Copies a frame into the core: rgba is the width x height RGBA frame given to setup(),
     * luma its 8-bit luma (width x height bytes). A null pointer leaves that buffer as it is.
     * Does nothing before setup().
     */
    void setVideoFrame(const ARUint8 *rgba, const ARUint8 *luma);

    /**
     * Runs one frame: takes a finished KPM pass, starts a new one when the detection policy
     * says so, then tracks every marker found. Call after setVideoFrame().
     * @return the result count of the KPM pass collected during this call (single-thread:
     *         the pass that ran in it; threaded: one that finished on the worker), or -1 when
     *         none was collected
     */
    int detectNFTMarker();

    /** Turns pose filtering on or off; it only applies when config.poseFilteringSupported. */
    void setFiltering(bool enableFiltering);

    /** Whether KPM keeps searching for untracked markers while some marker is tracked. */
    void setContinuousDetection(bool enabled);

    /**
     * Minimum time between the end of a KPM pass and the start of the next while some marker
     * is tracked. Negative values and NaN mean 0 (every frame).
     */
    void setDetectionInterval(double ms);

    /**
     * Frees what the core owns: the detector first (waiting for a running search), then the
     * KPM and AR2 handles, the markers' surface sets and reference data, paramLT, the
     * per-marker state and the frame buffers. markerCount() is then 0, but getNFTData() still
     * returns the data of the markers loaded before. The core can be set up again with
     * setup() and setupAR2(). Called by the destructor; a second call frees nothing.
     * @return 0
     */
    int teardown();

private:
    struct KpmHandleDeleter {
        void operator()(KpmHandle *handle) const;
    };
    using KpmHandlePtr = std::unique_ptr<KpmHandle, KpmHandleDeleter>;

    bool allMarkersTracked() const;
    bool anyMarkerTracked() const;
    /**
     * Takes a finished KPM pass from the detector, if there is one, and starts tracking the
     * markers it found. @return true when a pass was collected; resultNum then holds its count
     */
    bool collectDetections(int &resultNum);
    void trackMarkers();

    KpmHandlePtr createKpmHandle(ARParamLT *cparamLT);
    std::unique_ptr<NFTDetector> createDetector();
    /** Waits for a running KPM search and drops its result. */
    void dropRunningSearch();
    /** Frees the AR2 handle with the call matching the config's AR2 variant. */
    void deleteAR2Handle();
    /** Frees paramLT and the AR2 handle (what setCamera() recreates). */
    void deleteHandle();

    NFTTrackingConfig config;

    bool withFiltering;
    // Filtering-related variables
    double filterCutoffFrequency;
    double filterSampleRate;

    int id;

    ARParam param;
    ARParamLT *paramLT;

    std::unique_ptr<ARUint8[]> videoFrame;
    int videoFrameSize;
    std::unique_ptr<ARUint8[]> videoLuma;

    int width;
    int height;

    KpmHandlePtr kpmHandle;
    AR2HandleT *ar2Handle;
    // Runs the KPM searches on kpmHandle; created by the first addNFTMarkers().
    std::unique_ptr<NFTDetector> detector;

    // One state per loadable page; index = page number = marker id.
    std::array<NFTMarkerState, PAGES_MAX> markerStates;

    // Detection policy. KPM runs on every frame while no marker is tracked.
    // While some are tracked and some are not, it runs at most once every
    // detectionIntervalMs, counted from the end of the previous pass, and not at
    // all if continuousDetection is off. While every loaded marker is tracked
    // it does not run.
    bool continuousDetection;
    double detectionIntervalMs;
    // When the last pass finished; -infinity so the first pass is never throttled.
    double lastKpmEndMs;

    int surfaceSetCount;
    AR2SurfaceSetT *surfaceSet[PAGES_MAX];
    // KPM reference data of every marker loaded so far, across all
    // addNFTMarkers() calls. kpmSetRefDataSet() rebuilds the matcher from
    // scratch, so each call must hand it the whole set, not just the new batch.
    KpmRefDataSet *refDataSetAll;
    std::vector<nftMarker> nftMarkers;

    ARdouble nearPlane;
    ARdouble farPlane;

    ARdouble cameraLens_[16];
    AR_PIXEL_FORMAT pixFormat;
};

#endif // ARTOOLKIT_NFT_CORE_H
