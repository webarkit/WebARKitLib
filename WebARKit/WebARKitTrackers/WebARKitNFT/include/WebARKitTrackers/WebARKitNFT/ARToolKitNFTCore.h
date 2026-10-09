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
 * Not thread-safe: drive it from one thread. The only other thread is the threaded KPM
 * detector's worker, which the core waits for before it changes or frees what the worker uses.
 *
 * Typical use: loadCamera(), setup(), setupAR2(), then addNFTMarkers().
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
     * Loads a camera parameter file into the process-wide camera registry.
     * @return the camera id, valid for any core, or -1 when the file cannot be loaded
     */
    static int loadCamera(const std::string &path);

    /**
     * Allocates the frame buffers for a width x height RGBA frame and applies the camera.
     * @return this controller's id
     */
    int setup(int width, int height, int cameraID);

    /**
     * Applies a camera from the registry: resizes it to the frame, frees and recreates
     * paramLT (freeing the AR2 handle with it) and recomputes the lens. Call setupAR2() after it.
     * @return 0, or -1 when the camera id is unknown or paramLT cannot be created
     */
    int setCamera(int id, int cameraID);

    /**
     * Creates the AR2 tracking handle (variant and settings from the config) and the KPM
     * handle from paramLT. Call after setup() and after every setCamera().
     * @return 0, or -1 on failure
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
     * @return the ids of the new markers, or an empty vector when any of them fails to load
     *         (the earlier markers stay loaded)
     */
    std::vector<int> addNFTMarkers(const std::vector<std::string> &paths);

    /** Decompresses a .zft archive into tempPath. @return 1 on success, -1 on failure */
    int decompressZFT(const std::string &path, const std::string &tempPath);

    /**
     * Data of a loaded marker, still available after teardown(), as in the bindings.
     * An out-of-range index aborts (std::vector::at).
     */
    nftMarker getNFTData(int index) const;

    /** The number of loaded markers. */
    int markerCount() const { return surfaceSetCount; }

    /** The tracking state of a loaded marker, or nullptr when index is out of range. */
    const NFTMarkerState *markerState(int index) const;

    /** Frees everything the core owns: detector first, then the handles and the markers. */
    int teardown();

private:
    struct KpmHandleDeleter {
        void operator()(KpmHandle *handle) const;
    };
    using KpmHandlePtr = std::unique_ptr<KpmHandle, KpmHandleDeleter>;

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
