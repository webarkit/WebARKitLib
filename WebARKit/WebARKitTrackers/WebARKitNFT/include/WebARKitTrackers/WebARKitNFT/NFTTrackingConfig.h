#ifndef NFT_TRACKING_CONFIG_H
#define NFT_TRACKING_CONFIG_H

// ARParam is an anonymous-struct typedef and cannot be forward-declared; the factory below
// takes one, so this header needs the AR declarations (<AR/param.h> must not come first).
#include <AR/ar.h>

#include <memory>

/**
 * Defined when NFTTrackingConfig::Detector::External and the makeDetector factory exist, so a
 * consumer can build against versions with and without them (#115).
 */
#define WEBARKIT_NFT_HAS_EXTERNAL_DETECTOR 1

class NFTDetector;
typedef struct _KpmHandle KpmHandle;

/** The default clock: emscripten_get_now() on Emscripten, steady_clock elsewhere. Defined in NFTClock.cpp. */
double nftDefaultClockMs();

/**
 * Selects the variant-specific behaviour of the NFT tracking core.
 *
 * The single-thread and the threaded JS bindings differ only in these settings;
 * singleThreadPreset() and threadedPreset() reproduce each of them exactly. A default- or
 * value-initialised config holds the single-thread settings.
 */
struct NFTTrackingConfig {
  /**
   * Sync: SyncKpmDetector. Threaded: ThreadedKpmDetector (needs WEBARKIT_NFT_THREADS).
   * External: whatever makeDetector returns, in any build.
   */
  enum class Detector { Sync, Threaded, External };
  enum class AR2Variant { SingleThread, Threaded };

  Detector detector = Detector::Sync;                 // how the KPM detection runs
  AR2Variant ar2Variant = AR2Variant::SingleThread;   // ar2*Mod handles, or ar2* with AR2_TRACKING_DEFAULT_THREAD_NUM
  bool cpuDependentSearchSize = false;                // search size 12 when threadGetCPU() > 1, else 6
  double defaultDetectionIntervalMs = 300.0;          // minimum time between two KPM detections
  bool poseFilteringSupported = true;                 // the pose filter can be enabled
  double (*clock)() = &nftDefaultClockMs;             // current time in milliseconds; null means nftDefaultClockMs

  /**
   * Creates the detector when detector is External; unused otherwise. The core calls it where
   * it would create its own detector. That is in setupAR2() when markers are already loaded,
   * and otherwise in the first addNFTMarkers() after it; so again after every setupAR2(),
   * setCamera() or teardown(), which destroy the previous one. Null, or a factory returning
   * null, makes that addNFTMarkers() or setupAR2() fail with an error (a setupAR2() with no
   * markers loaded does not call it yet, so it succeeds and the failure shows in the first
   * addNFTMarkers()). Each call starts afresh: the new detector learns the loaded markers
   * from markersAdded(). The factory must not call into the core (setupAR2(), setCamera(),
   * teardown(), addNFTMarkers() ...): such a call is refused before it changes anything, with
   * an error, and the setupAR2() or addNFTMarkers() that runs the factory then fails and
   * discards the detector, so a detector over a replaced KPM handle is never accepted.
   * @param user makeDetectorUser
   * @param kpm the core's KPM handle, owned by the core and valid for the detector's
   *        lifetime. Whether it already holds reference data depends on the caller: from
   *        setupAR2() it holds that of the markers already loaded; from addNFTMarkers() it is
   *        set after the call, by the same addNFTMarkers() if it succeeds, otherwise by the
   *        next one that does. Do not rely on either in the constructor. A detector that runs
   *        KPM elsewhere may ignore it, or use it to run KPM in-process as a fallback.
   * @param camera the camera the core uses, already resized to the frame by setCamera(); a
   *        copy can be sent to a detector that does not share memory with the core
   */
  std::unique_ptr<NFTDetector> (*makeDetector)(void *user, KpmHandle *kpm,
                                               const ARParam &camera) = nullptr;
  void *makeDetectorUser = nullptr;                   // passed to makeDetector, not owned
};

/** Settings of the single-thread binding. */
inline NFTTrackingConfig singleThreadPreset() {
  NFTTrackingConfig config;
  config.detector = NFTTrackingConfig::Detector::Sync;
  config.ar2Variant = NFTTrackingConfig::AR2Variant::SingleThread;
  config.cpuDependentSearchSize = false;
  config.defaultDetectionIntervalMs = 300.0;
  config.poseFilteringSupported = true;
  config.clock = &nftDefaultClockMs;
  return config;
}

/** Settings of the threaded binding. */
inline NFTTrackingConfig threadedPreset() {
  NFTTrackingConfig config;
  config.detector = NFTTrackingConfig::Detector::Threaded;
  config.ar2Variant = NFTTrackingConfig::AR2Variant::Threaded;
  config.cpuDependentSearchSize = true;
  config.defaultDetectionIntervalMs = 0.0;
  config.poseFilteringSupported = true;  // #692: the threaded build used to never filter
  config.clock = &nftDefaultClockMs;
  return config;
}

#endif // NFT_TRACKING_CONFIG_H
