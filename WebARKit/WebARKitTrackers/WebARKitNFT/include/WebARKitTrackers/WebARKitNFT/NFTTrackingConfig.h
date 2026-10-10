#ifndef NFT_TRACKING_CONFIG_H
#define NFT_TRACKING_CONFIG_H

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
  enum class Detector { Sync, Threaded };
  enum class AR2Variant { SingleThread, Threaded };

  Detector detector = Detector::Sync;                 // how the KPM detection runs
  AR2Variant ar2Variant = AR2Variant::SingleThread;   // ar2*Mod handles, or ar2* with AR2_TRACKING_DEFAULT_THREAD_NUM
  bool cpuDependentSearchSize = false;                // search size 12 when threadGetCPU() > 1, else 6
  double defaultDetectionIntervalMs = 300.0;          // minimum time between two KPM detections
  bool poseFilteringSupported = true;                 // the pose filter can be enabled
  double (*clock)() = &nftDefaultClockMs;             // current time in milliseconds; null means nftDefaultClockMs
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
  config.poseFilteringSupported = false;
  config.clock = &nftDefaultClockMs;
  return config;
}

#endif // NFT_TRACKING_CONFIG_H
