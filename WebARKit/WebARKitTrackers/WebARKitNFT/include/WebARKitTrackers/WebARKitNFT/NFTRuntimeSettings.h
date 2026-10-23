#ifndef NFT_RUNTIME_SETTINGS_H
#define NFT_RUNTIME_SETTINGS_H

#include <AR/ar.h>
#include <ARUtil/log.h>

#include <cmath>

/**
 * The AR2 tracking parameters. Before runtime settings they were constants in setupAR2();
 * the defaults below are those constants.
 */
struct NFTAR2Settings {
  float trackingThresh = 5.0f;
  float simThresh = 0.5f;
  int searchFeatureNum = 16;
  int templateSize1 = 6;
  int templateSize2 = 6;
  /**
   * 0 means "the preset's choice": 6, or 12 on a multi-CPU build with
   * config.cpuDependentSearchSize. It is kept as a sentinel, not resolved on store, so a later
   * setupAR2() resolves it again. It exists because the threaded preset already varied the
   * search size; leaving it out of the settings would hard-code that choice again.
   */
  int searchSize = 0;
};

/**
 * What may change while tracking, applied by ARToolKitNFTCore::configure(). Separate from
 * NFTTrackingConfig, which holds the variant choices fixed at construction (detector, AR2
 * variant, presets).
 */
struct NFTRuntimeSettings {
  /**
   * 0.1, not the 0.0001 the core used to start with (D4). ARControllerNFT already advertises
   * 0.1 and consumers re-set it right after initialisation; aligning the core is the only way
   * the first lens is right without a second computation. The near plane does not affect
   * tracking, only the projection.
   */
  ARdouble nearPlane = 0.1;
  ARdouble farPlane = 1000.0;
  bool continuousDetection = true;
  /** -1 means "the preset's defaultDetectionIntervalMs". getRuntimeSettings() resolves it. */
  double detectionIntervalMs = -1;
  bool filtering = false;
  /**
   * -1 leaves the current level alone (D9). The level is process-wide, not owned by the core,
   * so a caller that builds a fresh struct to change something else (the planes, say) must not
   * reset a level that was set elsewhere. It is the default for that reason. Otherwise one of
   * the AR_LOG_LEVEL_* values, written to arLogLevel and webarkitLogLevel.
   * getRuntimeSettings() reports the effective level, never -1.
   */
  int logLevel = -1;
  NFTAR2Settings ar2;
};

/**
 * Whether configure() accepts the settings. configure() validates everything first and applies
 * nothing on failure (D7), because an unsupported request must fail, never be silently
 * substituted. The individual setters stay lenient so existing callers do not break.
 *
 * AR2 itself checks nothing (every ar2Set* accepts any value; only ar2SetSearchFeatureNum
 * clamps), so the AR2 bounds are this project's: conservative, and wide enough to include
 * every value the presets use.
 */
inline bool nftRuntimeSettingsValid(const NFTRuntimeSettings &s) {
  // The planes build the projection matrix: a non-positive near plane or far <= near is
  // degenerate. NaN and infinity are rejected by isfinite.
  if (!std::isfinite(s.nearPlane) || !std::isfinite(s.farPlane)) return false;
  if (s.nearPlane <= 0 || s.farPlane <= s.nearPlane) return false;

  // -1 is the "follow the preset" sentinel; any other negative value is a mistake here.
  if (!std::isfinite(s.detectionIntervalMs)) return false;
  if (s.detectionIntervalMs != -1 && s.detectionIntervalMs < 0) return false;

  // -1 leaves the level unchanged; otherwise the levels of ARUtil/log.h, which WebARKit's
  // logger shares numerically.
  if (s.logLevel != -1 && (s.logLevel < AR_LOG_LEVEL_DEBUG || s.logLevel > AR_LOG_LEVEL_REL_INFO)) return false;

  const NFTAR2Settings &a = s.ar2;
  // The tracking threshold is a template-matching score cutoff; the presets use 5.0.
  if (!std::isfinite(a.trackingThresh) || a.trackingThresh <= 0.0f || a.trackingThresh > 100.0f) return false;
  // A normalised similarity, so (0, 1].
  if (!std::isfinite(a.simThresh) || a.simThresh <= 0.0f || a.simThresh > 1.0f) return false;
  // AR2_SEARCH_FEATURE_MAX is 40 and ar2SetSearchFeatureNum would clamp to [3, 40]: reject
  // instead of clamping, so the stored value is the one AR2 uses.
  if (a.searchFeatureNum < 3 || a.searchFeatureNum > 40) return false;
  // Template sizes are half-widths in pixels; the presets use 6. 32 is far beyond useful.
  if (a.templateSize1 < 1 || a.templateSize1 > 32) return false;
  if (a.templateSize2 < 1 || a.templateSize2 > 32) return false;
  // 0 follows the preset (6, or 12 when threaded); the presets never exceed 12, 64 is generous.
  if (a.searchSize != 0 && (a.searchSize < 1 || a.searchSize > 64)) return false;
  return true;
}

#endif // NFT_RUNTIME_SETTINGS_H
