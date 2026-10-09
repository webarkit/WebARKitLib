#ifndef NFT_DETECTOR_H
#define NFT_DETECTOR_H

#include <AR/ar.h>

#include <vector>

/** One marker found by a KPM detection pass: its page (marker index) and camera pose. */
struct NFTDetection {
  int page;
  float trans[3][4];
};

/**
 * Runs the KPM (key-point matching) detection of the NFT tracking core.
 *
 * A pass is started with start() and its result is taken with collect(). The synchronous
 * implementation finishes a pass inside start(); a threaded one finishes it on a worker, so
 * the caller never assumes the result is ready right after start().
 */
class NFTDetector {
 public:
  virtual ~NFTDetector() = default;

  /**
   * Takes the result of the last finished pass, once.
   * @param out receives the detections of the pass (poses KPM could compute), replacing its contents
   * @param resultNum receives the number of results KPM reported (including those without a pose)
   * @return true when a finished pass was collected, false when there is none (out is left untouched)
   */
  virtual bool collect(std::vector<NFTDetection> &out, int &resultNum) = 0;

  /** True when no pass is running, so start() can be called. */
  virtual bool idle() const = 0;

  /**
   * Starts a detection pass on a luma frame.
   * @param luma 8-bit luma of the frame, as large as the KPM handle's frame; read during the pass
   * @param skipPages pages KPM must not report this pass (they are tracked already), may be null
   * @param skipNum number of entries of skipPages
   * @return true when the pass started (or, for a synchronous detector, finished)
   */
  virtual bool start(ARUint8 *luma, const int *skipPages, int skipNum) = 0;

  /** Blocks until no pass is running. */
  virtual void waitIdle() = 0;
};

#endif  // NFT_DETECTOR_H
