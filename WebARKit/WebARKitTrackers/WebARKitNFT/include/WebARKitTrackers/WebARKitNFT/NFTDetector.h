#ifndef NFT_DETECTOR_H
#define NFT_DETECTOR_H

#include <AR/ar.h>
// Also defines WEBARKIT_NFT_HAS_EXTERNAL_DETECTOR, so it is seen from this header alone.
#include <WebARKitTrackers/WebARKitNFT/NFTTrackingConfig.h>

#include <string>
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
 *
 * The camera: an interface method never passes it. A detector supplied through
 * NFTTrackingConfig::makeDetector receives it, with the KPM handle, when the factory creates
 * it. The core destroys the detector whenever the camera can change (setupAR2(), setCamera(),
 * teardown()) and creates a new one with the next addNFTMarkers(). So that recreation is the
 * only camera notification: no detector outlives a camera change.
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

  /**
   * True when no pass is running, so start() can be called. A pass that has finished but
   * whose result has not been collected yet still counts as running.
   */
  virtual bool idle() const = 0;

  /**
   * Starts a detection pass on a luma frame.
   * @param luma 8-bit luma of the frame, as large as the KPM handle's frame; read during the
   *        pass. It is the core's frame buffer, overwritten by the next setVideoFrame(): a
   *        detector that finishes the pass after start() returns copies it, as the threaded
   *        one does.
   * @param skipPages pages KPM must not report this pass (they are tracked already), may be
   *        null; valid only during the call
   * @param skipNum number of entries of skipPages
   * @return true only when the pass has already finished inside start(): its result is ready,
   *         so the caller collects at once. false when the pass runs asynchronously or was
   *         refused; use idle() to tell whether a pass is running.
   */
  virtual bool start(ARUint8 *luma, const int *skipPages, int skipNum) = 0;

  /**
   * Ends the running pass, if any: waits for it (threaded) or abandons it (a detector that
   * cannot block, such as one running KPM in a Web Worker). Either way its result is never
   * collected: afterwards idle() is true and collect() returns false until a new pass has
   * finished. A detector that abandons a pass must itself discard the pass's late result,
   * for example with a generation counter. The core calls it, when idle() is false, before it
   * replaces what a pass reads (the KPM reference data, the KPM handle, the camera). In
   * teardown() it destroys the detector instead, so a detector's destructor must end a
   * running pass the same way.
   */
  virtual void waitIdle() = 0;

  /**
   * Tells the detector which markers the core loaded: the pages it assigned and the dataset
   * paths they were loaded from (without extension, as given to addNFTMarkers(), so in the
   * core's file system), in the same order. Called once at creation with every marker
   * already loaded, then after each addNFTMarkers() that loads a batch, with that batch only.
   * Never called with empty vectors nor for a batch that failed. Pages are the ids
   * addNFTMarkers() returns. Each detector starts with no markers: what an earlier detector
   * was told does not carry over. Default: nothing.
   *
   * It is called from inside addNFTMarkers() and must not call back into the core: nothing
   * that loads markers, sets the camera or tears down (addNFTMarkers(), setup(),
   * setCamera(), setupAR2(), teardown()). Such a call can destroy this detector while it
   * runs. If the detector is gone when the call returns, the core logs an error and that
   * addNFTMarkers() fails.
   */
  virtual void markersAdded(const std::vector<int> & /*pages*/,
                            const std::vector<std::string> & /*datasetPathnames*/) {}
};

#endif  // NFT_DETECTOR_H
