#ifndef THREADED_KPM_DETECTOR_H
#define THREADED_KPM_DETECTOR_H

#include <ARUtil/thread_sub.h>
#include <KPM/kpm.h>
#include <WebARKitTrackers/WebARKitNFT/NFTDetector.h>

#include <memory>
#include <vector>

/**
 * Detector that runs the KPM matching on a worker thread (trackingSub).
 *
 * start() hands the frame to the worker and returns false; the result is taken later with
 * collect(). Meant for a single caller thread: the worker is the only other thread.
 */
class ThreadedKpmDetector final : public NFTDetector {
 public:
  /**
   * Starts the worker.
   * @param kpmHandle the matcher to run; not owned, must outlive the detector
   * @return the detector, or nullptr when kpmHandle is null or the worker cannot start
   */
  static std::unique_ptr<ThreadedKpmDetector> create(KpmHandle *kpmHandle);

  /** Waits for a running pass, then stops the worker. The KPM handle can be freed afterwards. */
  ~ThreadedKpmDetector() override;

  ThreadedKpmDetector(const ThreadedKpmDetector &) = delete;
  ThreadedKpmDetector &operator=(const ThreadedKpmDetector &) = delete;

  bool collect(std::vector<NFTDetection> &out, int &resultNum) override;
  bool idle() const override { return !m_searchRunning; }
  bool start(ARUint8 *luma, const int *skipPages, int skipNum) override;

  /** Waits for the running pass and drops its result: the next collect() returns false. */
  void waitIdle() override;

 private:
  ThreadedKpmDetector(KpmHandle *kpmHandle, THREAD_HANDLE_T *threadHandle);

  KpmHandle *m_kpmHandle;
  THREAD_HANDLE_T *m_threadHandle;
  bool m_searchRunning;
};

#endif  // THREADED_KPM_DETECTOR_H
