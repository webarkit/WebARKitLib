#ifndef SYNC_KPM_DETECTOR_H
#define SYNC_KPM_DETECTOR_H

#include <KPM/kpm.h>
#include <WebARKitTrackers/WebARKitNFT/NFTDetector.h>

#include <vector>

/**
 * Detector that runs the KPM matching inside start(), on the calling thread.
 * The result waits in the detector until collect() takes it.
 */
class SyncKpmDetector final : public NFTDetector {
 public:
  /** @param kpmHandle the matcher to run; not owned, must outlive the detector */
  explicit SyncKpmDetector(KpmHandle *kpmHandle);

  bool collect(std::vector<NFTDetection> &out, int &resultNum) override;
  bool idle() const override { return true; }
  bool start(ARUint8 *luma, const int *skipPages, int skipNum) override;
  void waitIdle() override {}

 private:
  KpmHandle *m_kpmHandle;
  std::vector<NFTDetection> m_detections;
  int m_resultNum;
  bool m_hasResult;
};

#endif  // SYNC_KPM_DETECTOR_H
