#include <WebARKitTrackers/WebARKitNFT/SyncKpmDetector.h>

#include <cstring>

SyncKpmDetector::SyncKpmDetector(KpmHandle *kpmHandle)
    : m_kpmHandle(kpmHandle), m_resultNum(-1), m_hasResult(false) {}

bool SyncKpmDetector::start(ARUint8 *luma, const int *skipPages, int skipNum) {
  if (!m_kpmHandle || !luma) return false;

  // Pages already being tracked need no pose from KPM this pass.
  // kpmMatching() clears the skip flags again when it finishes.
  if (skipPages && skipNum > 0) {
    // kpmSetMatchingSkipPage() takes a non-const array.
    std::vector<int> pages(skipPages, skipPages + skipNum);
    kpmSetMatchingSkipPage(m_kpmHandle, pages.data(), skipNum);
  }

  kpmMatching(m_kpmHandle, luma);

  KpmResult *kpmResult = nullptr;
  int kpmResultNum = -1;
  kpmGetResult(m_kpmHandle, &kpmResult, &kpmResultNum);

  m_detections.clear();
  for (int i = 0; i < kpmResultNum; i++) {
    if (kpmResult[i].camPoseF != 0) continue;
    NFTDetection detection;
    detection.page = kpmResult[i].pageNo;
    std::memcpy(detection.trans, kpmResult[i].camPose, sizeof(detection.trans));
    m_detections.push_back(detection);
  }
  m_resultNum = kpmResultNum;
  m_hasResult = true;
  return true;
}

bool SyncKpmDetector::collect(std::vector<NFTDetection> &out, int &resultNum) {
  if (!m_hasResult) return false;
  out.swap(m_detections);
  m_detections.clear();
  resultNum = m_resultNum;
  m_hasResult = false;
  return true;
}
