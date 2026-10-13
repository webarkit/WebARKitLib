#include <WebARKitLog.h>
#include <WebARKitTrackers/WebARKitNFT/ARToolKitNFTCore.h>
#include <WebARKitTrackers/WebARKitNFT/ThreadedKpmDetector.h>
#include <WebARKitTrackers/WebARKitNFT/trackingSub.h>

#include <cstring>

// One worker result per page: the results buffer holds TRACKING_INIT_MAX_RESULTS entries,
// so no page the core can load is ever cut from a pass.
static_assert(PAGES_MAX == TRACKING_INIT_MAX_RESULTS,
              "PAGES_MAX must equal TRACKING_INIT_MAX_RESULTS");

ThreadedKpmDetector::ThreadedKpmDetector(KpmHandle *kpmHandle, THREAD_HANDLE_T *threadHandle)
    : m_kpmHandle(kpmHandle), m_threadHandle(threadHandle), m_searchRunning(false) {}

std::unique_ptr<ThreadedKpmDetector> ThreadedKpmDetector::create(KpmHandle *kpmHandle) {
  THREAD_HANDLE_T *threadHandle = trackingInitInit(kpmHandle);
  if (!threadHandle) {
    WEBARKIT_LOGe("ThreadedKpmDetector: unable to start the KPM worker.\n");
    return nullptr;
  }
  return std::unique_ptr<ThreadedKpmDetector>(new ThreadedKpmDetector(kpmHandle, threadHandle));
}

ThreadedKpmDetector::~ThreadedKpmDetector() {
  // The worker uses the KPM handle until the pass ends: wait, then let the thread exit.
  waitIdle();
  trackingInitQuit(&m_threadHandle);
}

bool ThreadedKpmDetector::start(ARUint8 *luma, const int *skipPages, int skipNum) {
  if (!m_threadHandle || !luma || m_searchRunning) return false;

  // The worker is idle here, so setting the skip pages cannot race with it;
  // kpmMatching() clears them when the pass ends.
  if (skipPages && skipNum > 0) {
    // kpmSetMatchingSkipPage() takes a non-const array.
    std::vector<int> pages(skipPages, skipPages + skipNum);
    kpmSetMatchingSkipPage(m_kpmHandle, pages.data(), skipNum);
  }

  // trackingInitStart() copies the frame, so luma is free again on return.
  if (trackingInitStart(m_threadHandle, luma) != 0) return false;
  m_searchRunning = true;
  return false;  // the pass finishes on the worker: collect() returns it later
}

bool ThreadedKpmDetector::collect(std::vector<NFTDetection> &out, int &resultNum) {
  if (!m_searchRunning) return false;

  TrackingInitResult results[TRACKING_INIT_MAX_RESULTS];
  int n = 0;
  const int ret = trackingInitGetResults(m_threadHandle, results, TRACKING_INIT_MAX_RESULTS, &n);
  if (ret == 0) return false;  // still searching

  // Finished (1) or failed (-1): either way the worker is free again.
  m_searchRunning = false;
  out.clear();
  if (ret != 1) {
    resultNum = -1;
    return true;
  }
  resultNum = n;
  for (int i = 0; i < n; i++) {
    NFTDetection detection;
    detection.page = results[i].page;
    std::memcpy(detection.trans, results[i].trans, sizeof(detection.trans));
    out.push_back(detection);
  }
  return true;
}

void ThreadedKpmDetector::waitIdle() {
  if (!m_searchRunning) return;
  // Not a bare threadEndWait(): trackingSub must forget the uncollected search, or the next
  // trackingInitStart() is refused and the search after it never reports.
  trackingInitDiscard(m_threadHandle);
  m_searchRunning = false;
}
