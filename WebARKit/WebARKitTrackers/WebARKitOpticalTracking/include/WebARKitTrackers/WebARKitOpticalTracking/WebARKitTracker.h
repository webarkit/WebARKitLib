#ifndef WEBARKIT_TRACKER_H
#define WEBARKIT_TRACKER_H

#include "WebARKitEnums.h"
#include <WebARKitLog.h>
#include <opencv2/xfeatures2d.hpp>

namespace webarkit {

class WebARKitTracker {
  public:
    WebARKitTracker();

    ~WebARKitTracker();

    WebARKitTracker(WebARKitTracker&&);

    WebARKitTracker& operator=(WebARKitTracker&&);

    void initialize(webarkit::TRACKER_TYPE trackerType);

    void initTracker(uchar* refData, size_t refCols, size_t refRows);

    void processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace);

    std::vector<double> getOutputData();

    bool isValid();

  private:
    class WebARKitTrackerImpl;

    std::shared_ptr<WebARKitTrackerImpl> _trackerImpl;
};

} // namespace webarkit

#endif