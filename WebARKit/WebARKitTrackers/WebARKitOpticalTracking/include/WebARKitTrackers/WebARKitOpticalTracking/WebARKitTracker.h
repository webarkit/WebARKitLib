#ifndef WEBARKIT_TRACKER_H
#define WEBARKIT_TRACKER_H

#include "WebARKitEnums.h"
#include <WebARKitLog.h>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

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