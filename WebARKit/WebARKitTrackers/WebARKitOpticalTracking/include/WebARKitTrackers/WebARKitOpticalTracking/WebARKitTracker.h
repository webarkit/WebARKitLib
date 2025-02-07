#ifndef WEBARKIT_TRACKER_H
#define WEBARKIT_TRACKER_H

#include "WebARKitEnums.h"
#include <WebARKitLog.h>
#include <WebARKitCamera.h>
#include <WebARKitGL.h>
#include <WebARKitPattern.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>
#include <opencv2/xfeatures2d.hpp>

namespace webarkit {

class WebARKitTracker {
  public:
    WebARKitTracker();

    ~WebARKitTracker();

    WebARKitTracker(WebARKitTracker&&);

    WebARKitTracker& operator=(WebARKitTracker&&);

    void initialize(webarkit::TRACKER_TYPE trackerType, int frameWidth, int frameHeight);

    void initTracker(cv::Mat refData, size_t refCols, size_t refRows, ColorSpace colorSpace);

    void initTracker(uchar* refData, size_t refCols, size_t refRows, ColorSpace colorSpace);

    void processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace, BLUR_TYPE blurType);

    std::vector<double> getOutputData();

    cv::Mat getPoseMatrix();
    
    float* getPoseMatrix2();

    //float (*WebARKitTracker::getPoseMatrix3()[3][4]);

    cv::Mat getGLViewMatrix();

    std::array<double, 16> getCameraProjectionMatrix();

    bool isValid();

  private:
    class WebARKitTrackerImpl;

    std::shared_ptr<WebARKitTrackerImpl> _trackerImpl;
};

} // namespace webarkit

#endif