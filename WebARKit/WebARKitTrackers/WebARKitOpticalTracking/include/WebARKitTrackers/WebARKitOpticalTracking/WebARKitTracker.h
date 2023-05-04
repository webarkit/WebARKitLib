#ifndef WEBARKIT_TRACKER_H
#define WEBARKIT_TRACKER_H

#include "WebARKitEnums.h"
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


    void initialize(webarkit::TRACKER_TYPE trackerType);

    void initTracker(uchar* refData, size_t refCols, size_t refRows);

    void processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace);

    std::vector<double> getOutputData();

    bool isValid();

  protected:
    bool resetTracking(cv::Mat& frameCurr);

    bool track(cv::Mat& currIm);

    void processFrame(cv::Mat& frame);

    bool homographyValid(cv::Mat& H);

    void fill_output(cv::Mat& H);

    void clear_output();

    bool _valid;

    std::vector<cv::Point2f> corners;

    cv::Mat m_H;

    cv::Mat prevIm;

    int numMatches;

    std::vector<cv::Point2f> framePts;

    bool initialized;

  private:
    std::vector<double> output; // 9 from homography matrix, 8 from warped corners*/

    cv::Ptr<cv::Feature2D> _featureDetector;

    cv::Ptr<cv::BFMatcher> _matcher;

    cv::Mat refGray, refDescr;

    std::vector<cv::KeyPoint> refKeyPts;

    void setDetectorType(webarkit::TRACKER_TYPE trackerType);
};

} // namespace webarkit

#endif