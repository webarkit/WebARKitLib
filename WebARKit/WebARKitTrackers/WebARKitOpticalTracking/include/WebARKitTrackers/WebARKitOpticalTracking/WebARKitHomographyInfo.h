#ifndef WEBARKIT_HOMOGRAPHY_INFO_H
#define WEBARKIT_HOMOGRAPHY_INFO_H
#include "WebARKitConfig.h"

namespace webarkit {
namespace homography {

class WebARKitHomographyInfo {
  public:
    WebARKitHomographyInfo();

    WebARKitHomographyInfo(cv::Mat hom, std::vector<uchar> newStatus, std::vector<cv::DMatch> matches);

    bool validHomography;
    cv::Mat homography;
    std::vector<uchar> status;
    std::vector<cv::DMatch> inlier_matches;
};

} // namespace homography

} // namespace webarkit

#endif