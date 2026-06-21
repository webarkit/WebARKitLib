#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitHomographyInfo.h>

namespace webarkit {
namespace homography {

WebARKitHomographyInfo::WebARKitHomographyInfo() { validHomography = false; }

WebARKitHomographyInfo::WebARKitHomographyInfo(cv::Mat hom, std::vector<uchar> newStatus,
                                               std::vector<cv::DMatch> matches) {
    homography = hom;
    status = newStatus;
    inlier_matches = matches;
    if (matches.size() > 4) {
        validHomography = true;
    }
}

} // namespace homography

} // namespace webarkit
