#include <WebARKitPattern.h>
#include <opencv2/calib3d.hpp>

void WebARKitPatternTrackingInfo::computePose(const WebARKitPattern& pattern, cv::Mat caMatrix, cv::Mat distCoeffs)
{
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux, taux;

    cv::solvePnPRansac(pattern.points3d, points2d, caMatrix, distCoeffs, raux, taux);
    raux.convertTo(Rvec, CV_32F);
    taux.convertTo(Tvec, CV_32F);

    cv::Mat_<float> rotMat(3, 3);
    cv::Rodrigues(Rvec, rotMat);

    cv::hconcat(rotMat, Tvec, pose3d);
}