#include <WebARKitPattern.h>
#include <iostream>
#include <opencv2/calib3d.hpp>

void WebARKitPatternTrackingInfo::computePose(const WebARKitPattern& pattern, std::vector<cv::Point2f>& imgPoints,
                                              cv::Mat& caMatrix, cv::Mat& distCoeffs) {
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1); // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1); // output translation vector

    cv::solvePnPRansac(pattern.points3d, imgPoints, caMatrix, distCoeffs, rvec, tvec);

    cv::Mat rMat;
    cv::Rodrigues(rvec, rMat);
    cv::hconcat(rMat, tvec, pose3d);

    std::cout << "pose3d: " << pose3d.rows << "x" << pose3d.cols << std::endl;
}