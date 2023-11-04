#include <WebARKitPattern.h>
#include <opencv2/calib3d.hpp>

WebARKitPatternTrackingInfo::WebARKitPatternTrackingInfo() {
    pose3d = cv::Mat::zeros(3, 4, CV_64FC1);
    m_scale = 1.0f;
}

void WebARKitPatternTrackingInfo::computePose(std::vector<cv::Point3f>& treeDPoints,
                                              std::vector<cv::Point2f>& imgPoints, cv::Mat& caMatrix,
                                              cv::Mat& distCoeffs) {
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1); // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1); // output translation vector

    cv::solvePnPRansac(treeDPoints, imgPoints, caMatrix, distCoeffs, rvec, tvec);

    cv::Mat rMat;
    cv::Rodrigues(rvec, rMat);
    cv::hconcat(rMat, tvec, pose3d);

    invertPose();
}

void WebARKitPatternTrackingInfo::invertPose() {

    cv::Mat invertPose(3, 4, CV_64FC1);
    for (auto j = 0; j < 3; j++) {
        invertPose.at<double>(j, 0) = pose3d.at<double>(j, 0);
        invertPose.at<double>(j, 1) = -pose3d.at<double>(j, 1);
        invertPose.at<double>(j, 2) = -pose3d.at<double>(j, 2);
        invertPose.at<double>(j, 3) = pose3d.at<double>(j, 3) * m_scale * 0.001f * 1.64f;
    }

    pose3d = invertPose;
}