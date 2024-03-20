#include <WebARKitPattern.h>
#include <opencv2/calib3d.hpp>

WebARKitPatternTrackingInfo::WebARKitPatternTrackingInfo() {
    pose3d = cv::Mat::zeros(4, 4, CV_64FC1);
    glViewMatrix = cv::Mat::zeros(4, 4, CV_64F);
    m_scale = 1.0f;
}

void WebARKitPatternTrackingInfo::computePose(std::vector<cv::Point3f>& treeDPoints,
                                              std::vector<cv::Point2f>& imgPoints, const cv::Matx33f& caMatrix,
                                              const cv::Mat& distCoeffs) {
    //cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1); // output rotation vector
    //cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1); // output translation vector
    cv::Mat rvec, tvec;

    cv::solvePnPRansac(treeDPoints, imgPoints, caMatrix, distCoeffs, rvec, tvec);

    cv::Mat rMat;
    cv::Rodrigues(rvec, rMat);
    //cv::hconcat(rMat, tvec, pose3d);

    for (unsigned int row = 0; row < 3; ++row) {
        for (unsigned int col = 0; col < 3; ++col) {
            pose3d.at<double>(row, col) = rMat.at<double>(row, col);
        }
        pose3d.at<double>(row, 3) = tvec.at<double>(row, 0);
    }
    pose3d.at<double>(3, 3) = 1.0f;

    invertPose();
}

void WebARKitPatternTrackingInfo::computeGLviewMatrix() {
   cv::transpose(pose3d , glViewMatrix);
}

void WebARKitPatternTrackingInfo::invertPose() {

    /*cv::Mat invertPose(3, 4, CV_64FC1);
    for (auto j = 0; j < 3; j++) {
        invertPose.at<double>(j, 0) = pose3d.at<double>(j, 0);
        invertPose.at<double>(j, 1) = -pose3d.at<double>(j, 1);
        invertPose.at<double>(j, 2) = -pose3d.at<double>(j, 2);
        //invertPose.at<double>(j, 3) = pose3d.at<double>(j, 3) * m_scale * 0.001f * 1.64f;
        invertPose.at<double>(j, 3) = pose3d.at<double>(j, 3);
    }*/

    cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);
    cvToGl.at<double>(0, 0) = 1.0f;
    cvToGl.at<double>(1, 1) = -1.0f; // Invert the y axis
    cvToGl.at<double>(2, 2) = -1.0f; // invert the z axis
    cvToGl.at<double>(3, 3) = 1.0f;

    pose3d = cvToGl * pose3d;

    // pose3d = invertPose;
}