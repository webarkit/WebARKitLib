#ifndef WEBARKITPATTERN_H
#define WEBARKITPATTERN_H

#include <opencv2/core/core.hpp>

struct WebARKitPattern {
    cv::Size size;

    // cv::Mat grayImg;

    // std::vector<cv::KeyPoint> keypoints;
    // cv::Mat descriptors;

    std::vector<cv::Point2f> points2d;
    std::vector<cv::Point3f> points3d;
};

/**
 * Intermediate pattern tracking info structure
 */
class WebARKitPatternTrackingInfo {
  public:
    WebARKitPatternTrackingInfo();

    cv::Mat homography;
    std::vector<cv::Point2f> points2d;
    cv::Mat pose3d;
    float transMat [3][4];
    float trans [3][4];
    cv::Mat glViewMatrix;

    void setScale(const float scale) { m_scale = scale; }

    float getScale() { return m_scale; }

    void cameraPoseFromPoints(cv::Mat& pose, const std::vector<cv::Point3f>& objPts, const std::vector<cv::Point2f>& imgPts, const cv::Matx33f& caMatrix, const cv::Mat& distCoeffs);

    /**
     * Compute pattern pose using PnP algorithm
     */
    void computePose(std::vector<cv::Point3f>& treeDPoints, std::vector<cv::Point2f>& imgPoints, const cv::Matx33f& caMatrix,
                     const cv::Mat& distCoeffs);

    void getTrackablePose(cv::Mat& pose);

    void updateTrackable();

    void computeGLviewMatrix();

    void computeGLviewMatrix(cv::Mat &pose);

  private:
    float m_scale;
    void invertPose();
};

#endif // WEBARKITPATTERN_H