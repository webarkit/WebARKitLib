#ifndef WEBARKITPATTERN_H
#define WEBARKITPATTERN_H

#include <opencv2/core/core.hpp>

struct WebARKitPattern {
    cv::Size size;

    //cv::Mat grayImg;

    //std::vector<cv::KeyPoint> keypoints;
    //cv::Mat descriptors;

    std::vector<cv::Point2f> points2d;
    std::vector<cv::Point3f> points3d;
};

/**
 * Intermediate pattern tracking info structure
 */
struct WebARKitPatternTrackingInfo
{
  cv::Mat                   homography;
  std::vector<cv::Point2f>  points2d;
  cv::Mat                   pose3d;

  /**
   * Compute pattern pose using PnP algorithm
   */
  void computePose(std::vector<cv::Point3f>& treeDPoints, std::vector<cv::Point2f>& imgPoints, cv::Mat& caMatrix, cv::Mat& distCoeffs);
};

#endif // WEBARKITPATTERN_H