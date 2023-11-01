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

#endif // WEBARKITPATTERN_H