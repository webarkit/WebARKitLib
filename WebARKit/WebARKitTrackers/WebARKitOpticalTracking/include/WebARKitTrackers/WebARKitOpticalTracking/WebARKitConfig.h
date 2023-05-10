#ifndef WEBARKIT_CONFIG_H
#define WEBARKIT_CONFIG_H

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

extern const double GOOD_MATCH_RATIO;
extern const int MAX_FEATURES;
extern const int N;
extern const int MIN_NUM_MATCHES;
extern const cv::Size winSize;
extern cv::TermCriteria termcrit;
extern const int MAX_COUNT;
extern const int minRequiredDetectedFeatures;
extern const int markerTemplateWidth;
extern const int featureDetectPyramidLevel;
extern const double nn_match_ratio; ///< Nearest-neighbour matching ratio
extern const double ransac_thresh;  ///< RANSAC inlier threshold
extern cv::RNG rng;
extern const int harrisBorder;
#endif
