#ifndef WEBARKIT_CONFIG_H
#define WEBARKIT_CONFIG_H

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

extern const double DEFAULT_NN_MATCH_RATIO;
extern const double TEBLID_NN_MATCH_RATIO;
extern const int DEFAULT_MAX_FEATURES;
extern const int TEBLID_MAX_FEATURES;
extern const int N;
extern const int MIN_NUM_MATCHES;
extern const int markerTemplateWidth; ///< Width in pixels of image patches used in template matching.
extern const int maxLevel;  ///< Maximum number of levels in optical flow image pyramid.
extern const cv::Size winSize;
extern const cv::TermCriteria termcrit;
extern const double featureDetectPyramidLevel; ///> Scale factor applied to image pyramid to determine image to perform feature matching upon.
extern const int featureBorder;
extern const cv::Size blurSize;
extern const double ransac_thresh;
extern cv::RNG rng;
extern const double m_pi;
extern const std::string WEBARKIT_HEADER_VERSION_STRING;
extern const int WEBARKIT_HEADER_VERSION_MAJOR;
extern const int WEBARKIT_HEADER_VERSION_MINOR;
extern const int WEBARKIT_HEADER_VERSION_TINY;
extern const int WEBARKIT_HEADER_VERSION_DEV;
#endif
