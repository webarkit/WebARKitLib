#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>

extern const double GOOD_MATCH_RATIO = 0.7f;
extern const int MAX_FEATURES = 8000;
extern const int N = 10;
extern const int MIN_NUM_MATCHES = 20;
extern const int MAX_COUNT = 300;
extern const int minRequiredDetectedFeatures = 50;
extern const int featureDetectPyramidLevel = 2; ///> Scale factor applied to image pyramid to determine image to perform feature matching upon.
extern const double nn_match_ratio = 0.8f; ///< Nearest-neighbour matching ratio
extern const double ransac_thresh = 2.5f; ///< RANSAC inlier threshold
extern cv::RNG rng( 0xFFFFFFFF );
extern const int harrisBorder = 10;