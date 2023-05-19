#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>

extern const double DEFAULT_NN_MATCH_RATIO = 0.7f;
extern const double TEBLID_NN_MATCH_RATIO = 0.8f;
extern const int DEFAULT_MAX_FEATURES = 8000;
extern const int TEBLID_MAX_FEATURES = 10000;
extern const int N = 10;
extern const int MIN_NUM_MATCHES = 50;
extern const int maxLevel = 3; ///< Maximum number of levels in optical flow image pyramid.
extern const cv::Size winSize(31,31);
extern const cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
extern const cv::Size blurSize(3, 3);