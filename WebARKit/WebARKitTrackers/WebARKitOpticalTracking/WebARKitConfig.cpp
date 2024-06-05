#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>

extern const double DEFAULT_NN_MATCH_RATIO = 0.7f;
extern const double TEBLID_NN_MATCH_RATIO = 0.8f;
extern const int DEFAULT_MAX_FEATURES = 800;
extern const int TEBLID_MAX_FEATURES = 1000;
extern const int N = 10;
extern const int MIN_NUM_MATCHES = 8;
extern const int minRequiredDetectedFeatures = 50; ///< Minimum number of detected features required to consider a target matched.
extern const int markerTemplateWidth = 15; ///< Width in pixels of image patches used in template matching.
extern const int maxLevel = 3; ///< Maximum number of levels in optical flow image pyramid.
extern const cv::Size winSize(31, 31);
extern const cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
extern const int searchRadius = 15;
extern const int match_method = cv::TM_SQDIFF_NORMED;
extern const cv::Size featureImageMinSize(640, 480); ///< Minimum size when downscaling incoming images used for feature tracking.
extern const double featureDetectPyramidLevel =
    1.05f; ///> Scale factor applied to image pyramid to determine image to perform feature matching upon.
extern const int featureBorder = 8;
extern const cv::Size blurSize(3, 3);
extern const double ransac_thresh = 2.5f; 
extern cv::RNG rng( 0xFFFFFFFF );
extern const double m_pi = 3.14159265358979323846;
extern const std::string WEBARKIT_HEADER_VERSION_STRING = "1.0.0";
/*@
    The MAJOR version number defines non-backwards compatible
    changes in the ARToolKit API. Range: [0-99].
 */
extern const int WEBARKIT_HEADER_VERSION_MAJOR = 1;

/*@
    The MINOR version number defines additions to the ARToolKit
    API, or (occsasionally) other significant backwards-compatible
    changes in runtime functionality. Range: [0-99].
 */
extern const int WEBARKIT_HEADER_VERSION_MINOR = 0;

/*@
    The TINY version number defines bug-fixes to existing
    functionality. Range: [0-99].
 */
extern const int WEBARKIT_HEADER_VERSION_TINY = 0;

/*@
    The BUILD version number will always be zero in releases,
    but may be non-zero in post-release development builds,
    version-control repository-sourced code, or other. Range: [0-99].
 */
extern const int WEBARKIT_HEADER_VERSION_DEV = 0;
