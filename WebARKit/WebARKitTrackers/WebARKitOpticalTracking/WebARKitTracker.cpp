#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>

namespace webarkit {

class WebARKitTracker::WebARKitTrackerImpl {
  public:
    WebARKitTrackerImpl()
        : corners(4), initialized(false), output(17, 0.0), _valid(false), _isDetected(false), numMatches(0),
          minNumMatches(MIN_NUM_MATCHES), _nn_match_ratio(0.7f) {
        m_camMatrix = cv::Matx33d::zeros();
        m_distortionCoeff = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
    };

    ~WebARKitTrackerImpl() = default;

    void initialize(webarkit::TRACKER_TYPE trackerType, int frameWidth, int frameHeight) {
        setDetectorType(trackerType);
        if (trackerType == webarkit::TEBLID_TRACKER) {
            _nn_match_ratio = TEBLID_NN_MATCH_RATIO;
        } else if (trackerType == webarkit::AKAZE_TRACKER) {
            _nn_match_ratio = DEFAULT_NN_MATCH_RATIO;
            minNumMatches = 40;
        } else {
            _nn_match_ratio = DEFAULT_NN_MATCH_RATIO;
            minNumMatches = 15;
        }
        WEBARKIT_LOGi("Min Num Matches: %d\n", minNumMatches);
        _camera->setupCamera(frameWidth, frameHeight);
        _camera->printSettings();

        std::array<double, 9> camData = _camera->getCameraData();
        for (auto i = 0; i < 3; i++) {
            for (auto j = 0; j < 3; j++) {
                m_camMatrix(i, j) = camData[i * 3 + j];
            }
        }

        for (auto i = 0; i < 3; i++) {
            for (auto j = 0; j < 3; j++) {
                WEBARKIT_LOGi("Camera Matrix: %.2f\n", m_camMatrix(i, j));
            }
        }

        for (auto i = 0; i < 4; i++) {
            WEBARKIT_LOGi("Distortion coefficients: %.2f\n", m_distortionCoeff.at<double>(i, 0));
        }

        webarkit::cameraProjectionMatrix(camData, 0.1, 1000.0, frameWidth, frameHeight, m_cameraProjectionMatrix);
    }

    template<typename T>
    void initTracker(T refData, size_t refCols, size_t refRows, ColorSpace colorSpace) {
        WEBARKIT_LOGi("Init Tracker!\n");

        cv::Mat refGray = convert2Grayscale(refData, refCols, refRows, colorSpace);

        cv::Mat trackerFeatureMask = createTrackerFeatureMask(refGray);

        if(!extractFeatures(refGray, trackerFeatureMask, refKeyPts, refDescr)) {
            WEBARKIT_LOGe("No features detected!\n");
            return;
        };

        // Normalized dimensions :
        const float maxSize = std::max(refCols, refRows);
        const float unitW = refCols / maxSize;
        const float unitH = refRows / maxSize;

        _pattern.size = cv::Size(refCols, refRows);

        WEBARKIT_LOGd("WebARKitPattern size ready!\n");

        _pattern.points2d.push_back(cv::Point2f(0, 0));
        _pattern.points2d.push_back(cv::Point2f(refCols, 0));
        _pattern.points2d.push_back(cv::Point2f(refCols, refRows));
        _pattern.points2d.push_back(cv::Point2f(0, refRows));

        WEBARKIT_LOGd("WebARKitPattern points2d ready!\n");

        _pattern.points3d.push_back(cv::Point3f(-unitW, -unitH, 0));
        _pattern.points3d.push_back(cv::Point3f(unitW, -unitH, 0));
        _pattern.points3d.push_back(cv::Point3f(unitW, unitH, 0));
        _pattern.points3d.push_back(cv::Point3f(-unitW, unitH, 0));

        WEBARKIT_LOGd("WebARKitPattern points3d ready!\n");

        corners[0] = cvPoint(0, 0);
        corners[1] = cvPoint(refCols, 0);
        corners[2] = cvPoint(refCols, refRows);
        corners[3] = cvPoint(0, refRows);

        _bBox.push_back(cv::Point2f(0, 0));
        _bBox.push_back(cv::Point2f(refCols, 0));
        _bBox.push_back(cv::Point2f(refCols, refRows));
        _bBox.push_back(cv::Point2f(0, refRows));

        initialized = true;

        WEBARKIT_LOGi("Tracker ready!\n");
    }

    bool extractFeatures(const cv::Mat& grayImage,  cv::Mat& featureMask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const {
        assert(!grayImage.empty());
        assert(grayImage.channels() == 1);

        this->_featureDetector->detect(grayImage, keypoints, featureMask);
        if (keypoints.empty()){
            WEBARKIT_LOGe("No keypoints detected!\n");
            return false;
        }
        this->_featureDescriptor->compute(grayImage, keypoints, descriptors);
        if (descriptors.empty()) {
            WEBARKIT_LOGe("No descriptors computed!\n");
            return false;
        }
        return true;
   }

    void processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) {
        cv::Mat grayFrame = convert2Grayscale(frameData, frameCols, frameRows, colorSpace);
        cv::blur(grayFrame, grayFrame, blurSize);
        buildImagePyramid(grayFrame);
        processFrame(grayFrame);
        grayFrame.release();
    };

    std::vector<double> getOutputData() { return output; };

    cv::Mat getPoseMatrix() { return _patternTrackingInfo.pose3d; };

    cv::Mat getGLViewMatrix() { return _patternTrackingInfo.glViewMatrix; };

    std::array<double, 16> getCameraProjectionMatrix() { return m_cameraProjectionMatrix; };

    bool isValid() { return _valid; };

  protected:
    bool resetTracking(cv::Mat& currIm) {
        if (!initialized) {
            WEBARKIT_LOGe("Reference image not found. AR is unintialized!\n");
            return NULL;
        }

        WEBARKIT_LOGi("Reset Tracking!\n");

        clear_output();

        _isDetected = false;

        cv::Mat frameDescr;
        std::vector<cv::KeyPoint> frameKeyPts;
        bool valid;

        cv::Mat featureMask = createFeatureMask(currIm);

        if(!extractFeatures(currIm, featureMask, frameKeyPts, frameDescr)) {
            WEBARKIT_LOGe("No features detected in resetTracking!\n");
            return false;
        };

        if (!_isDetected) {
            std::vector<std::vector<cv::DMatch>> knnMatches;
            _matcher->knnMatch(frameDescr, refDescr, knnMatches, 2);

            framePts.clear();
            std::vector<cv::Point2f> refPts;

            // find the best matches
            for (size_t i = 0; i < knnMatches.size(); ++i) {
                if (knnMatches[i][0].distance < _nn_match_ratio * knnMatches[i][1].distance) {
                    framePts.push_back(frameKeyPts[knnMatches[i][0].queryIdx].pt);
                    refPts.push_back(refKeyPts[knnMatches[i][0].trainIdx].pt);
                }
            }

            WEBARKIT_LOG("Num Matches: %zu\n", framePts.size());

            if (framePts.size() >= minNumMatches) {
                m_H = cv::findHomography(refPts, framePts, cv::RANSAC);
                if ((valid = homographyValid(m_H))) {
                    _isDetected = true;
                    numMatches = framePts.size();
                    perspectiveTransform(_bBox, _bBoxTransformed, m_H);
                }
            }
        }

        return valid;
    };

    bool track() {
        if (!initialized) {
            WEBARKIT_LOGe("Reference image not found. AR is unintialized!\n");
            return NULL;
        }

        WEBARKIT_LOGi("Start tracking!\n");
        clear_output();

        // use optical flow to track keypoints
        std::vector<float> err;
        std::vector<uchar> status;
        std::vector<cv::Point2f> currPts, goodPtsCurr, goodPtsPrev;
        bool valid;
        calcOpticalFlowPyrLK(_prevPyramid, _pyramid, framePts, currPts, status, err, winSize, maxLevel, termcrit, 0,
                             0.001);

        // calculate average variance
        double mean, avg_variance = 0.0;
        double sum = 0.0;
        double diff;
        std::vector<double> diffs;
        for (size_t i = 0; i < framePts.size(); ++i) {
            if (status[i]) {
                goodPtsCurr.push_back(currPts[i]);
                goodPtsPrev.push_back(framePts[i]);

                diff = sqrt(pow(currPts[i].x - framePts[i].x, 2.0) + pow(currPts[i].y - framePts[i].y, 2.0));
                sum += diff;
                diffs.push_back(diff);
            }
        }

        mean = sum / diffs.size();
        for (int i = 0; i < goodPtsCurr.size(); ++i) {
            avg_variance += pow(diffs[i] - mean, 2);
        }
        avg_variance /= diffs.size();

        if ((goodPtsCurr.size() > numMatches / 2) && (1.75 > avg_variance)) {
            cv::Mat transform = estimateAffine2D(goodPtsPrev, goodPtsCurr);

            // add row of {0,0,1} to transform to make it 3x3
            cv::Mat row = cv::Mat::zeros(1, 3, CV_64F);
            row.at<double>(0, 2) = 1.0;
            transform.push_back(row);

            // update homography matrix
            m_H = transform * m_H;

            // set old points to new points
            framePts = goodPtsCurr;
            std::vector<cv::Point2f> warpedCorners;

            if ((valid = homographyValid(m_H))) {
                fill_output(m_H);

                warpedCorners = getSelectedFeaturesWarped(m_H);

                _patternTrackingInfo.computePose(_pattern.points3d, warpedCorners, m_camMatrix, m_distortionCoeff);

                _patternTrackingInfo.computeGLviewMatrix();

                _isDetected = true;
            } else {
                _isDetected = false;
            }
        }

        //swapImagePyramid();

        return valid;
    };

    void processFrame(cv::Mat& frame) {
        if (!this->_valid) {
            this->_valid = resetTracking(frame);
        } else {
            this->_valid = track();
        }
        WEBARKIT_LOG("Marker detected : %s\n", _isDetected ? "true" : "false");
        swapImagePyramid();
    };

    bool homographyValid(cv::Mat& H) {
        if (H.empty()) {
            return false;
        }
        const double det = H.at<double>(0, 0) * H.at<double>(1, 1) - H.at<double>(1, 0) * H.at<double>(0, 1);
        return 1 / N < fabs(det) && fabs(det) < N;
    };

    void fill_output(cv::Mat& H) {
        std::vector<cv::Point2f> warped(4);
        cv::perspectiveTransform(corners, warped, H);

        output[0] = H.at<double>(0, 0);
        output[1] = H.at<double>(0, 1);
        output[2] = H.at<double>(0, 2);
        output[3] = H.at<double>(1, 0);
        output[4] = H.at<double>(1, 1);
        output[5] = H.at<double>(1, 2);
        output[6] = H.at<double>(2, 0);
        output[7] = H.at<double>(2, 1);
        output[8] = H.at<double>(2, 2);

        output[9] = warped[0].x;
        output[10] = warped[0].y;
        output[11] = warped[1].x;
        output[12] = warped[1].y;
        output[13] = warped[2].x;
        output[14] = warped[2].y;
        output[15] = warped[3].x;
        output[16] = warped[3].y;
    };

    void clear_output() {
        output = std::vector<double>(17, 0.0);
    };

    void buildImagePyramid(cv::Mat& frame) { cv::buildOpticalFlowPyramid(frame, _pyramid, winSize, maxLevel); }

    void swapImagePyramid() { _pyramid.swap(_prevPyramid); }

    cv::Mat createTrackerFeatureMask(cv::Mat& frame) {
        cv::Mat featureMask;
        if (featureMask.empty()) {
            // Only create mask if we have something to draw in it.
            featureMask = cv::Mat::zeros(frame.size(), frame.type());
        }
        cv::Rect innerRegion(featureBorder, featureBorder, frame.cols - (featureBorder * 2),
                             frame.rows - (featureBorder * 2));
        cv::Mat maskRoi(featureMask, innerRegion);
        maskRoi.setTo(cv::Scalar(255));
        return featureMask;
    }

    cv::Mat createFeatureMask(cv::Mat& frame) {
        cv::Mat featureMask;
        if (_isDetected) {
            if (featureMask.empty()) {
                // Only create mask if we have something to draw in it.
                featureMask = cv::Mat::ones(frame.size(), frame.type());
            }
            std::vector<std::vector<cv::Point>> contours(1);
            for (int j = 0; j < 4; j++) {
                contours[0].push_back(cv::Point(_bBoxTransformed[j].x / featureDetectPyramidLevel,
                                                _bBoxTransformed[j].y / featureDetectPyramidLevel));
            }
            drawContours(featureMask, contours, 0, cv::Scalar(0), -1, 8);
        }
        return featureMask;
    }

    std::vector<cv::Point2f> getSelectedFeaturesWarped(cv::Mat& H) {
        std::vector<cv::Point2f> warpedPoints;
        perspectiveTransform(_pattern.points2d, warpedPoints, H);
        return warpedPoints;
    }

    bool _valid;

    bool _isDetected;

    std::vector<cv::Point2f> corners;

    cv::Mat m_H;

    cv::Mat prevIm;

    int numMatches;

    int minNumMatches;

    std::vector<cv::Point2f> framePts;

    bool initialized;

    WebARKitCamera* _camera = new WebARKitCamera();

    WebARKitPattern _pattern;

    WebARKitPatternTrackingInfo _patternTrackingInfo;

    cv::Matx33d m_camMatrix;
    cv::Mat m_distortionCoeff;

    std::array<double, 16> m_cameraProjectionMatrix;

  private:
    std::vector<double> output; // 9 from homography matrix, 8 from warped corners*/

    cv::Ptr<cv::Feature2D> _featureDetector;

    cv::Ptr<cv::Feature2D> _featureDescriptor;

    cv::Ptr<cv::BFMatcher> _matcher;

    cv::Mat refGray, refDescr;

    std::vector<cv::KeyPoint> refKeyPts;

    webarkit::TRACKER_TYPE _trackerType;

    double _nn_match_ratio;

    std::vector<cv::Mat> _pyramid, _prevPyramid;

    std::vector<cv::Point2f> _bBoxTransformed;

    std::vector<cv::Point2f> _bBox;

    void setDetectorType(webarkit::TRACKER_TYPE trackerType) {
        _trackerType = trackerType;
        if (trackerType == webarkit::TRACKER_TYPE::AKAZE_TRACKER) {
            this->_featureDetector = cv::AKAZE::create();
            this->_featureDescriptor = cv::AKAZE::create();
        } else if (trackerType == webarkit::TRACKER_TYPE::ORB_TRACKER) {
            this->_featureDetector = cv::ORB::create(DEFAULT_MAX_FEATURES);
            this->_featureDescriptor = cv::ORB::create(DEFAULT_MAX_FEATURES);
        } else if (trackerType == webarkit::TRACKER_TYPE::FREAK_TRACKER) {
            this->_featureDetector = cv::ORB::create(10000);
            // this->_featureDetector = cv::xfeatures2d::StarDetector::create(DEFAULT_MAX_FEATURES);
            this->_featureDescriptor = cv::xfeatures2d::FREAK::create();
        } else if (trackerType == webarkit::TRACKER_TYPE::TEBLID_TRACKER) {
            this->_featureDetector = cv::ORB::create(TEBLID_MAX_FEATURES);
            this->_featureDescriptor = cv::xfeatures2d::TEBLID::create(1.00f);
        }
        _matcher = cv::BFMatcher::create();
    };
};

WebARKitTracker::WebARKitTracker() : _trackerImpl(new WebARKitTrackerImpl()) {}

WebARKitTracker::~WebARKitTracker() = default; // destructor

WebARKitTracker::WebARKitTracker(WebARKitTracker&&) = default; // copy constructor

WebARKitTracker& WebARKitTracker::operator=(WebARKitTracker&&) = default; // move assignment operator

void WebARKitTracker::initialize(webarkit::TRACKER_TYPE trackerType, int frameWidth, int frameHeight) {
    _trackerImpl->initialize(trackerType, frameWidth, frameHeight);
}

void WebARKitTracker::initTracker(cv::Mat refData, size_t refCols, size_t refRows, ColorSpace colorSpace) {
    _trackerImpl->initTracker<cv::Mat>(refData, refCols, refRows, colorSpace);
}

void WebARKitTracker::initTracker(uchar* refData, size_t refCols, size_t refRows, ColorSpace colorSpace) {
    _trackerImpl->initTracker<uchar*>(refData, refCols, refRows, colorSpace);
}

void WebARKitTracker::processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) {
    _trackerImpl->processFrameData(frameData, frameCols, frameRows, colorSpace);
}

std::vector<double> WebARKitTracker::getOutputData() { return _trackerImpl->getOutputData(); }

cv::Mat WebARKitTracker::getPoseMatrix() { return _trackerImpl->getPoseMatrix(); }

cv::Mat WebARKitTracker::getGLViewMatrix() { return _trackerImpl->getGLViewMatrix(); }

std::array<double, 16> WebARKitTracker::getCameraProjectionMatrix() {
    return _trackerImpl->getCameraProjectionMatrix();
}

bool WebARKitTracker::isValid() { return _trackerImpl->isValid(); }

} // namespace webarkit