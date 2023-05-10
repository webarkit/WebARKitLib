#include <WebARKitTrackers/WebARKitOpticalTracking/HarrisDetector.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/TrackableInfo.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>

namespace webarkit {

class WebARKitTracker::WebARKitTrackerImpl {
  public:
    WebARKitTrackerImpl()
        : corners(4), initialized(false), output(17, 0.0), _valid(false), numMatches(0), _currentlyTrackedMarkers(0) {
        _featureDetectorW = WebARKitFeatureDetector();
        _maxNumberOfMarkersToTrack = 1;
    };

    ~WebARKitTrackerImpl() = default;

    void initialize(webarkit::TRACKER_TYPE trackerType) {
        SetFeatureDetector(trackerType);
        setDetectorType(trackerType);
    }

    void initTracker(uchar* refData, size_t refCols, size_t refRows) {
        std::cout << "Init Tracker!" << std::endl;
        cv::Mat refGray(refRows, refCols, CV_8UC1, refData);

        // Uncomment this line if you want to test the HarrisDetector class...
        // _harrisDetector.FindCorners(refGray);

        this->_featureDetector->detectAndCompute(refGray, cv::noArray(), refKeyPts, refDescr);

        corners[0] = cvPoint(0, 0);
        corners[1] = cvPoint(refCols, 0);
        corners[2] = cvPoint(refCols, refRows);
        corners[3] = cvPoint(0, refRows);

        initialized = true;

        std::cout << "Tracker ready!" << std::endl;
    };

    void processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) {
        cv::Mat grayFrame;
        if (colorSpace == ColorSpace::RGBA) {
            cv::Mat colorFrame(frameRows, frameCols, CV_8UC4, frameData);
            grayFrame.create(frameRows, frameCols, CV_8UC1);
            cv::cvtColor(colorFrame, grayFrame, cv::COLOR_RGBA2GRAY);
        } else if (colorSpace == ColorSpace::GRAY) {
            grayFrame = cv::Mat(frameRows, frameCols, CV_8UC1, frameData);
        }
        processFrame(grayFrame);
        grayFrame.release();
    };

    std::vector<double> getOutputData() { return output; };

    bool isValid() { return _valid; };

  protected:
    bool resetTracking(cv::Mat& currIm) {
        if (!initialized) {
            std::cout << "Reference image not found. AR is unintialized!" << std::endl;
            return NULL;
        }

        std::cout << "Reset Tracking!" << std::endl;

        clear_output();

        cv::Mat frameDescr;
        std::vector<cv::KeyPoint> frameKeyPts;

        this->_featureDetector->detectAndCompute(currIm, cv::noArray(), frameKeyPts, frameDescr);

        std::vector<std::vector<cv::DMatch>> knnMatches;
        _matcher->knnMatch(frameDescr, refDescr, knnMatches, 2);

        framePts.clear();
        std::vector<cv::Point2f> refPts;

        // find the best matches
        for (size_t i = 0; i < knnMatches.size(); ++i) {
            if (knnMatches[i][0].distance < GOOD_MATCH_RATIO * knnMatches[i][1].distance) {
                framePts.push_back(frameKeyPts[knnMatches[i][0].queryIdx].pt);
                refPts.push_back(refKeyPts[knnMatches[i][0].trainIdx].pt);
            }
        }

        bool valid;

        if (framePts.size() >= MIN_NUM_MATCHES) {
            m_H = cv::findHomography(refPts, framePts, cv::RANSAC);
            if ((valid = homographyValid(m_H))) {
                numMatches = framePts.size();

                if (currIm.empty()) {
                    std::cout << "prevIm is empty!" << std::endl;
                    return NULL;
                }
                currIm.copyTo(prevIm);
            }
        }

        return valid;
    };

    bool track(cv::Mat& currIm) {
        if (!initialized) {
            std::cout << "Reference image not found. AR is unintialized!" << std::endl;
            return NULL;
        }

        if (prevIm.empty()) {
            std::cout << "Tracking is uninitialized!" << std::endl;
            return NULL;
        }

        std::cout << "Start tracking!" << std::endl;
        clear_output();

        // use optical flow to track keypoints
        std::vector<float> err;
        std::vector<uchar> status;
        std::vector<cv::Point2f> currPts, goodPtsCurr, goodPtsPrev;
        bool valid;
        calcOpticalFlowPyrLK(prevIm, currIm, framePts, currPts, status, err);

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
            if ((valid = homographyValid(m_H))) {
                fill_output(m_H);
            }
        }

        currIm.copyTo(prevIm);

        return valid;
    };

    void processFrame(cv::Mat& frame) {
        if (!this->_valid) {
            this->_valid = resetTracking(frame);
        } else {
            this->_valid = track(frame);
        }
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

    void clear_output() { output = std::vector<double>(17, 0.0); };

    bool _valid;

    std::vector<cv::Point2f> corners;

    cv::Mat m_H;

    cv::Mat prevIm;

    int numMatches;

    std::vector<cv::Point2f> framePts;

    bool initialized;

  private:
    int _maxNumberOfMarkersToTrack;

    std::vector<double> output; // 9 from homography matrix, 8 from warped corners*/

    WebARKitFeatureDetector _featureDetectorW;

    HarrisDetector _harrisDetector;

    cv::Ptr<cv::Feature2D> _featureDetector;

    cv::Ptr<cv::BFMatcher> _matcher;

    cv::Mat refGray, refDescr;

    std::vector<cv::KeyPoint> refKeyPts;

    std::vector<TrackableInfo> _trackables;

    int _currentlyTrackedMarkers;

    webarkit::TRACKER_TYPE _selectedFeatureDetectorType;

    cv::Mat CreateFeatureMask(cv::Mat frame) {
        cv::Mat featureMask;
        for (int i = 0; i < _trackables.size(); i++) {
            if (_trackables[i]._isDetected) {
                if (featureMask.empty()) {
                    // Only create mask if we have something to draw in it.
                    featureMask = cv::Mat::ones(frame.size(), CV_8UC1);
                }
                std::vector<std::vector<cv::Point>> contours(1);
                for (int j = 0; j < 4; j++) {
                    contours[0].push_back(cv::Point(_trackables[i]._bBoxTransformed[j].x / featureDetectPyramidLevel,
                                                    _trackables[i]._bBoxTransformed[j].y / featureDetectPyramidLevel));
                }
                drawContours(featureMask, contours, 0, cv::Scalar(0), -1, 8);
            }
        }
        return featureMask;
    }

    bool CanDetectNewFeatures() { return (_currentlyTrackedMarkers < _maxNumberOfMarkersToTrack); }

    bool CanMatchNewFeatures(int detectedFeaturesSize) { return (detectedFeaturesSize > minRequiredDetectedFeatures); }

    void MatchFeatures(std::vector<cv::KeyPoint> newFrameFeatures, cv::Mat newFrameDescriptors) {
        int maxMatches = 0;
        int bestMatchIndex = -1;
        std::vector<cv::KeyPoint> finalMatched1, finalMatched2;
        for (int i = 0; i < _trackables.size(); i++) {
            if (!_trackables[i]._isDetected) {
                std::vector<std::vector<cv::DMatch>> matches =
                    _featureDetectorW.MatchFeatures(newFrameDescriptors, _trackables[i]._descriptors);
                if (matches.size() > minRequiredDetectedFeatures) {
                    std::vector<cv::KeyPoint> matched1, matched2;
                    std::vector<uchar> status;
                    int totalGoodMatches = 0;
                    for (unsigned int j = 0; j < matches.size(); j++) {
                        // Ratio Test for outlier removal, removes ambiguous matches.
                        if (matches[j][0].distance < nn_match_ratio * matches[j][1].distance) {
                            matched1.push_back(newFrameFeatures[matches[j][0].queryIdx]);
                            matched2.push_back(_trackables[i]._featurePoints[matches[j][0].trainIdx]);
                            status.push_back(1);
                            totalGoodMatches++;
                        } else {
                            status.push_back(0);
                        }
                    }
                    if (totalGoodMatches > maxMatches) {
                        finalMatched1 = matched1;
                        finalMatched2 = matched2;
                        maxMatches = totalGoodMatches;
                        bestMatchIndex = i;
                    }
                }
            }
        }

        if (maxMatches > 0) {
            for (int i = 0; i < finalMatched1.size(); i++) {
                finalMatched1[i].pt.x *= featureDetectPyramidLevel;
                finalMatched1[i].pt.y *= featureDetectPyramidLevel;
            }

            HomographyInfo homoInfo = GetHomographyInliers(Points(finalMatched2), Points(finalMatched1));
            if (homoInfo.validHomography) {
                // std::cout << "New marker detected" << std::endl;
                _trackables[bestMatchIndex]._trackSelection.SelectPoints();
                _trackables[bestMatchIndex]._trackSelection.SetHomography(homoInfo.homography);
                _trackables[bestMatchIndex]._isDetected = true;
                _trackables[bestMatchIndex]._resetTracks = true;

                perspectiveTransform(_trackables[bestMatchIndex]._bBox, _trackables[bestMatchIndex]._bBoxTransformed,
                                     homoInfo.homography);
                _currentlyTrackedMarkers++;
            }
        }
    }

    void SetFeatureDetector(webarkit::TRACKER_TYPE trackerType) {
        _selectedFeatureDetectorType = trackerType;
        _featureDetectorW.SetFeatureDetector(trackerType);
    }

    void setDetectorType(webarkit::TRACKER_TYPE trackerType) {
        if (trackerType == webarkit::TRACKER_TYPE::AKAZE_TRACKER) {
            this->_featureDetector = cv::AKAZE::create();
        } else if (trackerType == webarkit::TRACKER_TYPE::ORB_TRACKER) {
            this->_featureDetector = cv::ORB::create(MAX_FEATURES);
        }
        _matcher = cv::BFMatcher::create();
    };
};

WebARKitTracker::WebARKitTracker() : _trackerImpl(new WebARKitTrackerImpl()) {}

WebARKitTracker::~WebARKitTracker() = default; // destructor

WebARKitTracker::WebARKitTracker(WebARKitTracker&&) = default; // copy constructor

WebARKitTracker& WebARKitTracker::operator=(WebARKitTracker&&) = default; // move assignment operator

void WebARKitTracker::initialize(webarkit::TRACKER_TYPE trackerType) { _trackerImpl->initialize(trackerType); }

void WebARKitTracker::initTracker(uchar* refData, size_t refCols, size_t refRows) {
    _trackerImpl->initTracker(refData, refCols, refRows);
}

void WebARKitTracker::processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) {
    _trackerImpl->processFrameData(frameData, frameCols, frameRows, colorSpace);
}

std::vector<double> WebARKitTracker::getOutputData() { return _trackerImpl->getOutputData(); }

bool WebARKitTracker::isValid() { return _trackerImpl->isValid(); }

} // namespace webarkit