#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>

namespace webarkit {

class WebARKitTracker::WebARKitTrackerImpl {
  public:
    WebARKitTrackerImpl() : corners(4), initialized(false), output(17, 0.0), _valid(false), numMatches(0){};
    ~WebARKitTrackerImpl() = default;

    void initialize(webarkit::TRACKER_TYPE trackerType) { setDetectorType(trackerType); }

    void initTracker(uchar* refData, size_t refCols, size_t refRows) {
        std::cout << "Init Tracker!" << std::endl;
        cv::Mat refGray(refRows, refCols, CV_8UC1, refData);

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
    std::vector<double> output; // 9 from homography matrix, 8 from warped corners*/

    cv::Ptr<cv::Feature2D> _featureDetector;

    cv::Ptr<cv::BFMatcher> _matcher;

    cv::Mat refGray, refDescr;

    std::vector<cv::KeyPoint> refKeyPts;

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

WebARKitTracker::~WebARKitTracker() = default; //destructor

WebARKitTracker::WebARKitTracker(WebARKitTracker&&) = default; //copy constructor

WebARKitTracker& WebARKitTracker::operator=(WebARKitTracker&&) = default; //move assignment operator

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