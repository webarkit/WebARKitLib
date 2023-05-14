#include <AR/ar.h>
#include <AR/config.h>
#include <WebARKit/WebARKitLog.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/HarrisDetector.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/TrackableInfo.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>

namespace webarkit {
//static int gCameraID = 0;

class WebARKitTracker::WebARKitTrackerImpl {
  public:
    WebARKitTrackerImpl()
        : corners(4), initialized(false), output(17, 0.0), _valid(false), numMatches(0)
        {
        _featureDetectorW = WebARKitFeatureDetector();
        _currentlyTrackedMarkers = 0;
        _frameSizeX = 0;
        _frameSizeY = 0;
        _K = cv::Mat();
        _distortionCoeff = cv::Mat();
    };

    ~WebARKitTrackerImpl() = default;

    void initialize(webarkit::TRACKER_TYPE trackerType) {
        //SetFeatureDetector(trackerType);
        setDetectorType(trackerType);
    }

    void initialize_w(webarkit::TRACKER_TYPE trackerType, size_t xsize, size_t ysize) {
        _frameSizeX = xsize;
        _frameSizeY = ysize;
        _maxNumberOfMarkersToTrack = 1;
        SetFeatureDetector(trackerType);
        setDetectorType(trackerType);
        _K = cv::Mat(3, 3, CV_64FC1);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                _K.at<double>(i, j) = (double)(m_param.mat[i][j]);
            }
        }
        std::cout << "dist function version: " << m_param.dist_function_version << std::endl;
        if (m_param.dist_function_version == 5) {
            // k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4.
            _distortionCoeff = cv::Mat::zeros(12, 1, CV_64F);
            for (int i = 0; i < 12; i++)
                _distortionCoeff.at<double>(i) = m_param.dist_factor[i];
        } else if (m_param.dist_function_version == 4) {
            _distortionCoeff = cv::Mat::zeros(5, 1, CV_64F);
            // k1,k2,p1,p2, and k3=0.
            for (int i = 0; i < 4; i++)
                _distortionCoeff.at<double>(i) = m_param.dist_factor[i];
            _distortionCoeff.at<double>(4) = 0.0;
        } else {
            ARLOGw("Unsupported camera parameters.\n");
        }
    }

    int loadARParam(std::string cparam_name, webarkit::TRACKER_TYPE trackerType, size_t xsize, size_t ysize) {
        // ARParam param;
        if (arParamLoad(cparam_name.c_str(), 1, &m_param) < 0) {
            ARLOGe("loadCamera(): Error loading parameter file %s for camera.", cparam_name.c_str());
            return -1;
        }
        //std::cout << "param xsize: " << m_param.xsize << std::endl;
        //std::cout << "dist function version: " << m_param.dist_function_version << std::endl;
        //int cameraID = gCameraID++;
        //cameraParams[cameraID] = m_param;

        initialize_w(trackerType, xsize, ysize);

        //initialize_w(trackerType, m_param.xsize, m_param.ysize);

        //return cameraID;
        return 1;
    }

    void AddMarker(uchar* buff, std::string fileName, int width, int height, int uid, float scale) {
        TrackableInfo newTrackable;
        // cv::Mat() wraps `buff` rather than copying it, but this is OK as we share ownership with caller via the
        // shared_ptr.
        // newTrackable._imageBuff = buff;
        newTrackable._image = cv::Mat(height, width, CV_8UC1, buff);
        if (!newTrackable._image.empty()) {
            newTrackable._id = uid;
            newTrackable._fileName = fileName;
            newTrackable._scale = scale;
            newTrackable._width = newTrackable._image.cols;
            newTrackable._height = newTrackable._image.rows;
            newTrackable._featurePoints = _featureDetectorW.DetectFeatures(newTrackable._image, cv::Mat());
            newTrackable._descriptors =
                _featureDetectorW.CalcDescriptors(newTrackable._image, newTrackable._featurePoints);
            newTrackable._cornerPoints = _harrisDetector.FindCorners(newTrackable._image);
            newTrackable._bBox.push_back(cv::Point2f(0, 0));
            newTrackable._bBox.push_back(cv::Point2f(newTrackable._width, 0));
            newTrackable._bBox.push_back(cv::Point2f(newTrackable._width, newTrackable._height));
            newTrackable._bBox.push_back(cv::Point2f(0, newTrackable._height));
            newTrackable._isTracking = false;
            newTrackable._isDetected = false;
            newTrackable._resetTracks = false;
            newTrackable._trackSelection = TrackingPointSelector(newTrackable._cornerPoints, newTrackable._width,
                                                                 newTrackable._height, markerTemplateWidth);

            _trackables.push_back(newTrackable);
            std::cout << "2D Marker added." << std::endl;
            // ARLOGi("2D marker added.\n");
        }
    }

    bool GetTrackablePose(int trackableId, float transMat[3][4]) {
        auto t = std::find_if(_trackables.begin(), _trackables.end(),
                              [&](const TrackableInfo& e) { return e._id == trackableId; });
        if (t != _trackables.end()) {
            cv::Mat poseOut;
            t->_pose.convertTo(poseOut, CV_32FC1);
            // std::cout << "poseOut" << std::endl;
            // std::cout << poseOut << std::endl;
            memcpy(transMat, poseOut.ptr<float>(0), 3 * 4 * sizeof(float));
            return true;
        }
        return false;
    }

    bool IsTrackableVisible(int trackableId) {
        auto t = std::find_if(_trackables.begin(), _trackables.end(),
                              [&](const TrackableInfo& e) { return e._id == trackableId; });
        if (t != _trackables.end()) {
            return (t->_isDetected || t->_isTracking);
        }
        return false;
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

    void ProcessFrameData_w(uchar* frame) {
        // Just wraps `frame` rather than copying it, i.e. `frame` must remain valid
        // for the duration of the call.
        cv::Mat newFrame(_frameSizeY, _frameSizeX, CV_8UC1, frame);
        ProcessFrame_w(newFrame);
        newFrame.release();
    }

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

    std::unordered_map<int, ARParam> cameraParams;

    ARParam m_param;

    std::vector<double> output; // 9 from homography matrix, 8 from warped corners*/

    WebARKitFeatureDetector _featureDetectorW;

    HarrisDetector _harrisDetector;

    cv::Ptr<cv::Feature2D> _featureDetector;

    cv::Ptr<cv::BFMatcher> _matcher;

    cv::Mat refGray, refDescr;

    std::vector<cv::KeyPoint> refKeyPts;

    int _frameCount;

    int _frameSizeX;

    int _frameSizeY;

    std::vector<cv::Mat> _pyramid, _prevPyramid;

    std::vector<TrackableInfo> _trackables;

    int _currentlyTrackedMarkers;

    cv::Mat _K;

    cv::Mat _distortionCoeff;

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
                std::cout << "trackable to match" << std::endl;
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
                std::cout << "New marker detected" << std::endl;
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

    std::vector<cv::Point2f> SelectTrackablePoints(int trackableIndex) {
        if (_trackables[trackableIndex]._resetTracks) {
            _trackables[trackableIndex]._trackSelection.SelectPoints();
            _trackables[trackableIndex]._resetTracks = false;
            return _trackables[trackableIndex]._trackSelection.GetSelectedFeatures();
        } else {
            return _trackables[trackableIndex]._trackSelection.GetTrackedFeatures();
        }
    }

    void RunOpticalFlow(int trackableId, std::vector<cv::Point2f> trackablePoints,
                        std::vector<cv::Point2f> trackablePointsWarped) {
        std::vector<cv::Point2f> flowResultPoints, trackablePointsWarpedResult;
        std::vector<uchar> statusFirstPass, statusSecondPass;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(_prevPyramid, _pyramid, trackablePointsWarped, flowResultPoints, statusFirstPass, err,
                                 winSize, 3, termcrit, 0, 0.001);
        cv::calcOpticalFlowPyrLK(_pyramid, _prevPyramid, flowResultPoints, trackablePointsWarpedResult,
                                 statusSecondPass, err, winSize, 3, termcrit, 0, 0.001);

        int killed1 = 0;
        std::vector<cv::Point2f> filteredTrackablePoints, filteredTrackedPoints;
        for (auto j = 0; j != flowResultPoints.size(); ++j) {
            if (!statusFirstPass[j] || !statusSecondPass[j]) {
                statusFirstPass[j] = (uchar)0;
                killed1++;
                continue;
            }
            filteredTrackablePoints.push_back(trackablePoints[j]);
            filteredTrackedPoints.push_back(flowResultPoints[j]);
        }
        if (UpdateTrackableHomography(trackableId, filteredTrackablePoints, filteredTrackedPoints)) {
            _trackables[trackableId]._isTracking = true;
        } else {
            _trackables[trackableId]._isDetected = false;
            _trackables[trackableId]._isTracking = false;
            _currentlyTrackedMarkers--;
        }
    }

    bool UpdateTrackableHomography(int trackableId, std::vector<cv::Point2f> matchedPoints1,
                                   std::vector<cv::Point2f> matchedPoints2) {
        if (matchedPoints1.size() > 4) {
            HomographyInfo homoInfo = GetHomographyInliers(matchedPoints1, matchedPoints2);
            if (homoInfo.validHomography) {
                _trackables[trackableId]._trackSelection.UpdatePointStatus(homoInfo.status);
                _trackables[trackableId]._trackSelection.SetHomography(homoInfo.homography);
                perspectiveTransform(_trackables[trackableId]._bBox, _trackables[trackableId]._bBoxTransformed,
                                     homoInfo.homography);
                if (_frameCount > 1) {
                    _trackables[trackableId]._resetTracks = true;
                }
                return true;
            }
        }
        return false;
    }

    std::vector<cv::Point2f> GetVerticesFromPoint(cv::Point ptOrig, int width = markerTemplateWidth,
                                                  int height = markerTemplateWidth) {
        std::vector<cv::Point2f> vertexPoints;
        vertexPoints.push_back(cv::Point2f(ptOrig.x - width / 2, ptOrig.y - height / 2));
        vertexPoints.push_back(cv::Point2f(ptOrig.x + width / 2, ptOrig.y - height / 2));
        vertexPoints.push_back(cv::Point2f(ptOrig.x + width / 2, ptOrig.y + height / 2));
        vertexPoints.push_back(cv::Point2f(ptOrig.x - width / 2, ptOrig.y + height / 2));
        return vertexPoints;
    }

    std::vector<cv::Point2f> GetVerticesFromTopCorner(int x, int y, int width, int height) {
        std::vector<cv::Point2f> vertexPoints;
        vertexPoints.push_back(cv::Point2f(x, y));
        vertexPoints.push_back(cv::Point2f(x + width, y));
        vertexPoints.push_back(cv::Point2f(x + width, y + height));
        vertexPoints.push_back(cv::Point2f(x, y + height));
        return vertexPoints;
    }

    cv::Rect GetTemplateRoi(cv::Point2f pt) {
        return cv::Rect(pt.x - (markerTemplateWidth / 2), pt.y - (markerTemplateWidth / 2), markerTemplateWidth,
                        markerTemplateWidth);
    }

    bool IsRoiValidForFrame(cv::Rect frameRoi, cv::Rect roi) { return (roi & frameRoi) == roi; }

    cv::Rect InflateRoi(cv::Rect roi, int inflationFactor) {
        cv::Rect newRoi = roi;
        newRoi.x -= inflationFactor;
        newRoi.y -= inflationFactor;
        newRoi.width += 2 * inflationFactor;
        newRoi.height += 2 * inflationFactor;
        return newRoi;
    }

    std::vector<cv::Point2f> FloorVertexPoints(std::vector<cv::Point2f> vertexPoints) {
        std::vector<cv::Point2f> testVertexPoints = vertexPoints;
        float minX = std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        for (int k = 0; k < testVertexPoints.size(); k++) {
            if (testVertexPoints[k].x < minX) {
                minX = testVertexPoints[k].x;
            }
            if (testVertexPoints[k].y < minY) {
                minY = testVertexPoints[k].y;
            }
        }
        for (int k = 0; k < testVertexPoints.size(); k++) {
            testVertexPoints[k].x -= minX;
            testVertexPoints[k].y -= minY;
        }
        return testVertexPoints;
    }

    cv::Mat MatchTemplateToImage(cv::Mat& searchImage, cv::Mat& warpedTemplate) {
        int result_cols = searchImage.cols - warpedTemplate.cols + 1;
        int result_rows = searchImage.rows - warpedTemplate.rows + 1;
        if (result_cols > 0 && result_rows > 0) {
            cv::Mat result;
            result.create(result_rows, result_cols, CV_32FC1);

            double minVal;
            double maxVal;
            minMaxLoc(warpedTemplate, &minVal, &maxVal, 0, 0, cv::Mat());

            cv::Mat normSeatchROI;
            normalize(searchImage, normSeatchROI, minVal, maxVal, cv::NORM_MINMAX, -1, cv::Mat());
            /// Do the Matching and Normalize
            matchTemplate(normSeatchROI, warpedTemplate, result, match_method);
            return result;
        } else {
            // std::cout << "Results image too small" << std::endl;
            return cv::Mat();
        }
    }

    void RunTemplateMatching(cv::Mat& frame, int trackableId) {
        // std::cout << "Starting template match" << std::endl;
        std::vector<cv::Point2f> finalTemplatePoints, finalTemplateMatchPoints;
        // Get a handle on the corresponding points from current image and the marker
        std::vector<cv::Point2f> trackablePoints = _trackables[trackableId]._trackSelection.GetTrackedFeatures();
        std::vector<cv::Point2f> trackablePointsWarped =
            _trackables[trackableId]._trackSelection.GetSelectedFeaturesWarped();
        // Create an empty result image - May be able to pre-initialize this container

        for (int j = 0; j < trackablePointsWarped.size(); j++) {
            auto pt = trackablePointsWarped[j];
            if (cv::pointPolygonTest(_trackables[trackableId]._bBoxTransformed, trackablePointsWarped[j], true) > 0) {
                auto ptOrig = trackablePoints[j];

                cv::Rect templateRoi = GetTemplateRoi(pt);
                cv::Rect frameROI(0, 0, frame.cols, frame.rows);
                if (IsRoiValidForFrame(frameROI, templateRoi)) {
                    cv::Rect markerRoi(0, 0, _trackables[trackableId]._image.cols,
                                       _trackables[trackableId]._image.rows);

                    std::vector<cv::Point2f> vertexPoints = GetVerticesFromPoint(ptOrig);
                    std::vector<cv::Point2f> vertexPointsResults;
                    perspectiveTransform(vertexPoints, vertexPointsResults,
                                         _trackables[trackableId]._trackSelection.GetHomography());

                    cv::Rect srcBoundingBox = cv::boundingRect(cv::Mat(vertexPointsResults));

                    vertexPoints.clear();
                    vertexPoints = GetVerticesFromTopCorner(srcBoundingBox.x, srcBoundingBox.y, srcBoundingBox.width,
                                                            srcBoundingBox.height);
                    perspectiveTransform(vertexPoints, vertexPointsResults,
                                         _trackables[trackableId]._trackSelection.GetHomography().inv());

                    std::vector<cv::Point2f> testVertexPoints = FloorVertexPoints(vertexPointsResults);
                    std::vector<cv::Point2f> finalWarpPoints =
                        GetVerticesFromTopCorner(0, 0, srcBoundingBox.width, srcBoundingBox.height);
                    cv::Mat templateHomography =
                        findHomography(testVertexPoints, finalWarpPoints, cv::RANSAC, ransac_thresh);

                    if (!templateHomography.empty()) {
                        cv::Rect templateBoundingBox = cv::boundingRect(cv::Mat(vertexPointsResults));
                        cv::Rect searchROI = InflateRoi(templateRoi, searchRadius);
                        if (IsRoiValidForFrame(frameROI, searchROI)) {
                            searchROI = searchROI & frameROI;
                            templateBoundingBox = templateBoundingBox & markerRoi;

                            if (templateBoundingBox.area() > 0 && searchROI.area() > templateBoundingBox.area()) {
                                cv::Mat searchImage = frame(searchROI);
                                cv::Mat templateImage = _trackables[trackableId]._image(templateBoundingBox);
                                cv::Mat warpedTemplate;

                                warpPerspective(templateImage, warpedTemplate, templateHomography,
                                                srcBoundingBox.size());
                                cv::Mat matchResult = MatchTemplateToImage(searchImage, warpedTemplate);

                                if (!matchResult.empty()) {
                                    double minVal;
                                    double maxVal;
                                    cv::Point minLoc, maxLoc, matchLoc;
                                    minMaxLoc(matchResult, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
                                    if (minVal < 0.5) {
                                        matchLoc = minLoc;
                                        matchLoc.x += searchROI.x + (warpedTemplate.cols / 2);
                                        matchLoc.y += searchROI.y + (warpedTemplate.rows / 2);
                                        finalTemplatePoints.push_back(ptOrig);
                                        finalTemplateMatchPoints.push_back(matchLoc);
                                    }
                                }
                            } else {
                                // std::cout << "ROIs not good" << std::endl;
                            }
                        }
                    } else {
                        // std::cout << "Empty homography" << std::endl;
                    }
                }
            }
        }
        if (!UpdateTrackableHomography(trackableId, finalTemplatePoints, finalTemplateMatchPoints)) {
            _trackables[trackableId]._isTracking = false;
            _trackables[trackableId]._isDetected = false;
            _currentlyTrackedMarkers--;
        }
    }

    void BuildImagePyramid(cv::Mat& frame) { cv::buildOpticalFlowPyramid(frame, _pyramid, winSize, maxLevel); }

    void SwapImagePyramid() { _pyramid.swap(_prevPyramid); }

    void ProcessFrame_w(cv::Mat& frame) {
        // std::cout << "Building pyramid" << std::endl;
        BuildImagePyramid(frame);
        // std::cout << "Drawing detected markers to mask" << std::endl;
        std::cout << "Currently tracked markers: " <<  _currentlyTrackedMarkers << std::endl;
        std::cout << "Max n. markers 2 track: " << _maxNumberOfMarkersToTrack << std::endl;
        if (CanDetectNewFeatures()) {
            std::cout << "Detecting new features" << std::endl;
            cv::Mat detectionFrame;
            cv::pyrDown(frame, detectionFrame,
                        cv::Size(frame.cols / featureDetectPyramidLevel, frame.rows / featureDetectPyramidLevel));
            cv::Mat featureMask = CreateFeatureMask(detectionFrame);
            std::vector<cv::KeyPoint> newFrameFeatures = _featureDetectorW.DetectFeatures(detectionFrame, featureMask);

            if (CanMatchNewFeatures(static_cast<int>(newFrameFeatures.size()))) {
                std::cout << "Matching new features" << std::endl;
                cv::Mat newFrameDescriptors = _featureDetectorW.CalcDescriptors(detectionFrame, newFrameFeatures);
                MatchFeatures(newFrameFeatures, newFrameDescriptors);
            }
        }
        if (_frameCount > 0) {
            if ((_currentlyTrackedMarkers > 0) && (_prevPyramid.size() > 0)) {
                std::cout << "Begin tracking phase" << std::endl;
                for (int i = 0; i < _trackables.size(); i++) {
                    if (_trackables[i]._isDetected) {
                        std::vector<cv::Point2f> trackablePoints = SelectTrackablePoints(i);
                        std::vector<cv::Point2f> trackablePointsWarped =
                            _trackables[i]._trackSelection.GetSelectedFeaturesWarped();
                        std::cout << "Starting Optical Flow" << std::endl;
                        RunOpticalFlow(i, trackablePoints, trackablePointsWarped);
                        if (_trackables[i]._isTracking) {
                            // Refine optical flow with template match.
                            RunTemplateMatching(frame, i);
                        }
                    }
                }
            }
        }
        for (auto&& t : _trackables) {
            if (t._isDetected || t._isTracking) {

                std::vector<cv::Point2f> imgPoints = t._trackSelection.GetSelectedFeaturesWarped();
                std::vector<cv::Point3f> objPoints = t._trackSelection.GetSelectedFeatures3d();

                CameraPoseFromPoints(t._pose, objPoints, imgPoints);
            }
        }
        SwapImagePyramid();
        _frameCount++;
    }

    void CameraPoseFromPoints(cv::Mat& pose, std::vector<cv::Point3f> objPts, std::vector<cv::Point2f> imgPts) {
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1); // output rotation vector
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1); // output translation vector

        // to compute the transformation matrix we need kc and dist coefficients from Camera !!
        cv::solvePnPRansac(objPts, imgPts, _K, _distortionCoeff, rvec, tvec);

        cv::Mat rMat;
        Rodrigues(rvec, rMat);
        cv::hconcat(rMat, tvec, pose);
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

void WebARKitTracker::initialize_w(webarkit::TRACKER_TYPE trackerType, size_t xsize, size_t ysize) {
    _trackerImpl->initialize_w(trackerType, xsize, ysize);
}

void WebARKitTracker::initTracker(uchar* refData, size_t refCols, size_t refRows) {
    _trackerImpl->initTracker(refData, refCols, refRows);
}

void WebARKitTracker::loadARParam(std::string paramName, webarkit::TRACKER_TYPE trackerType, size_t xsize, size_t ysize) {
    _trackerImpl->loadARParam(paramName, trackerType, xsize, ysize);
}

void WebARKitTracker::AddMarker(uchar* buff, std::string fileName, int width, int height, int uid, float scale) {
    _trackerImpl->AddMarker(buff, fileName, width, height, uid, scale);
}

bool WebARKitTracker::GetTrackablePose(int trackableId, float transMat[3][4]) {
    return _trackerImpl->GetTrackablePose(trackableId, transMat);
}

bool WebARKitTracker::IsTrackableVisible(int trackableId) { return _trackerImpl->IsTrackableVisible(trackableId); }

void WebARKitTracker::processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) {
    _trackerImpl->processFrameData(frameData, frameCols, frameRows, colorSpace);
}

void WebARKitTracker::ProcessFrameData_w(uchar* frameData) { _trackerImpl->ProcessFrameData_w(frameData); }

std::vector<double> WebARKitTracker::getOutputData() { return _trackerImpl->getOutputData(); }

bool WebARKitTracker::isValid() { return _trackerImpl->isValid(); }

} // namespace webarkit