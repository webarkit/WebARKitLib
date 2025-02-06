#include <WebARKitTrackers/WebARKitOpticalTracking/TrackerVisualization.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/TrackingPointSelector.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitHomographyInfo.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>


namespace webarkit {

class WebARKitTracker::WebARKitTrackerImpl {
  public:
    bool _trackVizActive;
    TrackerVisualization _trackViz;

    WebARKitTrackerImpl()
        : corners(4), initialized(false), output(17, 0.0), _valid(false), _maxNumberOfMarkersToTrack(1),
          _currentlyTrackedMarkers(0), _frameCount(0),  _frameSizeX(0),
        _frameSizeY(0),
        _featureDetectPyrLevel(0),
        _featureDetectScaleFactor(cv::Vec2f(1.0f, 1.0f)),
        _isDetected(false), _isTracking(false), numMatches(0),
          minNumMatches(MIN_NUM_MATCHES), _nn_match_ratio(0.7f), _trackVizActive(false),
          _trackViz(TrackerVisualization()) { 
        m_camMatrix = cv::Matx33d::zeros();
        m_distortionCoeff = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
    };

    ~WebARKitTrackerImpl() = default;

    void initialize(webarkit::TRACKER_TYPE trackerType, int frameWidth, int frameHeight) {
        _frameSizeX = frameWidth;
        _frameSizeY = frameHeight;

        // Calculate image downsamping factor. 0 = no size change, 1 = half width and height, 2 = quarter width and height etc.
        double xmin_log2 = std::log2(static_cast<double>(featureImageMinSize.width));
        double ymin_log2 = std::log2(static_cast<double>(featureImageMinSize.height));
        _featureDetectPyrLevel = std::min(std::floor(std::log2(static_cast<double>(_frameSizeX)) - xmin_log2), std::floor(std::log2(static_cast<double>(_frameSizeY)) - ymin_log2));
        
        // Calculate the exact scale factor using the same calculation pyrDown uses.
        int xScaled = _frameSizeX;
        int yScaled = _frameSizeY;
        for (int i = 1; i <= _featureDetectPyrLevel; i++) {
            xScaled = (xScaled + 1) / 2;
            yScaled = (yScaled + 1) / 2;
            _featureDetectScaleFactor = cv::Vec2f((float)_frameSizeX / (float)xScaled, (float)_frameSizeY / (float)yScaled);
        }

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
        WEBARKIT_LOGd("Min Num Matches: %d\n", minNumMatches);
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

        for (auto i = 0; i < 16; i++) {

                WEBARKIT_LOGi("Camera Proj Matrix: %.2f\n", m_cameraProjectionMatrix[i]);

        }

        //1.9102363924347978, 0, 0, 0, 0, 2.5377457054523322, 0, 0, -0.013943280545895442, -0.005830389685211879, -1.0000002000000199, -1, 0, 0, -0.00020000002000000202, 0

        _pyramid.clear();
        _prevPyramid.clear();
        _currentlyTrackedMarkers = 0;
    }

    template <typename T> void initTracker(T refData, size_t refCols, size_t refRows, ColorSpace colorSpace) {
        WEBARKIT_LOGi("Init Tracker!\n");

        cv::Mat refGray = convert2Grayscale(refData, refCols, refRows, colorSpace);
        refGray.copyTo(_image);

        cv::Mat trackerFeatureMask = createTrackerFeatureMask(refGray);

        if (!extractFeatures(refGray, trackerFeatureMask, refKeyPts, refDescr)) {
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

        _trackSelection = TrackingPointSelector(Points(refKeyPts), refCols, refRows, markerTemplateWidth);

        initialized = true;

        WEBARKIT_LOGi("Tracker ready!\n");
    }

    bool extractFeatures(const cv::Mat& grayImage, cv::Mat& featureMask, std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors) const {
        assert(!grayImage.empty());
        assert(grayImage.channels() == 1);

        this->_featureDetector->detect(grayImage, keypoints, featureMask);
        WEBARKIT_LOGd("keypoints size: %d\n", keypoints.size());
        if (keypoints.empty()) {
            WEBARKIT_LOGe("No keypoints detected!\n");
            return false;
        }
        this->_featureDescriptor->compute(grayImage, keypoints, descriptors);
        WEBARKIT_LOGd("descriptors size: %d\n", descriptors.size());
        if (descriptors.empty()) {
            WEBARKIT_LOGe("No descriptors computed!\n");
            return false;
        }
        return true;
    }

    void processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace,
                          BLUR_TYPE blurType) {
        cv::Mat grayFrame = convert2Grayscale(frameData, frameCols, frameRows, colorSpace);
        if (blurType == BLUR_TYPE::BOX_BLUR) {
            cv::blur(grayFrame, grayFrame, blurSize);
        }
        else if (blurType == BLUR_TYPE::MEDIAN_BLUR) {
            cv::medianBlur(grayFrame, grayFrame, blurSize.width);
        }
        processFrame(grayFrame);
        grayFrame.release();
    };

    std::vector<double> getOutputData() { return output; };

    cv::Mat getPoseMatrix() { return _patternTrackingInfo.pose3d; };

    float* getPoseMatrix2() { return (float*)_patternTrackingInfo.trans; }

    cv::Mat getGLViewMatrix() { return _patternTrackingInfo.glViewMatrix; };

    std::array<double, 16> getCameraProjectionMatrix() { return m_cameraProjectionMatrix; };

    bool isValid() { return _valid; };

  protected:
    bool RunTemplateMatching(cv::Mat frame, int trackableId) {
        // std::cout << "Starting template match" << std::endl;
        std::vector<cv::Point2f> finalTemplatePoints, finalTemplateMatchPoints;
        // Get a handle on the corresponding points from current image and the marker
        // std::vector<cv::Point2f> trackablePoints = _trackables[trackableId]._trackSelection.GetTrackedFeatures();
        // std::vector<cv::Point2f> trackablePointsWarped =
        // _trackables[trackableId]._trackSelection.GetTrackedFeaturesWarped();
        std::vector<cv::Point2f> trackablePoints = _trackSelection.GetTrackedFeatures();
        std::vector<cv::Point2f> trackablePointsWarped = _trackSelection.GetTrackedFeaturesWarped();
        // Create an empty result image - May be able to pre-initialize this container

        int n = (int)trackablePointsWarped.size();
        if (_trackVizActive) {
            _trackViz.templateMatching = {};
            _trackViz.templateMatching.templateMatchingCandidateCount = n;
        }

        for (int j = 0; j < n; j++) {
            auto pt = trackablePointsWarped[j];
            // if (cv::pointPolygonTest(_trackables[trackableId]._bBoxTransformed, trackablePointsWarped[j], true) > 0)
            // {
            if (cv::pointPolygonTest(_bBoxTransformed, trackablePointsWarped[j], true) > 0) {
                auto ptOrig = trackablePoints[j];

                cv::Rect templateRoi = GetTemplateRoi(pt);
                cv::Rect frameROI(0, 0, frame.cols, frame.rows);
                if (IsRoiValidForFrame(frameROI, templateRoi)) {
                    // cv::Rect markerRoi(0, 0, _trackables[trackableId]._image.cols,
                    // _trackables[trackableId]._image.rows);
                    cv::Rect markerRoi(0, 0, _image.cols, _image.rows);

                    std::vector<cv::Point2f> vertexPoints = GetVerticesFromPoint(ptOrig);
                    std::vector<cv::Point2f> vertexPointsResults;
                    // perspectiveTransform(vertexPoints, vertexPointsResults,
                    // _trackables[trackableId]._trackSelection.GetHomography());
                    perspectiveTransform(vertexPoints, vertexPointsResults, _trackSelection.GetHomography());

                    cv::Rect srcBoundingBox = cv::boundingRect(cv::Mat(vertexPointsResults));

                    vertexPoints.clear();
                    vertexPoints = GetVerticesFromTopCorner(srcBoundingBox.x, srcBoundingBox.y, srcBoundingBox.width,
                                                            srcBoundingBox.height);
                    // perspectiveTransform(vertexPoints, vertexPointsResults,
                    // _trackables[trackableId]._trackSelection.GetHomography().inv());
                    perspectiveTransform(vertexPoints, vertexPointsResults, _trackSelection.GetHomography().inv());

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
                                // cv::Mat templateImage = _trackables[trackableId]._image(templateBoundingBox);
                                cv::Mat templateImage = _image(templateBoundingBox);
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
                                    } else {
                                        if (_trackVizActive)
                                            _trackViz.templateMatching.failedTemplateMinimumCorrelationCount++;
                                    }
                                } else {
                                    if (_trackVizActive)
                                        _trackViz.templateMatching.failedTemplateMatchCount++;
                                }
                            } else {
                                if (_trackVizActive)
                                    _trackViz.templateMatching.failedTemplateBigEnoughTestCount++;
                            }
                        } else {
                            if (_trackVizActive)
                                _trackViz.templateMatching.failedSearchROIInFrameTestCount++;
                        }
                    } else {
                        if (_trackVizActive)
                            _trackViz.templateMatching.failedGotHomogTestCount++;
                    }
                } else {
                    if (_trackVizActive)
                        _trackViz.templateMatching.failedROIInFrameTestCount++;
                }
            } else {
                if (_trackVizActive)
                    _trackViz.templateMatching.failedBoundsTestCount++;
            }
        }
        bool gotHomography = updateTrackableHomography(trackableId, finalTemplatePoints, finalTemplateMatchPoints);
        if (!gotHomography) {
            // _trackables[trackableId]._isTracking = false;
            // _trackables[trackableId]._isDetected = false;
            _isTracking = false;
            _isDetected = false;
            _currentlyTrackedMarkers--;
        }
        if (_trackVizActive) {
            _trackViz.templateMatching.templateMatchingOK = gotHomography;
        }
        return gotHomography;
    }

    void processFrame(cv::Mat& frame) {
        buildImagePyramid(frame);

        if (!initialized) {
            WEBARKIT_LOGe("Reference image not found. AR is unintialized!\n");
            assert(initialized == false);
        }

        if (_trackVizActive) {
            memset(_trackViz.bounds, 0, 8 * sizeof(float));
            _trackViz.opticalFlowTrackablePoints.clear();
            _trackViz.opticalFlowTrackedPoints.clear();
            _trackViz.opticalFlowOK = false;
        }

        WEBARKIT_LOGd("Reset Tracking!\n");

        clear_output();

        _isDetected = false;

        cv::Mat frameDescr;
        std::vector<cv::KeyPoint> frameKeyPts;

        // This if cond. doesn't works as expected in artoolkitx. This make the tracking process unstable.
        // if (_currentlyTrackedMarkers < _maxNumberOfMarkersToTrack) {
            /*cv::Mat detectionFrame;
            if (_featureDetectPyrLevel < 1) {
                detectionFrame = frame;
            } else {
                cv::Mat srcFrame = frame;
                for (int pyrLevel = 1; pyrLevel <= _featureDetectPyrLevel; pyrLevel++) {
                    cv::pyrDown(srcFrame, detectionFrame, cv::Size(0, 0));
                    srcFrame = detectionFrame;
                }
            }*/

            cv::Mat featureMask = createFeatureMask(frame);

            if (!extractFeatures(frame, featureMask, frameKeyPts, frameDescr)) {
                WEBARKIT_LOGe("No features detected in extractFeatures!\n");
                // return false;
            };
            //if (!_isDetected) {
            WEBARKIT_LOGd("frame KeyPoints size: %d\n", frameKeyPts.size());
            if (static_cast<int>(frameKeyPts.size()) > minRequiredDetectedFeatures) {
                MatchFeatures(frameKeyPts, frameDescr);
            }
        //} ref -> if (_currentlyTrackedMarkers < _maxNumberOfMarkersToTrack) {
        int i = 0;
        if (_isDetected) {
            WEBARKIT_LOGd("Start tracking!\n");
            if (_frameCount > 0 && _prevPyramid.size() > 0) {
                // if (_prevPyramid.size() > 0) {
                //  std::cout << "Starting Optical Flow" << std::endl;
                std::vector<cv::Point2f> trackablePoints = _trackSelection.GetInitialFeatures();
                std::vector<cv::Point2f> trackablePointsWarped = _trackSelection.GetTrackedFeaturesWarped();
                if (!runOpticalFlow(i, trackablePoints, trackablePointsWarped)) {
                    WEBARKIT_LOGd("Optical flow failed.\n");
                } else {
                    if (_trackVizActive) { _trackViz.opticalFlowOK = true;}
                    //  Refine optical flow with template match.
                    if (!RunTemplateMatching(frame, i)) {
                        WEBARKIT_LOGd("Template matching failed.");
                    }
                    // std::cout << "Optical flow ok." << std::endl;
                }
            }
        }

        if (_isDetected || _isTracking) {
            cv::Mat _pose;
            std::vector<cv::Point2f> imgPoints = _trackSelection.GetTrackedFeaturesWarped();
            std::vector<cv::Point3f> objPoints = _trackSelection.GetTrackedFeatures3d();
            _patternTrackingInfo.cameraPoseFromPoints(_pose, objPoints, imgPoints, m_camMatrix, m_distortionCoeff);
            // _patternTrackingInfo.computePose(_pattern.points3d, warpedCorners, m_camMatrix, m_distortionCoeff);
            _patternTrackingInfo.getTrackablePose(_pose);
            _patternTrackingInfo.updateTrackable();
            _patternTrackingInfo.computeGLviewMatrix(_pose);
            fill_output(m_H);
            WEBARKIT_LOGi("Marker tracked ! Num. matches : %d\n", numMatches);
        }

        swapImagePyramid();
        _frameCount++;
    }

    void fill_output(cv::Mat& H) {
        output[0] = H.at<double>(0, 0);
        output[1] = H.at<double>(0, 1);
        output[2] = H.at<double>(0, 2);
        output[3] = H.at<double>(1, 0);
        output[4] = H.at<double>(1, 1);
        output[5] = H.at<double>(1, 2);
        output[6] = H.at<double>(2, 0);
        output[7] = H.at<double>(2, 1);
        output[8] = H.at<double>(2, 2);

        output[9] = _bBoxTransformed[0].x;
        output[10] = _bBoxTransformed[0].y;
        output[11] = _bBoxTransformed[1].x;
        output[12] = _bBoxTransformed[1].y;
        output[13] = _bBoxTransformed[2].x;
        output[14] = _bBoxTransformed[2].y;
        output[15] = _bBoxTransformed[3].x;
        output[16] = _bBoxTransformed[3].y;
    };

    void clear_output() { std::fill(output.begin(), output.end(), 0); };

    void buildImagePyramid(cv::Mat& frame) { cv::buildOpticalFlowPyramid(frame, _pyramid, winSize, maxLevel); }

    void swapImagePyramid() { _pyramid.swap(_prevPyramid); }

    void MatchFeatures(const std::vector<cv::KeyPoint>& newFrameFeatures, cv::Mat newFrameDescriptors) {
        int maxMatches = 0;
        int bestMatchIndex = -1;
        std::vector<cv::KeyPoint> finalMatched1, finalMatched2;
        // for (int i = 0; i < _trackables.size(); i++) {
        if (!_isDetected) {
            std::vector<std::vector<cv::DMatch>> matches = getMatches(newFrameDescriptors);
            numMatches = matches.size();
            WEBARKIT_LOGd("Num Matches: %d\n", numMatches);

            if (matches.size() > minRequiredDetectedFeatures) {
                std::vector<cv::KeyPoint> matched1, matched2;
                std::vector<uchar> status;
                int totalGoodMatches = 0;
                for (unsigned int j = 0; j < matches.size(); j++) {
                    // Ratio Test for outlier removal, removes ambiguous matches.
                    if (matches[j][0].distance < _nn_match_ratio * matches[j][1].distance) {
                        matched1.push_back(newFrameFeatures[matches[j][0].queryIdx]);
                        matched2.push_back(refKeyPts[matches[j][0].trainIdx]);
                        status.push_back(1);
                        totalGoodMatches++;
                    } else {
                        status.push_back(0);
                    }
                }
                // Measure goodness of match by most number of matching features.
                // This allows for maximum of a single marker to match each time.
                // TODO: Would a better metric be percentage of marker features matching?
                if (totalGoodMatches > maxMatches) {
                    finalMatched1 = matched1;
                    finalMatched2 = matched2;
                    maxMatches = totalGoodMatches;
                    // bestMatchIndex = i;
                }
            }
        }
        // } // end for cycle

        if (maxMatches > 0) {
            for (int i = 0; i < finalMatched1.size(); i++) {
                finalMatched1[i].pt.x *= _featureDetectScaleFactor[0];
                finalMatched1[i].pt.y *= _featureDetectScaleFactor[1];
            }

            homography::WebARKitHomographyInfo homoInfo =
                getHomographyInliers(Points(finalMatched2), Points(finalMatched1));
            if (homoInfo.validHomography) {
                // std::cout << "New marker detected" << std::endl;
                //_trackables[bestMatchIndex]._isDetected = true;
                _isDetected = true;
                // Since we've just detected the marker, make sure next invocation of
                // GetInitialFeatures() for this marker makes a new selection.
                //_trackables[bestMatchIndex]._trackSelection.ResetSelection();
                _trackSelection.ResetSelection();
                //_trackables[bestMatchIndex]._trackSelection.SetHomography(homoInfo.homography);
                _trackSelection.SetHomography(homoInfo.homography);

                // Use the homography to form the initial estimate of the bounding box.
                // This will be refined by the optical flow pass.
                // perspectiveTransform(_trackables[bestMatchIndex]._bBox, _trackables[bestMatchIndex]._bBoxTransformed,
                // homoInfo.homography);
                perspectiveTransform(_bBox, _bBoxTransformed, homoInfo.homography);
                if (_trackVizActive) {
                    for (int i = 0; i < 4; i++) {
                        // _trackViz.bounds[i][0] = _trackables[bestMatchIndex]._bBoxTransformed[i].x;
                        // _trackViz.bounds[i][1] = _trackables[bestMatchIndex]._bBoxTransformed[i].y;
                        _trackViz.bounds[i][0] = _bBoxTransformed[i].x;
                        _trackViz.bounds[i][1] = _bBoxTransformed[i].y;
                    }
                }
                _currentlyTrackedMarkers++;
            }
        }
    }

    bool runOpticalFlow(int trackableId, const std::vector<cv::Point2f>& trackablePoints,
                        const std::vector<cv::Point2f>& trackablePointsWarped) {
        std::vector<cv::Point2f> flowResultPoints, trackablePointsWarpedResult;
        std::vector<uchar> statusFirstPass, statusSecondPass;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(_prevPyramid, _pyramid, trackablePointsWarped, flowResultPoints, statusFirstPass, err,
                                 winSize, maxLevel, termcrit, 0, 0.001);
        // By using bi-directional optical flow, we improve quality of detected points.
        cv::calcOpticalFlowPyrLK(_pyramid, _prevPyramid, flowResultPoints, trackablePointsWarpedResult,
                                 statusSecondPass, err, winSize, maxLevel, termcrit, 0, 0.001);

        // Keep only the points for which flow was found in both temporal directions.
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

        if (_trackVizActive) {
            _trackViz.opticalFlowTrackablePoints = filteredTrackablePoints;
            _trackViz.opticalFlowTrackedPoints = filteredTrackedPoints;
        }
        // std::cout << "Optical flow discarded " << killed1 << " of " << flowResultPoints.size() << " points" <<
        // std::endl;

        if (!updateTrackableHomography(trackableId, filteredTrackablePoints, filteredTrackedPoints)) {
            _isDetected = false;
            _isTracking = false;
            this->_valid = false;
            _currentlyTrackedMarkers--;
            return false;
        }

        _isTracking = true;
        return true;
    }

    bool updateTrackableHomography(int trackableId, const std::vector<cv::Point2f>& matchedPoints1,
                                   const std::vector<cv::Point2f>& matchedPoints2) {
        if (matchedPoints1.size() > 4) {
            homography::WebARKitHomographyInfo homoInfo = getHomographyInliers(matchedPoints1, matchedPoints2);
            if (homoInfo.validHomography) {
                _trackSelection.UpdatePointStatus(homoInfo.status);
                _trackSelection.SetHomography(homoInfo.homography);
                m_H = homoInfo.homography;
                this->_valid = homoInfo.validHomography;
                // Update the bounding box.
                perspectiveTransform(_bBox, _bBoxTransformed, homoInfo.homography);
                if (_trackVizActive) {
                    for (int i = 0; i < 4; i++) {
                        // _trackViz.bounds[i][0] = _trackables[trackableId]._bBoxTransformed[i].x;
                        // _trackViz.bounds[i][1] = _trackables[trackableId]._bBoxTransformed[i].y;
                        _trackViz.bounds[i][0] = _bBoxTransformed[i].x;
                        _trackViz.bounds[i][1] = _bBoxTransformed[i].y;
                    }
                }
                if (_frameCount > 1) {
                    // _trackables[trackableId]._trackSelection.ResetSelection();
                    _trackSelection.ResetSelection();
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

    cv::Mat MatchTemplateToImage(cv::Mat searchImage, cv::Mat warpedTemplate) {
        int result_cols = searchImage.cols - warpedTemplate.cols + 1;
        int result_rows = searchImage.rows - warpedTemplate.rows + 1;
        if (result_cols > 0 && result_rows > 0) {
            cv::Mat result;
            result.create(result_rows, result_cols, CV_32FC1);

            double minVal;
            double maxVal;
            minMaxLoc(warpedTemplate, &minVal, &maxVal, 0, 0, cv::noArray());

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

    bool IsRoiValidForFrame(cv::Rect frameRoi, cv::Rect roi) { return (roi & frameRoi) == roi; }

    cv::Rect InflateRoi(cv::Rect roi, int inflationFactor) {
        cv::Rect newRoi = roi;
        newRoi.x -= inflationFactor;
        newRoi.y -= inflationFactor;
        newRoi.width += 2 * inflationFactor;
        newRoi.height += 2 * inflationFactor;
        return newRoi;
    }

    std::vector<cv::Point2f> FloorVertexPoints(const std::vector<cv::Point2f>& vertexPoints) {
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

    std::vector<std::vector<cv::DMatch>> getMatches(const cv::Mat& frameDescr) {
        std::vector<std::vector<cv::DMatch>> knnMatches;
        _matcher->knnMatch(frameDescr, refDescr, knnMatches, 2);
        return knnMatches;
    }

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
                    // contours[0].push_back(cv::Point(_trackables[i]._bBoxTransformed[j].x/_featureDetectScaleFactor[0],_trackables[i]._bBoxTransformed[j].y/_featureDetectScaleFactor[1]));
                    contours[0].push_back(cv::Point(_bBoxTransformed[j].x/_featureDetectScaleFactor[0],_bBoxTransformed[j].y/_featureDetectScaleFactor[1]));
                }
            drawContours(featureMask, contours, 0, cv::Scalar(0), -1, 8);
        }
        return featureMask;
    }

    cv::Mat _image;

    int _currentlyTrackedMarkers;

    int _frameCount;

    int _frameSizeX;
    int _frameSizeY;
    /// Pyramid level used in downsampling incoming image for feature matching. 0 = no size change, 1 = half width/height, 2 = quarter width/heigh etc.
    int _featureDetectPyrLevel;
    /// Scale factor applied to images used for feature matching. Will be 2^_featureDetectPyrLevel.
    cv::Vec2f _featureDetectScaleFactor;

    bool _valid;

    bool _isDetected;

    bool _isTracking;

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

    TrackingPointSelector _trackSelection;

    cv::Matx33d m_camMatrix;
    cv::Mat m_distortionCoeff;

    std::array<double, 16> m_cameraProjectionMatrix;
  private:
    int _maxNumberOfMarkersToTrack;

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
            const double akaze_thresh = 3e-4; // AKAZE detection threshold set to locate about 1000 keypoints
            cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
            akaze->setThreshold(akaze_thresh);
            this->_featureDetector = akaze;
            this->_featureDescriptor = akaze;
        } else if (trackerType == webarkit::TRACKER_TYPE::ORB_TRACKER) {
            this->_featureDetector = cv::ORB::create(DEFAULT_MAX_FEATURES);
            this->_featureDescriptor = cv::ORB::create(DEFAULT_MAX_FEATURES);
        } else if (trackerType == webarkit::TRACKER_TYPE::FREAK_TRACKER) {
            this->_featureDetector = cv::ORB::create(DEFAULT_MAX_FEATURES);
            this->_featureDescriptor = cv::xfeatures2d::FREAK::create();
        } else if (trackerType == webarkit::TRACKER_TYPE::TEBLID_TRACKER) {
            this->_featureDetector = cv::ORB::create(TEBLID_MAX_FEATURES);
            this->_featureDescriptor = cv::xfeatures2d::TEBLID::create(1.00f);
        }
        if (trackerType == webarkit::TRACKER_TYPE::AKAZE_TRACKER) {
            _matcher = cv::BFMatcher::create();
        } else {
            _matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        }
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

void WebARKitTracker::processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace,
                                       BLUR_TYPE blurType) {
    _trackerImpl->processFrameData(frameData, frameCols, frameRows, colorSpace, blurType);
}

std::vector<double> WebARKitTracker::getOutputData() { return _trackerImpl->getOutputData(); }

cv::Mat WebARKitTracker::getPoseMatrix() { return _trackerImpl->getPoseMatrix(); }

float* WebARKitTracker::getPoseMatrix2() { return _trackerImpl->getPoseMatrix2(); }

cv::Mat WebARKitTracker::getGLViewMatrix() { return _trackerImpl->getGLViewMatrix(); }

std::array<double, 16> WebARKitTracker::getCameraProjectionMatrix() {
    return _trackerImpl->getCameraProjectionMatrix();
}

bool WebARKitTracker::isValid() { return _trackerImpl->isValid(); }

} // namespace webarkit