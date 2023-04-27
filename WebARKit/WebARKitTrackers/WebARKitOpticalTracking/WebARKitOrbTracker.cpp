#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitOrbTracker.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>

namespace webarkit {

WebARKitOrbTracker::WebARKitOrbTracker()
    : WebARKitTracker(), initialized(false), orb(nullptr), matcher(nullptr),
      numMatches(0) {}

void WebARKitOrbTracker::initialize_gray_raw(uchar *refData, size_t refCols,
                                             size_t refRows) {
  std::cout << "Init Tracker!" << std::endl;

  orb = cv::ORB::create(MAX_FEATURES);
  matcher = cv::BFMatcher::create();

  cv::Mat refGray(refRows, refCols, CV_8UC1, refData);

  orb->detectAndCompute(refGray, cv::noArray(), refKeyPts, refDescr);

  corners[0] = cvPoint(0, 0);
  corners[1] = cvPoint(refCols, 0);
  corners[2] = cvPoint(refCols, refRows);
  corners[3] = cvPoint(0, refRows);

  initialized = true;

  std::cout << "Tracker ready!" << std::endl;
}

void WebARKitOrbTracker::processFrameData(uchar *frameData, size_t frameCols,
                                          size_t frameRows,
                                          ColorSpace colorSpace) {
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
}

void WebARKitOrbTracker::processFrame(cv::Mat &frame) {
  if (!this->_valid) {
    this->_valid = resetTracking(frame);
  }
  this->_valid = track(frame);
}

bool WebARKitOrbTracker::resetTracking(cv::Mat &currIm) {
  if (!initialized) {
    std::cout << "Reference image not found. AR is unintialized!" << std::endl;
    return NULL;
  }

  std::cout << "Reset Tracking!" << std::endl;

  clear_output();

  cv::Mat frameDescr;
  std::vector<cv::KeyPoint> frameKeyPts;

  orb->detectAndCompute(currIm, cv::noArray(), frameKeyPts, frameDescr);

  std::vector<std::vector<cv::DMatch>> knnMatches;
  matcher->knnMatch(frameDescr, refDescr, knnMatches, 2);

  framePts.clear();
  std::vector<cv::Point2f> refPts;

  // find the best matches
  for (size_t i = 0; i < knnMatches.size(); ++i) {
    if (knnMatches[i][0].distance <
        GOOD_MATCH_RATIO * knnMatches[i][1].distance) {
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
}

bool WebARKitOrbTracker::track(cv::Mat &currIm) {
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

      diff = sqrt(pow(currPts[i].x - framePts[i].x, 2.0) +
                  pow(currPts[i].y - framePts[i].y, 2.0));
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
}

} // namespace webarkit