#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitOrbTracker.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>

namespace webarkit {

WebARKitOrbTracker::WebARKitOrbTracker()
    : WebARKitTracker(), orb(nullptr), matcher(nullptr) {}

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
  } else {
    this->_valid = track(frame);
  }
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

} // namespace webarkit