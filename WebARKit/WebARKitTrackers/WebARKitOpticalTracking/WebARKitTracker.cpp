#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>


namespace webarkit {

WebARKitTracker::WebARKitTracker()
    : corners(4), initialized(false), output(17, 0.0), _valid(false), numMatches(0) {}

bool WebARKitTracker::track(cv::Mat &currIm) {
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


std::vector<double> WebARKitTracker::getOutputData() { return output; }

bool WebARKitTracker::isValid() { return _valid; }

bool WebARKitTracker::homographyValid(cv::Mat &H) {
  if (H.empty()) {
    return false;
  }
  const double det = H.at<double>(0, 0) * H.at<double>(1, 1) -
                     H.at<double>(1, 0) * H.at<double>(0, 1);
  return 1 / N < fabs(det) && fabs(det) < N;
}

void WebARKitTracker::fill_output(cv::Mat &H) {
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
}

void WebARKitTracker::clear_output() { output = std::vector<double>(17, 0.0); }

} // namespace webarkit