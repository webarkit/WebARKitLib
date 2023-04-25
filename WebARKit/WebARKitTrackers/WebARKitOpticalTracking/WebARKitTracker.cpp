#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>


namespace webarkit {

WebARKitTracker::WebARKitTracker()
    : corners(4), output(17, 0.0), _valid(false) {}

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