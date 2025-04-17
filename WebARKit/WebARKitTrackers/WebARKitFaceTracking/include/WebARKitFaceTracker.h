#ifndef WEBARKITFACETRACKER_H
#define WEBARKITFACETRACKER_H

#define DLIB_NO_GUI_SUPPORT 1

#include <cmath>
//#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
//#include <dlib/image_processing/render_face_detections.h>
#include <dlib/opencv.h>
#include <iostream>
#include <opencv2/core.hpp>    // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/core/types_c.h>
//#include <opencv2/highgui.hpp> // OpenCV window I/O
//#include <opencv2/imgproc.hpp> // Gaussian Blur
#include <opencv2/video.hpp>
//#include <opencv2/videoio.hpp>
#include <string>

constexpr int RESIZE_HEIGHT = 360;

namespace webarkit {
class WebARKitFaceTracker {
public:
  WebARKitFaceTracker();
  ~WebARKitFaceTracker();

  void init(const std::string &model_path);
  
  void track(const cv::Mat &image);

private:
  double interEyeDistance(dlib::full_object_detection& shape);
  bool preTrack(const cv::Mat &image);

  cv::TermCriteria termcrit;
  cv::Size winSize;
  double eyeDistance, dotRadius, sigma;
  bool eyeDistanceNotCalculated;
  int maxLevel;
  std::vector<uchar> status;

  std::vector<float> err;

  double fps;

  // Space for current frame, previous frame, and the grayscale versions.
  cv::Mat im, imSmall, imPrev, imGray, imGrayPrev;

  // Vector of images for storing image pyramids for optical flow
  std::vector<cv::Mat> imGrayPyr, imGrayPrevPyr;

  dlib::frontal_face_detector detector;
  dlib::shape_predictor landmarkDetector;

  // Vector to store face rectangles
  std::vector<dlib::rectangle> faces;

  // Space for landmark points
  std::vector<cv::Point2f> points, pointsPrev, pointsDetectedCur, pointsDetectedPrev;

  // First frame is handled differently.
  bool isFirstFrame;

  // Show stabilized video flag
  bool showStabilized;

  cv::Size prevSize;
};

} // namespace webarkit

#endif // WEBARKITFACETRACKER_H