#ifndef WEBARKITFACETRACKER_H
#define WEBARKITFACETRACKER_H

//#define DLIB_NO_GUI_SUPPORT 1

#include <cmath>
#include <iostream>
#include <string>

#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>

#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/video.hpp>

#include <WebARKit/WebARKitLog.h>

constexpr int RESIZE_HEIGHT = 360;

namespace webarkit {

class WebARKitFaceTracker {
public:
  WebARKitFaceTracker();
  ~WebARKitFaceTracker();

  void init(const std::string &model_path);
  void track(const cv::Mat &image);

private:
  double interEyeDistance(dlib::full_object_detection &shape);
  bool preTrack(const cv::Mat &image);

  // Optical flow parameters
  cv::TermCriteria termcrit;
  cv::Size winSize;
  int maxLevel;

  // Eye distance and stabilization parameters
  double eyeDistance, dotRadius, sigma;
  bool eyeDistanceNotCalculated;

  // Frame and image data
  cv::Mat im, imSmall, imPrev, imGray, imGrayPrev;
  std::vector<cv::Mat> imGrayPyr, imGrayPrevPyr;

  // Face detection and landmark detection
  dlib::frontal_face_detector detector;
  dlib::shape_predictor landmarkDetector;
  std::vector<dlib::rectangle> faces;

  // Landmark points
  std::vector<cv::Point2f> points, pointsPrev, pointsDetectedCur, pointsDetectedPrev;

  // Optical flow status and error
  std::vector<uchar> status;
  std::vector<float> err;

  // Flags and settings
  bool isFirstFrame;
  bool showStabilized;
  double fps;
  cv::Size prevSize;
};

} // namespace webarkit

#endif // WEBARKITFACETRACKER_H