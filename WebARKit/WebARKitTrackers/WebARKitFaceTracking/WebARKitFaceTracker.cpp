#include <WebARKitFaceTracker.h>

namespace webarkit {

// Constructor and Destructor
WebARKitFaceTracker::WebARKitFaceTracker()
    : fps(30), eyeDistance(0), dotRadius(0), sigma(0),
      termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03),
      winSize(101, 101), eyeDistanceNotCalculated(true), maxLevel(5),
      isFirstFrame(true), showStabilized(false) {
  // Constructor implementation
}

WebARKitFaceTracker::~WebARKitFaceTracker() {
  // Destructor implementation
}

// Initialization
void WebARKitFaceTracker::init(const std::string &model_path) {
  detector = dlib::get_frontal_face_detector();
  webarkitLOGi("Loading model from: %s", model_path.c_str());
  dlib::deserialize(model_path) >> landmarkDetector;
  webarkitLOGi("Model loaded successfully!");

  for (auto k = 0; k < landmarkDetector.num_parts(); ++k) {
    pointsPrev.emplace_back(0, 0);
    points.emplace_back(0, 0);
    pointsDetectedCur.emplace_back(0, 0);
    pointsDetectedPrev.emplace_back(0, 0);
  }
  webarkitLOGi("Init done.");
}

// Pre-tracking setup
bool WebARKitFaceTracker::preTrack(const cv::Mat &image) {
  imPrev = image.clone();
  cv::cvtColor(imPrev, imGrayPrev, cv::COLOR_BGR2GRAY);
  cv::buildOpticalFlowPyramid(imGrayPrev, imGrayPrevPyr, winSize, maxLevel);
  prevSize = imPrev.size();

  if (prevSize.empty()) {
    webarkitLOGe("Error: Previous image is empty.");
    return false;
  }
  return true;
}

// Tracking
void WebARKitFaceTracker::track(const cv::Mat &image) {
  if (!preTrack(image)) {
    return;
  }

  while (true) {
    cv::cvtColor(image, imGray, cv::COLOR_BGR2GRAY);
    float height = image.rows;
    float IMAGE_RESIZE = height / RESIZE_HEIGHT;
    cv::resize(image, imSmall, cv::Size(), 1.0 / IMAGE_RESIZE, 1.0 / IMAGE_RESIZE);

    dlib::cv_image<dlib::bgr_pixel> cimg_small(imSmall);
    dlib::cv_image<dlib::bgr_pixel> cimg(image);

    faces = detector(cimg_small);
    if (faces.empty()) continue;

    std::vector<dlib::full_object_detection> shapes;
    for (const auto &face : faces) {
      dlib::rectangle r(
          (long)(face.left() * IMAGE_RESIZE),
          (long)(face.top() * IMAGE_RESIZE),
          (long)(face.right() * IMAGE_RESIZE),
          (long)(face.bottom() * IMAGE_RESIZE)
      );

      dlib::full_object_detection shape = landmarkDetector(cimg, r);
      shapes.push_back(shape);

      for (auto k = 0; k < shape.num_parts(); ++k) {
        if (isFirstFrame) {
          pointsPrev[k].x = pointsDetectedPrev[k].x = shape.part(k).x();
          pointsPrev[k].y = pointsDetectedPrev[k].y = shape.part(k).y();
        } else {
          pointsPrev[k] = points[k];
          pointsDetectedPrev[k] = pointsDetectedCur[k];
        }

        points[k].x = pointsDetectedCur[k].x = shape.part(k).x();
        points[k].y = pointsDetectedCur[k].y = shape.part(k).y();
      }

      if (eyeDistanceNotCalculated) {
        eyeDistance = interEyeDistance(shape);
        winSize = cv::Size(2 * int(eyeDistance / 4) + 1, 2 * int(eyeDistance / 4) + 1);
        eyeDistanceNotCalculated = false;
        dotRadius = eyeDistance > 100 ? 3 : 2;
        sigma = eyeDistance * eyeDistance / 400;
      }

      cv::buildOpticalFlowPyramid(imGray, imGrayPyr, winSize, maxLevel);
      cv::calcOpticalFlowPyrLK(imGrayPrevPyr, imGrayPyr, pointsPrev, points, status, err, winSize, maxLevel, termcrit, 0, 0.0001);

      for (auto k = 0; k < shape.num_parts(); ++k) {
        double n = norm(pointsDetectedPrev[k] - pointsDetectedCur[k]);
        double alpha = exp(-n * n / sigma);
        points[k] = (1 - alpha) * pointsDetectedCur[k] + alpha * points[k];
      }

      if (showStabilized) {
        // Render stabilized points
        // renderFace(im, points, cv::Scalar(255, 0, 0), dotRadius);
      } else {
        // Render unstabilized points
        // renderFace(im, pointsDetectedCur, cv::Scalar(0, 0, 255), dotRadius);
      }
    }

    imPrev = image.clone();
    imGrayPrev = imGray.clone();
    imGrayPrevPyr = imGrayPyr;
    imGrayPyr.clear();
    isFirstFrame = false;
  }
}

// Utility
double WebARKitFaceTracker::interEyeDistance(dlib::full_object_detection &shape) {
  cv::Point2f leftEyeLeftCorner(shape.part(36).x(), shape.part(36).y());
  cv::Point2f rightEyeRightCorner(shape.part(45).x(), shape.part(45).y());
  return norm(rightEyeRightCorner - leftEyeLeftCorner);
}

} // namespace webarkit