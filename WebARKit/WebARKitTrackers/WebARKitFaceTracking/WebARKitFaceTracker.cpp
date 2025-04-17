#include <WebARKitFaceTracker.h>

namespace webarkit {

WebARKitFaceTracker::WebARKitFaceTracker()
    : fps(30), eyeDistance(0), dotRadius(0), sigma(0),
    termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20,
        0.03), winSize(101, 101),
      eyeDistanceNotCalculated(true), maxLevel(5), isFirstFrame(true),
      showStabilized(false) {
  // Constructor implementation
}

WebARKitFaceTracker::~WebARKitFaceTracker() {
  // Destructor implementation
}

double WebARKitFaceTracker::interEyeDistance(dlib::full_object_detection& shape)
{
    cv::Point2f leftEyeLeftCorner(shape.part(36).x(), shape.part(36).y());
    cv::Point2f rightEyeRightCorner(shape.part(45).x(), shape.part(45).y());
    double distance = norm(rightEyeRightCorner - leftEyeLeftCorner);
    return distance;
}

void WebARKitFaceTracker::init(const std::string &model_path) {
  // Load the face detector and shape predictor models
  detector = dlib::get_frontal_face_detector();
  dlib::deserialize(model_path) >> landmarkDetector;

  // Initialize point arrays with (0,0)
  for (auto k = 0; k < landmarkDetector.num_parts(); ++k) {
    pointsPrev.push_back(cv::Point2f(0, 0));
    points.push_back(cv::Point2f(0, 0));
    pointsDetectedCur.push_back(cv::Point2f(0, 0));
    pointsDetectedPrev.push_back(cv::Point2f(0, 0));
  }
}

bool WebARKitFaceTracker::preTrack(const cv::Mat &image) {
  //image.copy(imPrev); // Copy the current image to imPrev
  imPrev = image.clone();

  // Convert to grayscale for optical flow calculation
  cv::cvtColor(imPrev, imGrayPrev, cv::COLOR_BGR2GRAY);

  // Build image pyramid for fast optical flow calculation
  cv::buildOpticalFlowPyramid(imGrayPrev, imGrayPrevPyr, winSize, maxLevel);

  // Get image size
  prevSize = imPrev.size();

  if (prevSize.empty()) {
    std::cerr << "Error: Previous image is empty." << std::endl;
    return false; // Return false to indicate failure
  } else {
    return true;
  }
}

void WebARKitFaceTracker::track(const cv::Mat &image) {

  if (preTrack(image)) {
    return; // If preTrack fails, exit the function
  }

  while (true) {
    cv::cvtColor(image, imGray, cv::COLOR_BGR2GRAY);
    float height = image.rows;
    float IMAGE_RESIZE = height / RESIZE_HEIGHT;
    // Resize image for faster face detection
    cv::resize(image, imSmall, cv::Size(), 1.0 / IMAGE_RESIZE,
               1.0 / IMAGE_RESIZE);

    // Change to dlib's image format. No memory is copied.
    dlib::cv_image<dlib::bgr_pixel> cimg_small(imSmall);
    dlib::cv_image<dlib::bgr_pixel> cimg(image);
    // Convert the input image to dlib format
    //dlib::cv_image<dlib::bgr_pixel> dlibImage(image);

    // Detect faces in the image
    faces = detector(cimg_small);

    if (faces.size() < 1) continue;

    // If no faces are detected, return empty landmarks
    /*if (faces.empty()) {
      landmarks.clear();
      return;
    }*/

    // Space for landmarks on multiple faces.
    std::vector<dlib::full_object_detection> shapes;

     // Loop over all faces
     //for (unsigned long i = 0; i < faces.size(); ++i)
     for (const auto &face : faces) 
     {

        // Face detector was found over a smaller image.
        // So, we scale face rectangle to correct size.
        dlib::rectangle r(
            (long)(face.left() * IMAGE_RESIZE),
            (long)(face.top() * IMAGE_RESIZE),
            (long)(face.right() * IMAGE_RESIZE),
            (long)(face.bottom() * IMAGE_RESIZE)
        );

        // Run landmark detector on current frame
        dlib::full_object_detection shape = landmarkDetector(cimg, r);

        // Save current face in a vector
        shapes.push_back(shape);

        // Loop over every point
        for (auto k = 0; k < shape.num_parts(); ++k)
        {

            if (isFirstFrame)
            {
                // If it is the first frame copy the current frame points
                pointsPrev[k].x = pointsDetectedPrev[k].x = shape.part(k).x();
                pointsPrev[k].y = pointsDetectedPrev[k].y = shape.part(k).y();
            }
            else
            {
                // If not the first frame, copy points from previous frame.
                pointsPrev[k] = points[k];
                pointsDetectedPrev[k] = pointsDetectedCur[k];
            }

            // pointsDetectedCur stores results returned by the facial landmark detector
            // points stores the stabilized landmark points
            points[k].x = pointsDetectedCur[k].x = shape.part(k).x();
            points[k].y = pointsDetectedCur[k].y = shape.part(k).y();
        }

        if (eyeDistanceNotCalculated)
        {
            eyeDistance = interEyeDistance(shape);
            winSize = cv::Size(2 * int(eyeDistance / 4) + 1, 2 * int(eyeDistance / 4) + 1);
            eyeDistanceNotCalculated = false;
            dotRadius = eyeDistance > 100 ? 3 : 2;
            sigma = eyeDistance * eyeDistance / 400;
        }

        // Build an image pyramid to speed up optical flow
        cv::buildOpticalFlowPyramid(imGray, imGrayPyr, winSize, maxLevel);

        // Predict landmarks based on optical flow. points stores the new location of points.
        cv::calcOpticalFlowPyrLK(imGrayPrevPyr, imGrayPyr, pointsPrev, points, status, err, winSize, maxLevel, termcrit, 0, 0.0001);

        // Final landmark points are a weighted average of
        // detected landmarks and tracked landmarks

        for (auto k = 0; k < shape.num_parts(); ++k)
        {
            double n = norm(pointsDetectedPrev[k] - pointsDetectedCur[k]);
            double alpha = exp(-n * n / sigma);
            points[k] = (1 - alpha) * pointsDetectedCur[k] + alpha * points[k];
            // constrainPoint(points[k], imGray.size());
        }

        if (showStabilized)
        {
            // Show optical flow stabilized points
            //renderFace(im, points, cv::Scalar(255, 0, 0), dotRadius);
        }
        else
        {
            // Show landmark points (unstabilized)
            //renderFace(im, pointsDetectedCur, cv::Scalar(0, 0, 255), dotRadius);
        }
    }

    // Detect landmarks for each face
    /*for (const auto &face : faces) {
      dlib::full_object_detection shape = landmarkDetector(dlibImage, face);
      std::vector<cv::Point2f> landmarkPoints;
      for (int i = 0; i < shape.num_parts(); ++i) {
        landmarkPoints.push_back(
            cv::Point2f(shape.part(i).x(), shape.part(i).y()));
      }
      landmarks.push_back(landmarkPoints);
    }*/
    // Get ready for next frame.
    imPrev = image.clone();
    imGrayPrev = imGray.clone();
    imGrayPrevPyr = imGrayPyr;
    imGrayPyr = std::vector<cv::Mat>();
    isFirstFrame = false; // Set the first frame flag to false
  }
}
}