#include <gtest/gtest.h>
#include <WebARKitManager.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>
#include <WebARKitCamera.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <opencv2/imgcodecs.hpp>

class WebARKitEnumTest : public testing::TestWithParam<std::tuple<webarkit::TRACKER_TYPE, webarkit::ColorSpace>> {};


TEST_P(WebARKitEnumTest, TestEnumValues) {
  webarkit::TRACKER_TYPE tracker_value = std::get<0>(GetParam());
  webarkit::ColorSpace color_value = std::get<1>(GetParam());
  EXPECT_TRUE(tracker_value == webarkit::TRACKER_TYPE::AKAZE_TRACKER ||
              tracker_value == webarkit::TRACKER_TYPE::ORB_TRACKER ||
              tracker_value == webarkit::TRACKER_TYPE::FREAK_TRACKER ||
              tracker_value == webarkit::TRACKER_TYPE::TEBLID_TRACKER);
  EXPECT_TRUE(color_value == webarkit::ColorSpace::RGB ||
              color_value == webarkit::ColorSpace::RGBA ||
              color_value == webarkit::ColorSpace::GRAY ||
              color_value == webarkit::ColorSpace::BGR ||
              color_value == webarkit::ColorSpace::BGRA);
}

INSTANTIATE_TEST_SUITE_P(WebARKitEnumTestSuite, WebARKitEnumTest,
                         testing::Combine(testing::ValuesIn({webarkit::TRACKER_TYPE::AKAZE_TRACKER,
                                                             webarkit::TRACKER_TYPE::ORB_TRACKER,
                                                             webarkit::TRACKER_TYPE::FREAK_TRACKER,
                                                             webarkit::TRACKER_TYPE::TEBLID_TRACKER}),
                                          testing::ValuesIn({webarkit::ColorSpace::RGB,
                                                             webarkit::ColorSpace::RGBA,
                                                             webarkit::ColorSpace::GRAY,
                                                             webarkit::ColorSpace::BGR,
                                                             webarkit::ColorSpace::BGRA})));

// WebARKitLib#25: the enum values are part of the JS/C ABI, so new entries must be
// appended rather than renumbering the existing ones.
TEST(WebARKitEnumTest, ColorSpaceValuesAreStable) {
  EXPECT_EQ(webarkit::ColorSpace::RGB, 0);
  EXPECT_EQ(webarkit::ColorSpace::RGBA, 1);
  EXPECT_EQ(webarkit::ColorSpace::GRAY, 2);
  EXPECT_EQ(webarkit::ColorSpace::BGR, 3);
  EXPECT_EQ(webarkit::ColorSpace::BGRA, 4);
}

// WebARKitLib#25: the same picture handed over in RGB(A) or BGR(A) order must give
// the same grayscale image, for both convert2Grayscale overloads.
TEST(WebARKitColorSpaceTest, BgrAndRgbOrdersGiveSameGray) {
  // Distinct R/G/B values per pixel so a channel swap would change the luma.
  cv::Mat rgb(4, 4, CV_8UC3);
  cv::randu(rgb, cv::Scalar::all(0), cv::Scalar::all(255));
  cv::Mat bgr, rgba, bgra;
  cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
  cv::cvtColor(rgb, rgba, cv::COLOR_RGB2RGBA);
  cv::cvtColor(rgb, bgra, cv::COLOR_RGB2BGRA);

  cv::Mat expected = webarkit::convert2Grayscale(rgb, rgb.cols, rgb.rows, webarkit::ColorSpace::RGB);

  auto expectSame = [&](cv::Mat gray) {
    ASSERT_EQ(gray.type(), CV_8UC1);
    EXPECT_EQ(cv::norm(gray, expected, cv::NORM_INF), 0.0);
  };
  expectSame(webarkit::convert2Grayscale(bgr, bgr.cols, bgr.rows, webarkit::ColorSpace::BGR));
  expectSame(webarkit::convert2Grayscale(rgba, rgba.cols, rgba.rows, webarkit::ColorSpace::RGBA));
  expectSame(webarkit::convert2Grayscale(bgra, bgra.cols, bgra.rows, webarkit::ColorSpace::BGRA));
  expectSame(webarkit::convert2Grayscale(bgr.data, bgr.cols, bgr.rows, webarkit::ColorSpace::BGR).clone());
  expectSame(webarkit::convert2Grayscale(bgra.data, bgra.cols, bgra.rows, webarkit::ColorSpace::BGRA).clone());
}
                                                          
TEST(WebARKitConfigTest, TestConfigValues) {
  EXPECT_DOUBLE_EQ(DEFAULT_NN_MATCH_RATIO, 0.7);
  EXPECT_DOUBLE_EQ(TEBLID_NN_MATCH_RATIO, 0.8);
  EXPECT_DOUBLE_EQ(AKAZE_NN_MATCH_RATIO, 0.8);
  EXPECT_DOUBLE_EQ(FREAK_NN_MATCH_RATIO, 0.8);
  EXPECT_EQ(DEFAULT_MAX_FEATURES, 800);
  EXPECT_EQ(TEBLID_MAX_FEATURES, 1000);
  EXPECT_EQ(N, 10);
  EXPECT_EQ(MIN_NUM_MATCHES, 8);
  EXPECT_EQ(maxLevel, 3);
  EXPECT_EQ(featureDetectPyramidLevel, 1.05f);
  EXPECT_EQ(featureBorder, 8);
  EXPECT_EQ(WEBARKIT_HEADER_VERSION_STRING, "0.12.0");
}

TEST(WebARKitConfigTest, TestWinSize) {
  cv::Size expected_size(31, 31);
  EXPECT_EQ(expected_size.width, winSize.width);
  EXPECT_EQ(expected_size.height, winSize.height);
}

TEST(WebARKitConfigTest, TestTermCriteria) {
  int expected_type = cv::TermCriteria::COUNT | cv::TermCriteria::EPS;
  int expected_max_count = 20;
  double expected_epsilon = 0.03;
  EXPECT_EQ(expected_type, termcrit.type);
  EXPECT_EQ(expected_max_count, termcrit.maxCount);
  EXPECT_EQ(expected_epsilon, termcrit.epsilon);
}

TEST(WebARKitConfigTest, TestBlurSize) {
  cv::Size expected_blur_size(3, 3);
  EXPECT_EQ(expected_blur_size.width, blurSize.width);
  EXPECT_EQ(expected_blur_size.height, blurSize.height);
}

TEST(WebARKitConfigTest, TestPIConstant) {
  double internal_m_pi = 3.14159265358979323846;
  EXPECT_EQ(internal_m_pi, m_pi);
}

TEST(WebARKitCameraTest, TestCamera) {
  int width = 640;
  int height = 480;
  webarkit::WebARKitCamera camera;
  EXPECT_TRUE(camera.setupCamera(width, height));
  std::array<double, 9> camera_mat = camera.getCameraData();
  EXPECT_EQ(camera_mat[0], 571.25920269684582);
  EXPECT_EQ(camera_mat[2], 320.0);
  EXPECT_EQ(camera_mat[4], 571.25920269684582);
  EXPECT_EQ(camera_mat[5], 240.0);
  EXPECT_EQ(camera_mat[8], 1.0);
  EXPECT_EQ(camera.getFocalLength(), 571.25920269684582);
  camera.printSettings();
}

TEST(WebARKitGLTest, TestCameraProjectionMatrix) {
  int width = 640;
  int height = 480;
  webarkit::WebARKitCamera camera;
  camera.setupCamera(width, height);
  std::array<double, 9> camera_mat = camera.getCameraData();
  std::array<double, 16> projectionMatrix = {0.0};
  webarkit::cameraProjectionMatrix(camera_mat, 0.01, 100.0, width, height, projectionMatrix);
  EXPECT_EQ(projectionMatrix[0], 1.7851850084276433);
  EXPECT_EQ(projectionMatrix[5], 2.3802466779035241);
  EXPECT_EQ(projectionMatrix[10], -1.0002000200020003);
  EXPECT_EQ(projectionMatrix[11], -1.0);
  EXPECT_EQ(projectionMatrix[14], -0.020002000200020003);
}

// Check WebARKitManager initialisation.
TEST(WebARKitTest, InitialiseBaseAkazeTest) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Check if the WebARKitManager initialisation is successful
  EXPECT_TRUE(manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, 640, 480));
}

// Check WebARKitManager initialisation.
TEST(WebARKitTest, InitialiseBaseFreakTest) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Check if the WebARKitManager initialisation is successful
  EXPECT_TRUE(manager.initialiseBase(webarkit::TRACKER_TYPE::FREAK_TRACKER, 640, 480));
}

// Check WebARKitManager initialisation.
TEST(WebARKitTest, InitialiseBaseOrbTest) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Check if the WebARKitManager initialisation is successful
  EXPECT_TRUE(manager.initialiseBase(webarkit::TRACKER_TYPE::ORB_TRACKER, 640, 480));
}

// Check WebARKitManager initialisation.
TEST(WebARKitTest, InitialiseBaseTeblidTest) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Check if the WebARKitManager initialisation is successful
  EXPECT_TRUE(manager.initialiseBase(webarkit::TRACKER_TYPE::TEBLID_TRACKER, 640, 480));
}

// Check WebARKit version
TEST(WebARKitTest, CheckWebARKitVersion) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Init the manager with the Akaze tracker
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, 640, 480);
  // Check if the WebARKit version is correct
  EXPECT_STREQ(manager.getWebARKitVersion().c_str(), "0.12.0");
}

// Check cameraProjectionMatrix from manager
TEST(WebARKitTest, CheckCameraProjectionMatrix) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Init the manager with the Akaze tracker
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, 640, 480);
  // Check if the cameraProjectionMatrix is correct
  std::array<double, 16> camProjectionMatrix = manager.getCameraProjectionMatrix();
  EXPECT_EQ(camProjectionMatrix[0], 1.7851850084276433);
  EXPECT_EQ(camProjectionMatrix[5], 2.3802466779035241);
  EXPECT_EQ(camProjectionMatrix[10], -1.0002000200020003);
  EXPECT_EQ(camProjectionMatrix[11], -1.0);
  EXPECT_EQ(camProjectionMatrix[14], -0.20002000200020004);
}

TEST(WebARKitTest, InitTrackerTest) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Init the manager with the Akaze tracker
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, 640, 480);
  // Load the test image
  cv::Mat image = cv::imread("../pinball.jpg", cv::IMREAD_GRAYSCALE);

  if(image.data == NULL) {
    std::cout << "Something wrong while reading the image!" << std::endl;
  }
  
  if(image.empty()) {
    image = cv::Mat::zeros(2048, 1637,  CV_8UC4);
  }

  ASSERT_FALSE(image.empty());

  int width = image.cols;
  int height = image.rows;
  unsigned char* data = image.data;
  EXPECT_EQ(image.cols, 1637);
  EXPECT_EQ(image.rows, 2048);
  // Check if initTracker returns sucessfully
  EXPECT_TRUE(manager.initTracker(data, width, height, webarkit::ColorSpace::GRAY));
}

TEST(WebARKitTest, InitTrackerTest2) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Init the manager with the Akaze tracker
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, 640, 480);
  // Load the test image
  cv::Mat image = cv::imread("../pinball.jpg", cv::IMREAD_GRAYSCALE);

  if(image.data == NULL) {
    std::cout << "Something wrong while reading the image!" << std::endl;
  }

  if(image.empty()) {
    image = cv::Mat::zeros(2048, 1637,  CV_8UC4);
  }

  ASSERT_FALSE(image.empty());

  int width = image.cols;
  int height = image.rows;
  //unsigned char* data = image.data;
  EXPECT_EQ(image.cols, 1637);
  EXPECT_EQ(image.rows, 2048);
  // Check if initTracker returns sucessfully
  EXPECT_TRUE(manager.initTracker(image, width, height, webarkit::ColorSpace::GRAY));
}

// WebARKitLib#25: cv::imread decodes to BGR, so a native caller can now hand the
// reference image straight to initTracker without converting it first.
TEST(WebARKitTest, InitTrackerBGRTest) {
  webarkit::WebARKitManager manager;
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, 640, 480);
  cv::Mat image = cv::imread("../pinball.jpg", cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  ASSERT_EQ(image.type(), CV_8UC3);
  EXPECT_TRUE(manager.initTracker(image.data, image.cols, image.rows, webarkit::ColorSpace::BGR));
}


// Check WebARKit version
TEST(WebARKitTest, CheckShutDown) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Init the manager with the Akaze tracker
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, 640, 480);
  // Check if the WebARKit went down successfully
  EXPECT_TRUE(manager.shutdown());
}

// WebARKitLib#91: a frame larger than featureImageMinSize (640x480) is detected on a pyrDown'd
// copy, and a frame that copy did not detect is retried at full resolution (#53). With no
// marker in view that retry used to run on every frame, doubling the cost of searching.
namespace {

// 1280x960 is downsampled once, so the full-resolution retry exists.
constexpr int kBigW = 1280;
constexpr int kBigH = 960;

// A deterministic texture with no marker in it: it has features, but nothing that matches.
cv::Mat noFrame() {
  cv::RNG rng(12345);
  cv::Mat m(kBigH, kBigW, CV_8UC1);
  rng.fill(m, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(m, m, cv::Size(5, 5), 0);
  return m;
}

// The pinball reference scaled by `scale` and pasted in the middle of a mid-grey frame. At 0.15
// the downsampled pass finds 14 matches, one below the floor of 15 for AKAZE, while the
// full-resolution pass finds 76: the small marker of #53.
cv::Mat frameWithMarker(const cv::Mat &reference, double scale) {
  cv::Mat frame(kBigH, kBigW, CV_8UC1, cv::Scalar(128));
  cv::Mat small;
  cv::resize(reference, small, cv::Size(), scale, scale, cv::INTER_AREA);
  small.copyTo(frame(cv::Rect((kBigW - small.cols) / 2, (kBigH - small.rows) / 2, small.cols, small.rows)));
  return frame;
}

struct BigFrameTracker {
  webarkit::WebARKitManager manager;
  cv::Mat reference;

  BigFrameTracker() {
    // Every detection draws from the library's global generator; start each test from the same
    // state so that what ran before it in the process cannot change which points are tracked.
    rng = cv::RNG(0xFFFFFFFF);
    reference = cv::imread("../pinball.jpg", cv::IMREAD_GRAYSCALE);
    manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, kBigW, kBigH);
    if (!reference.empty()) {
      manager.initTracker(reference, reference.cols, reference.rows, webarkit::ColorSpace::GRAY);
    }
  }

  void feed(const cv::Mat &frame) {
    manager.processFrameData(frame.data, frame.cols, frame.rows, webarkit::ColorSpace::GRAY,
                             webarkit::BLUR_TYPE::NONE_BLUR);
  }
  int retries() { return manager.getTracker()->getFullResRetryCount(); }
};

}  // namespace

TEST(WebARKitFullResRetryTest, FramesWithNoMarkerDoNotRetryEveryTime) {
  BigFrameTracker t;
  ASSERT_FALSE(t.reference.empty());
  const cv::Mat frame = noFrame();
  for (int i = 0; i < 6; i++) t.feed(frame);
  EXPECT_FALSE(t.manager.isValid());
  // Once at the start and then every third frame (frames 0 and 3), not on all six.
  EXPECT_EQ(t.retries(), 2);
}

// The small marker of #53 must still be found: the pyramid pass fails, the full-resolution
// retry detects it, and optical flow then holds it.
TEST(WebARKitFullResRetryTest, ASmallMarkerIsStillFoundAtFullResolution) {
  BigFrameTracker t;
  ASSERT_FALSE(t.reference.empty());
  const cv::Mat frame = frameWithMarker(t.reference, 0.15);
  for (int i = 0; i < 3; i++) t.feed(frame);
  EXPECT_TRUE(t.manager.isValid());
  EXPECT_GE(t.retries(), 1);  // it was the full-resolution pass that found it
}

// A pyramid pass that came close (here 14 matches against a floor of 15) is a signal that a
// marker is there: the retry runs at once, however many empty frames came before it.
TEST(WebARKitFullResRetryTest, ASignalOnThePyramidPassRetriesAtOnce) {
  BigFrameTracker t;
  ASSERT_FALSE(t.reference.empty());
  const cv::Mat empty = noFrame();
  const cv::Mat withMarker = frameWithMarker(t.reference, 0.15);
  t.feed(empty);       // the retry of the first empty frame; the next two are skipped
  t.feed(withMarker);  // the pyramid pass has a signal, so the retry is not skipped
  t.feed(withMarker);
  EXPECT_TRUE(t.manager.isValid());
  // One retry for the empty frame and one for each marker frame: a cadence alone would skip
  // the marker frames and find nothing.
  EXPECT_EQ(t.retries(), 3);
}

// The smallest marker the full-resolution pass still finds (0.12): the downsampled pass has 3
// matches, no signal, so only the retry detects it. Detecting it must restart the cadence, or
// the next frame skips its retry and the marker is found two frames late. Before the cadence
// it was valid after four frames; it must not take longer.
TEST(WebARKitFullResRetryTest, AMarkerOnlyTheRetryFindsIsNotFoundLater) {
  BigFrameTracker t;
  ASSERT_FALSE(t.reference.empty());
  const cv::Mat frame = frameWithMarker(t.reference, 0.12);
  for (int i = 0; i < 4; i++) t.feed(frame);
  EXPECT_TRUE(t.manager.isValid());
}

// The cadence belongs to a target: a new reference image restarts it. Two empty frames leave the
// counter at 2 (the next retry would be a frame away), and loading a target must not inherit that.
TEST(WebARKitFullResRetryTest, ANewTargetRestartsTheCadence) {
  BigFrameTracker t;
  ASSERT_FALSE(t.reference.empty());
  const cv::Mat empty = noFrame();
  t.feed(empty);  // retries, the counter moves on
  t.feed(empty);  // skipped
  ASSERT_EQ(t.retries(), 1);

  t.manager.initTracker(t.reference, t.reference.cols, t.reference.rows, webarkit::ColorSpace::GRAY);
  t.feed(frameWithMarker(t.reference, 0.12));  // no pyramid signal: only the cadence decides
  EXPECT_EQ(t.retries(), 2);  // retried at once; without the reset it would still be 1
}
