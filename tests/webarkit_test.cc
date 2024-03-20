#include <gtest/gtest.h>
#include <WebARKitManager.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>
#include <WebARKitCamera.h>
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
              color_value == webarkit::ColorSpace::GRAY);
}

INSTANTIATE_TEST_SUITE_P(WebARKitEnumTestSuite, WebARKitEnumTest,
                         testing::Combine(testing::ValuesIn({webarkit::TRACKER_TYPE::AKAZE_TRACKER,
                                                             webarkit::TRACKER_TYPE::ORB_TRACKER,
                                                             webarkit::TRACKER_TYPE::FREAK_TRACKER,
                                                             webarkit::TRACKER_TYPE::TEBLID_TRACKER}),
                                          testing::ValuesIn({webarkit::ColorSpace::RGB,
                                                             webarkit::ColorSpace::RGBA,
                                                             webarkit::ColorSpace::GRAY})));
                                                          
TEST(WebARKitConfigTest, TestConfigValues) {
  EXPECT_EQ(DEFAULT_NN_MATCH_RATIO, 0.7f);
  EXPECT_EQ(TEBLID_NN_MATCH_RATIO, 0.8f);
  EXPECT_EQ(DEFAULT_MAX_FEATURES, 8000);
  EXPECT_EQ(TEBLID_MAX_FEATURES, 10000);
  EXPECT_EQ(N, 10);
  EXPECT_EQ(MIN_NUM_MATCHES, 8);
  EXPECT_EQ(maxLevel, 3);
  EXPECT_EQ(featureDetectPyramidLevel, 1.05f);
  EXPECT_EQ(featureBorder, 8);
  EXPECT_EQ(WEBARKIT_HEADER_VERSION_STRING, "1.0.0");
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
  EXPECT_EQ(projectionMatrix[0], -1.7851850084276433);
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
  EXPECT_STREQ(manager.getWebARKitVersion().c_str(), "1.0.0");
}

// Check cameraProjectionMatrix from manager
TEST(WebARKitTest, CheckCameraProjectionMatrix) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Init the manager with the Akaze tracker
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, 640, 480);
  // Check if the cameraProjectionMatrix is correct
  std::array<double, 16> camProjectionMatrix = manager.getCameraProjectionMatrix();
  EXPECT_EQ(camProjectionMatrix[0], -1.7851850084276433);
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


// Check WebARKit version
TEST(WebARKitTest, CheckShutDown) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Init the manager with the Akaze tracker
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER, 640, 480);
  // Check if the WebARKit went down successfully
  EXPECT_TRUE(manager.shutdown());
}