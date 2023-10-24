#include <gtest/gtest.h>
#include <WebARKitManager.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>

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
  EXPECT_EQ(MIN_NUM_MATCHES, 50);
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

// Check WebARKitManager initialisation.
TEST(WebARKitTest, InitialiseBaseTest) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Check if the WebARKitManager initialisation is successful
  EXPECT_TRUE(manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER));
}

// Check WebARKit version
TEST(WebARKitTest, CheckWebARKitVersion) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Init the manager with the Akaze tracker
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER);
  // Check if the WebARKit version is correct
  EXPECT_STREQ(manager.getWebARKitVersion().c_str(), "1.0.0");
}