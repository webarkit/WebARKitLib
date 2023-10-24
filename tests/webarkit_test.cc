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