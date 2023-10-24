#include <gtest/gtest.h>
#include <WebARKitManager.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>

class WebARKitEnumTest : public testing::TestWithParam<webarkit::TRACKER_TYPE> {};

TEST_P(WebARKitEnumTest, TestEnumValues) {
  webarkit::TRACKER_TYPE value = GetParam();
  EXPECT_TRUE(value == webarkit::TRACKER_TYPE::AKAZE_TRACKER ||
              value == webarkit::TRACKER_TYPE::ORB_TRACKER ||
              value == webarkit::TRACKER_TYPE::FREAK_TRACKER);
}

INSTANTIATE_TEST_SUITE_P(WebARKitEnumTestSuite, WebARKitEnumTest,
                         testing::ValuesIn({webarkit::TRACKER_TYPE::AKAZE_TRACKER,
                                            webarkit::TRACKER_TYPE::ORB_TRACKER,
                                            webarkit::TRACKER_TYPE::FREAK_TRACKER}));


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