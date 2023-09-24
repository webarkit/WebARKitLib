#include <gtest/gtest.h>
#include <WebARKitManager.h>

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