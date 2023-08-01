#include <gtest/gtest.h>
#include <WebARKitManager.h>

// Check WebARKitManager initialisation.
TEST(InitTest, InitialiseBaseTest) {
  // Create a WebARKitManager object
  webarkit::WebARKitManager manager;
  // Check if the WebARKitManager initialisation is successful
  EXPECT_TRUE(manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER));
}