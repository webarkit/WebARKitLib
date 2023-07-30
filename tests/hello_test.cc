#include <gtest/gtest.h>
#include <WebARKitManager.h>

// Demonstrate some basic assertions.
TEST(HelloTest, BasicAssertions) {
  webarkit::WebARKitManager manager;
  manager.initialiseBase(webarkit::TRACKER_TYPE::AKAZE_TRACKER);
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);

  EXPECT_STREQ(manager.getWebARKitVersion(), "1.0.0");
}