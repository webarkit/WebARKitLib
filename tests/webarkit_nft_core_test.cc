#include <gtest/gtest.h>

#include <WebARKitLog.h>
#include <WebARKitTrackers/WebARKitNFT/NFTTrackingConfig.h>

TEST(NFTTrackingConfigTest, SingleThreadPresetMatchesTheSingleThreadBinding) {
  const NFTTrackingConfig config = singleThreadPreset();
  EXPECT_EQ(config.detector, NFTTrackingConfig::Detector::Sync);
  EXPECT_EQ(config.ar2Variant, NFTTrackingConfig::AR2Variant::SingleThread);
  EXPECT_FALSE(config.cpuDependentSearchSize);
  EXPECT_EQ(config.defaultDetectionIntervalMs, 300.0);
  EXPECT_TRUE(config.poseFilteringSupported);
  EXPECT_EQ(config.clock, &nftDefaultClockMs);
}

TEST(NFTTrackingConfigTest, ThreadedPresetMatchesTheThreadedBinding) {
  const NFTTrackingConfig config = threadedPreset();
  EXPECT_EQ(config.detector, NFTTrackingConfig::Detector::Threaded);
  EXPECT_EQ(config.ar2Variant, NFTTrackingConfig::AR2Variant::Threaded);
  EXPECT_TRUE(config.cpuDependentSearchSize);
  EXPECT_EQ(config.defaultDetectionIntervalMs, 0.0);
  EXPECT_FALSE(config.poseFilteringSupported);
  EXPECT_EQ(config.clock, &nftDefaultClockMs);
}

TEST(NFTTrackingConfigTest, LoggerIsNative) {
  WEBARKIT_LOGi("core logger test");
  SUCCEED();
}
