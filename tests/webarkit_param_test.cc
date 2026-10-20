#include <gtest/gtest.h>
#include <AR/ar.h>

#include <cstdio>
#include <vector>

// paramFile.c is compiled into this test at -O3 (see CMakeLists.txt). Its byte swap
// used to write ints and doubles through pointers to unions, which breaks C's
// strict-aliasing rule; GCC 13 at -O3 then returned garbage distortion factors from
// arParamLoad() (#101). These tests check every function that byte-swaps.

namespace {

const char *kCameraFile = "data/camera_para.dat";

// The values of tests/data/camera_para.dat (distortion function version 4).
const double kMat00 = 609.3654091867005;
const double kDistFactor[AR_DIST_FACTOR_NUM_MAX] = {
    0.099172286689281464, -0.092120334506034851, -0.0018825730076059699,
    0.0013129330473020673, 591.40924072265625,   588.64886474609375,
    323.44790649414062,    237.60653686523438,   0.97053300336162862,
};

void expectCameraPara(const ARParam &param) {
  EXPECT_EQ(param.xsize, 640);
  EXPECT_EQ(param.ysize, 480);
  EXPECT_EQ(param.dist_function_version, 4);
  EXPECT_DOUBLE_EQ(param.mat[0][0], kMat00);
  for (int i = 0; i < AR_DIST_FACTOR_NUM_MAX; i++) {
    EXPECT_DOUBLE_EQ(param.dist_factor[i], kDistFactor[i]) << "dist_factor[" << i << "]";
  }
}

std::vector<ARUint8> readFile(const char *path) {
  std::vector<ARUint8> bytes;
  FILE *fp = std::fopen(path, "rb");
  if (!fp) return bytes;
  int c;
  while ((c = std::fgetc(fp)) != EOF) bytes.push_back(static_cast<ARUint8>(c));
  std::fclose(fp);
  return bytes;
}

}  // namespace

TEST(ParamFileTest, LoadsTheCameraParameters) {
  ARParam param;
  ASSERT_EQ(arParamLoad(kCameraFile, 1, &param), 0);
  expectCameraPara(param);
}

TEST(ParamFileTest, LoadsTheCameraParametersFromABuffer) {
  std::vector<ARUint8> bytes = readFile(kCameraFile);
  ASSERT_FALSE(bytes.empty());
  ARParam param;
  ASSERT_EQ(arParamLoadFromBuffer(bytes.data(), bytes.size(), &param), 0);
  expectCameraPara(param);
}

TEST(ParamFileTest, SavesWhatItLoads) {
  ARParam param;
  ASSERT_EQ(arParamLoad(kCameraFile, 1, &param), 0);
  const char *copy = "camera_para_roundtrip.dat";
  ASSERT_EQ(arParamSave(copy, 1, &param), 0);
  // The saved file is the original, byte for byte.
  EXPECT_EQ(readFile(copy), readFile(kCameraFile));
  ARParam reloaded;
  ASSERT_EQ(arParamLoad(copy, 1, &reloaded), 0);
  expectCameraPara(reloaded);
  std::remove(copy);
}
