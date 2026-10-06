#include <gtest/gtest.h>

#include <WebARKitTrackers/WebARKitNFT/NFTMarkerState.h>
#include <WebARKitTrackers/WebARKitNFT/markerDecompress.h>
#include <WebARKitTrackers/WebARKitNFT/trackingMod.h>
#ifdef WEBARKIT_NFT_THREADS
#include <WebARKitTrackers/WebARKitNFT/trackingSub.h>
#endif

#include <zlib.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

namespace {

// A .zft is a zlib-compressed JSON-like string: {"iset":"...","fset":"...","fset3":"..."}
void writeZft(const std::string &basename, const std::string &content) {
  uLongf compressedSize = compressBound(content.size());
  std::string compressed(compressedSize, '\0');
  ASSERT_EQ(compress(reinterpret_cast<Bytef *>(&compressed[0]), &compressedSize,
                     reinterpret_cast<const Bytef *>(content.data()), content.size()),
            Z_OK);
  std::ofstream out(basename + ".zft", std::ios::binary);
  out.write(compressed.data(), compressedSize);
}

std::string readFile(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

} // namespace

TEST(MarkerDecompressTest, ExtractsIsetFsetAndFset3) {
  writeZft("nft_test_marker", "{\"iset\":\"ISETDATA\",\"fset\":\"FSETDATA\",\"fset3\":\"FSET3DATA\"}");

  EXPECT_EQ(decompressMarkers("nft_test_marker", "nft_test_out"), 0);
  EXPECT_EQ(readFile("nft_test_out.iset"), "ISETDATA");
  EXPECT_EQ(readFile("nft_test_out.fset"), "FSETDATA");
  EXPECT_EQ(readFile("nft_test_out.fset3"), "FSET3DATA");

  std::remove("nft_test_marker.zft");
  std::remove("nft_test_out.iset");
  std::remove("nft_test_out.fset");
  std::remove("nft_test_out.fset3");
}

TEST(MarkerDecompressTest, MissingFileReturnsError) {
  EXPECT_EQ(decompressMarkers("does_not_exist", "nft_test_out"), -1);
}

TEST(MarkerDecompressTest, MalformedContentReturnsError) {
  writeZft("nft_test_bad", "{\"iset\":\"ISETDATA\"}");
  EXPECT_EQ(decompressMarkers("nft_test_bad", "nft_test_out"), -1);
  std::remove("nft_test_bad.zft");
}

TEST(NFTMarkerStateTest, DefaultsToNotTracking) {
  NFTMarkerState state;
  EXPECT_FALSE(state.tracking);
  EXPECT_FLOAT_EQ(state.err, -1.0f);
  EXPECT_EQ(state.ftmi, nullptr);
  EXPECT_TRUE(state.filterNeedsReset);
}

TEST(TrackingModTest, CreatesAndDeletesHandle) {
  AR2HandleT *handle = ar2CreateHandleSubMod(AR_PIXEL_FORMAT_MONO, 640, 480);
  ASSERT_NE(handle, nullptr);
  EXPECT_EQ(ar2DeleteHandleMod(&handle), 0);
  EXPECT_EQ(handle, nullptr);
}

#ifdef WEBARKIT_NFT_THREADS
TEST(TrackingSubTest, StartsAndQuitsWorkerThread) {
  KpmHandle *kpmHandle = kpmCreateHandle2(640, 480);
  ASSERT_NE(kpmHandle, nullptr);
  THREAD_HANDLE_T *thread = trackingInitInit(kpmHandle);
  ASSERT_NE(thread, nullptr);
  EXPECT_EQ(trackingInitQuit(&thread), 0);
  EXPECT_EQ(thread, nullptr);
  kpmDeleteHandle(&kpmHandle);
}
#endif
