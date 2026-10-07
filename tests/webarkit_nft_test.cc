#include <gtest/gtest.h>

#include <WebARKitTrackers/WebARKitNFT/NFTMarkerState.h>
#include <WebARKitTrackers/WebARKitNFT/markerDecompress.h>
#include <WebARKitTrackers/WebARKitNFT/trackingMod.h>
#ifdef WEBARKIT_NFT_THREADS
#include <WebARKitTrackers/WebARKitNFT/trackingSub.h>
#endif

#include <zlib.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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

bool fileExists(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  return in.good();
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

TEST(MarkerDecompressTest, ArchiveExpandingPastTheLimitReturnsError) {
  // Deflate MARKER_DECOMPRESS_MAX_SIZE + 1 MB of a valid-looking marker in
  // chunks, so the test never holds the expanded data in memory.
  z_stream strm = {};
  ASSERT_EQ(deflateInit(&strm, Z_BEST_COMPRESSION), Z_OK);
  std::string compressed;
  std::vector<unsigned char> out(64 * 1024);
  auto feed = [&](const std::string &data, int flush) {
    strm.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(data.data()));
    strm.avail_in = static_cast<uInt>(data.size());
    do {
      strm.next_out = out.data();
      strm.avail_out = static_cast<uInt>(out.size());
      deflate(&strm, flush);
      compressed.append(reinterpret_cast<char *>(out.data()), out.size() - strm.avail_out);
    } while (strm.avail_out == 0);
  };
  const std::string chunk(1024 * 1024, 'A');
  feed("{\"iset\":\"", Z_NO_FLUSH);
  for (size_t i = 0; i < MARKER_DECOMPRESS_MAX_SIZE / chunk.size() + 1; i++) feed(chunk, Z_NO_FLUSH);
  feed("\",\"fset\":\"F\",\"fset3\":\"G\"}", Z_FINISH);
  deflateEnd(&strm);
  {
    std::ofstream file("nft_test_huge.zft", std::ios::binary);
    file.write(compressed.data(), compressed.size());
  }

  EXPECT_EQ(decompressMarkers("nft_test_huge", "nft_test_huge_out"), -1);
  EXPECT_FALSE(fileExists("nft_test_huge_out.iset"));
  std::remove("nft_test_huge.zft");
}

TEST(MarkerDecompressTest, MalformedContentReturnsError) {
  writeZft("nft_test_bad", "{\"iset\":\"ISETDATA\"}");
  EXPECT_EQ(decompressMarkers("nft_test_bad", "nft_test_out"), -1);
  std::remove("nft_test_bad.zft");
}

TEST(MarkerDecompressTest, ExtractsMarkersLargerThanFourMegabytes) {
  const std::string iset(5 * 1024 * 1024, 0x41);
  writeZft("nft_test_big", "{\"iset\":\"" + iset + "\",\"fset\":\"F\",\"fset3\":\"G\"}");

  EXPECT_EQ(decompressMarkers("nft_test_big", "nft_test_big_out"), 0);
  EXPECT_EQ(readFile("nft_test_big_out.iset"), iset);
  EXPECT_EQ(readFile("nft_test_big_out.fset3"), "G");

  std::remove("nft_test_big.zft");
  std::remove("nft_test_big_out.iset");
  std::remove("nft_test_big_out.fset");
  std::remove("nft_test_big_out.fset3");
}

TEST(MarkerDecompressTest, KeepsBinaryBytesUnchanged) {
  const std::string bytes("A\nB\r\nC\x1a" "D", 8);
  writeZft("nft_test_bin", "{\"iset\":\"" + bytes + "\",\"fset\":\"F\",\"fset3\":\"G\"}");

  EXPECT_EQ(decompressMarkers("nft_test_bin", "nft_test_bin_out"), 0);
  EXPECT_EQ(readFile("nft_test_bin_out.iset"), bytes);

  std::remove("nft_test_bin.zft");
  std::remove("nft_test_bin_out.iset");
  std::remove("nft_test_bin_out.fset");
  std::remove("nft_test_bin_out.fset3");
}

TEST(MarkerDecompressTest, FieldsOutOfOrderReturnErrorAndWriteNothing) {
  writeZft("nft_test_order", "{\"iset\":\"I\",\"fset3\":\"G\",\"fset\":\"F\"}");
  EXPECT_EQ(decompressMarkers("nft_test_order", "nft_test_order_out"), -1);
  EXPECT_FALSE(fileExists("nft_test_order_out.iset"));
  std::remove("nft_test_order.zft");
}

TEST(MarkerDecompressTest, NonZlibDataReturnsError) {
  {
    std::ofstream out("nft_test_raw.zft", std::ios::binary);
    out << "{\"iset\":\"I\",\"fset\":\"F\",\"fset3\":\"G\"}";
  }
  EXPECT_EQ(decompressMarkers("nft_test_raw", "nft_test_raw_out"), -1);
  EXPECT_FALSE(fileExists("nft_test_raw_out.iset"));
  std::remove("nft_test_raw.zft");
}

TEST(MarkerDecompressTest, KeepsTheSourceArchive) {
  writeZft("nft_test_keep", "{\"iset\":\"I\",\"fset\":\"F\",\"fset3\":\"G\"}");
  EXPECT_EQ(decompressMarkers("nft_test_keep", "nft_test_keep_out"), 0);
  EXPECT_TRUE(fileExists("nft_test_keep.zft"));
  std::remove("nft_test_keep.zft");
  std::remove("nft_test_keep_out.iset");
  std::remove("nft_test_keep_out.fset");
  std::remove("nft_test_keep_out.fset3");
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

TEST(TrackingSubTest, RejectsStartUntilResultsAreCollected) {
  KpmHandle *kpmHandle = kpmCreateHandle2(64, 48);
  ASSERT_NE(kpmHandle, nullptr);
  THREAD_HANDLE_T *thread = trackingInitInit(kpmHandle);
  ASSERT_NE(thread, nullptr);
  std::vector<ARUint8> image(64 * 48, 128);

  EXPECT_EQ(trackingInitStart(thread, image.data()), 0);
  EXPECT_EQ(trackingInitStart(thread, image.data()), -1);

  TrackingInitResult results[TRACKING_INIT_MAX_RESULTS];
  int resultNum = 0;
  int ret;
  while ((ret = trackingInitGetResults(thread, results, TRACKING_INIT_MAX_RESULTS, &resultNum)) == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(ret, 1);
  EXPECT_EQ(trackingInitStart(thread, image.data()), 0);

  while (trackingInitGetResults(thread, results, TRACKING_INIT_MAX_RESULTS, &resultNum) == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(trackingInitQuit(&thread), 0);
  kpmDeleteHandle(&kpmHandle);
}
#endif
