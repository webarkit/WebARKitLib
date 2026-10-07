// markerDecompress compiled with MARKER_DECOMPRESS_MAX_SIZE = 1 MB (see CMakeLists.txt).
#include <gtest/gtest.h>

#include <WebARKitTrackers/WebARKitNFT/markerDecompress.h>

#include <zlib.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

static_assert(MARKER_DECOMPRESS_MAX_SIZE == 1024 * 1024, "this test expects a 1 MB limit");

namespace {

const std::string kHead = "{\"iset\":\"";
const std::string kTail = "\",\"fset\":\"F\",\"fset3\":\"G\"}";

// A marker whose decompressed size is exactly `size` bytes. The data is
// flushed before the final (empty) block, so zlib still has a block and the
// trailer to read when the output reaches `size`.
std::string markerOfSize(size_t size) {
  return kHead + std::string(size - kHead.size() - kTail.size(), 'A') + kTail;
}

void writeZftFlushedBeforeEnd(const std::string &basename, const std::string &content) {
  z_stream strm = {};
  ASSERT_EQ(deflateInit(&strm, Z_BEST_COMPRESSION), Z_OK);
  std::string compressed;
  std::vector<unsigned char> out(64 * 1024);
  auto run = [&](int flush) {
    do {
      strm.next_out = out.data();
      strm.avail_out = static_cast<uInt>(out.size());
      deflate(&strm, flush);
      compressed.append(reinterpret_cast<char *>(out.data()), out.size() - strm.avail_out);
    } while (strm.avail_out == 0);
  };
  strm.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(content.data()));
  strm.avail_in = static_cast<uInt>(content.size());
  run(Z_FULL_FLUSH);
  run(Z_FINISH);
  deflateEnd(&strm);
  std::ofstream file(basename + ".zft", std::ios::binary);
  file.write(compressed.data(), compressed.size());
}

bool fileExists(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  return in.good();
}

void removeOutputs(const std::string &out) {
  std::remove((out + ".iset").c_str());
  std::remove((out + ".fset").c_str());
  std::remove((out + ".fset3").c_str());
}

} // namespace

TEST(MarkerDecompressLimitTest, AcceptsArchiveExactlyAtTheLimit) {
  writeZftFlushedBeforeEnd("nft_limit_exact", markerOfSize(MARKER_DECOMPRESS_MAX_SIZE));
  EXPECT_EQ(decompressMarkers("nft_limit_exact", "nft_limit_exact_out"), 0);
  EXPECT_TRUE(fileExists("nft_limit_exact_out.fset3"));
  std::remove("nft_limit_exact.zft");
  removeOutputs("nft_limit_exact_out");
}

TEST(MarkerDecompressLimitTest, RejectsArchiveOneByteOverTheLimit) {
  writeZftFlushedBeforeEnd("nft_limit_over", markerOfSize(MARKER_DECOMPRESS_MAX_SIZE + 1));
  EXPECT_EQ(decompressMarkers("nft_limit_over", "nft_limit_over_out"), -1);
  EXPECT_FALSE(fileExists("nft_limit_over_out.iset"));
  std::remove("nft_limit_over.zft");
}

TEST(MarkerDecompressLimitTest, EnforcesLimitsBelowTheInitialBuffer) {
  // 2 MB is below the 4 MB initial buffer, but above this build's 1 MB limit.
  writeZftFlushedBeforeEnd("nft_limit_small", markerOfSize(2 * 1024 * 1024));
  EXPECT_EQ(decompressMarkers("nft_limit_small", "nft_limit_small_out"), -1);
  EXPECT_FALSE(fileExists("nft_limit_small_out.iset"));
  std::remove("nft_limit_small.zft");
}
