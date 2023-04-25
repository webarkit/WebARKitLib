#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>

namespace webarkit {

static cv::Mat im_gray(uchar data[], size_t cols, size_t rows) {
  uint32_t idx;
  uchar gray[rows][cols];
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      idx = (i * cols * 4) + j * 4;

      // rgba to rgb
      uchar r = data[idx];
      uchar g = data[idx + 1];
      uchar b = data[idx + 2];
      // uchar a = data[idx + 3];

      // turn frame image to gray scale
      gray[i][j] = (0.30 * r) + (0.59 * g) + (0.11 * b);
    }
  }

  return cv::Mat(rows, cols, CV_8UC1, gray);
}

static cv::Mat grayscale(uchar data[], size_t cols, size_t rows,
                         ColorSpace colorType) {
  int cn;
  switch (colorType) {
  case ColorSpace::RGBA:
    cn = 4;
    break;
  case ColorSpace::RGB:
    cn = 3;
    break;
  case ColorSpace::GRAY:

    std::cout << "Grayscale input is not allowed with grayscale !" << std::endl;

    break;
  default:
    cn = 4;
  }
  auto size = cols * rows;
  auto q = 0;
  std::vector<uchar> gray;
  uchar r;
  uchar g;
  uchar b;
  for (auto p = 0; p < size; p++) {
    r = data[q + 0], g = data[q + 1], b = data[q + 2];
    // https://stackoverflow.com/a/596241/5843642
    gray.push_back((r + r + r + b + g + g + g + g) >> 3);
    q += cn;
  }
  return cv::Mat(cols, rows, CV_8UC1, gray.data());
}

} // namespace webarkit