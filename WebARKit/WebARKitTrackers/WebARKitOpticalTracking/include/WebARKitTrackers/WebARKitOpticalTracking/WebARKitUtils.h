#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <iostream>

namespace webarkit {

#define arMalloc(V, T, S)                                                                                              \
    {                                                                                                                  \
        if (((V) = (T*)malloc(sizeof(T) * (S))) == NULL) {                                                             \
            std::cout << "Out of memory!!" << std::endl;                                                               \
            exit(1);                                                                                                   \
        }                                                                                                              \
    }

/*static auto im_gray(uchar* data, size_t cols, size_t rows) {
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
}*/

static cv::Mat grayscale(uchar data[], size_t cols, size_t rows, ColorSpace colorType) {
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

unsigned int webarkitGetVersion(char** versionStringRef) {
    const char* version = WEBARKIT_HEADER_VERSION_STRING.c_str();
    char* s;

    if (versionStringRef) {
        arMalloc(s, char, sizeof(version));
        strncpy(s, version, sizeof(version));
        *versionStringRef = s;
    }
    // Represent full version number (major, minor, tiny, build) in
    // binary coded decimal. N.B: Integer division.
    return (0x10000000u * ((unsigned int)WEBARKIT_HEADER_VERSION_MAJOR / 10u) +
            0x01000000u * ((unsigned int)WEBARKIT_HEADER_VERSION_MAJOR % 10u) +
            0x00100000u * ((unsigned int)WEBARKIT_HEADER_VERSION_MINOR / 10u) +
            0x00010000u * ((unsigned int)WEBARKIT_HEADER_VERSION_MINOR % 10u) +
            0x00001000u * ((unsigned int)WEBARKIT_HEADER_VERSION_TINY / 10u) +
            0x00000100u * ((unsigned int)WEBARKIT_HEADER_VERSION_TINY % 10u) +
            0x00000010u * ((unsigned int)WEBARKIT_HEADER_VERSION_DEV / 10u) +
            0x00000001u * ((unsigned int)WEBARKIT_HEADER_VERSION_DEV % 10u));
}

} // namespace webarkit