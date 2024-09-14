#ifndef WEBARKIT_UTILS_H
#define WEBARKIT_UTILS_H

// #include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitEnums.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitHomographyInfo.h>
#include <iostream>

namespace webarkit {

static std::vector<cv::Point2f> Points(std::vector<cv::KeyPoint> keypoints) {
    std::vector<cv::Point2f> res;
    for (unsigned i = 0; i < keypoints.size(); i++) {
        res.push_back(keypoints[i].pt);
    }
    return res;
}

/// Method for calculating and validating a homography matrix from a set of corresponding points.
/// pts1 and pts must have the same dimensionality.
/// @returns An WebARKitHomographyInfo instance, with its status vector of the same dimensionality as the pts1 and pts2
/// vectors.
static homography::WebARKitHomographyInfo getHomographyInliers(std::vector<cv::Point2f> pts1,
                                                               std::vector<cv::Point2f> pts2) {
    if (pts1.size() < 4) {
        return homography::WebARKitHomographyInfo();
    }

    cv::Mat inlier_mask, homography;
    homography = findHomography(pts1, pts2, cv::RANSAC, ransac_thresh, inlier_mask);
    if (homography.empty()) {
        // Failed to find a homography.
        return homography::WebARKitHomographyInfo();
    }

    const double det = homography.at<double>(0, 0) * homography.at<double>(1, 1) -
                       homography.at<double>(1, 0) * homography.at<double>(0, 1);
    if (det < 0) {
        return homography::WebARKitHomographyInfo();
    }

    const double N1 = sqrt(homography.at<double>(0, 0) * homography.at<double>(0, 0) +
                           homography.at<double>(1, 0) * homography.at<double>(1, 0));
    if (N1 > 4 || N1 < 0.1) {
        return homography::WebARKitHomographyInfo();
    }

    const double N2 = sqrt(homography.at<double>(0, 1) * homography.at<double>(0, 1) +
                           homography.at<double>(1, 1) * homography.at<double>(1, 1));
    if (N2 > 4 || N2 < 0.1) {
        return homography::WebARKitHomographyInfo();
    }

    const double N3 = sqrt(homography.at<double>(2, 0) * homography.at<double>(2, 0) +
                           homography.at<double>(2, 1) * homography.at<double>(2, 1));
    if (N3 > 0.002) {
        return homography::WebARKitHomographyInfo();
    }

    std::vector<uchar> status;
    std::vector<cv::DMatch> inlier_matches;
    int linliers = 0;
    for (int i = 0; i < pts1.size(); i++) {
        if ((int)inlier_mask.at<uchar>(i, 0) == 1) {
            status.push_back((uchar)1);
            inlier_matches.push_back(cv::DMatch(i, i, 0));
            linliers++;
        } else {
            status.push_back((uchar)0);
        }
    }
    // Return homography and corresponding inlier point sets
    return homography::WebARKitHomographyInfo(homography, status, inlier_matches);
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

static auto convert2Grayscale(cv::Mat& refData, size_t refCols, size_t refRows, ColorSpace colorSpace) {
    cv::Mat refGray;

    switch (colorSpace) {
    case ColorSpace::RGBA: {
        refGray.create(refRows, refCols, CV_8UC1);
        cv::cvtColor(refData, refGray, cv::COLOR_RGBA2GRAY);
    } break;
    case ColorSpace::RGB: {
        refGray.create(refRows, refCols, CV_8UC1);
        cv::cvtColor(refData, refGray, cv::COLOR_RGB2GRAY);
    } break;
    case ColorSpace::GRAY: {
        refGray = refData;
    } break;
    default: {
        refGray = refData;
    }
    }

    return refGray;
}

static auto convert2Grayscale(uchar* refData, size_t refCols, size_t refRows, ColorSpace colorSpace) {
    cv::Mat refGray;

    switch (colorSpace) {
    case ColorSpace::RGBA: {
        cv::Mat colorFrame(refRows, refCols, CV_8UC4, refData);
        refGray.create(refRows, refCols, CV_8UC1);
        cv::cvtColor(colorFrame, refGray, cv::COLOR_RGBA2GRAY);
        WEBARKIT_LOGd("convert to GRAY from RGBA !!\n");
    } break;
    case ColorSpace::RGB: {
        cv::Mat colorFrame(refRows, refCols, CV_8UC3, refData);
        refGray.create(refRows, refCols, CV_8UC1);
        cv::cvtColor(colorFrame, refGray, cv::COLOR_RGB2GRAY);
        WEBARKIT_LOGd("convert to GRAY from RGB !!\n");
    } break;
    case ColorSpace::GRAY: {
        refGray = cv::Mat(refRows, refCols, CV_8UC1, refData);
        WEBARKIT_LOGd("no need to convert to GRAY!!\n");
    } break;
    default: {
        refGray = cv::Mat(refRows, refCols, CV_8UC1, refData);
        WEBARKIT_LOGd("Default: no need to convert to GRAY!!\n");
    }
    }

    return refGray;
}

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

std::string inline webarkitGetVersion() { return WEBARKIT_HEADER_VERSION_STRING; }

unsigned int inline webarkitGetVersion(char** versionStringRef) {
    std::string version = WEBARKIT_HEADER_VERSION_STRING;

    if (versionStringRef) {
        *versionStringRef = const_cast<char*>(version.data());
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

#endif // WEBARKIT_UTILS_H