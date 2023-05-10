#include "WebARKitEnums.h"
#include "HomographyInfo.h"

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

std::vector<cv::Point2f> Points(std::vector<cv::KeyPoint> keypoints)
{
    std::vector<cv::Point2f> res;
    for(unsigned i = 0; i < keypoints.size(); i++) {
        res.push_back(keypoints[i].pt);
    }
    return res;
}

//Method for calculating and validating a homography matrix from a set of corresponding points.
HomographyInfo GetHomographyInliers(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2)
{
    cv::Mat inlier_mask, homography;
    std::vector<cv::DMatch> inlier_matches;
    if(pts1.size() >= 4) {
        homography = findHomography(pts1, pts2,
                                    cv::RANSAC, ransac_thresh, inlier_mask);
    }
    
    //Failed to find a homography
    if(pts1.size() < 4 || homography.empty()) {
        return HomographyInfo();
    }
    
    const double det = homography.at<double>(0, 0) * homography.at<double>(1, 1) - homography.at<double>(1, 0) * homography.at<double>(0, 1);
    if (det < 0)
        return HomographyInfo();
    
    const double N1 = sqrt(homography.at<double>(0, 0) * homography.at<double>(0, 0) + homography.at<double>(1, 0) * homography.at<double>(1, 0));
    if (N1 > 4 || N1 < 0.1)
        return HomographyInfo();
    
    const double N2 = sqrt(homography.at<double>(0, 1) * homography.at<double>(0, 1) + homography.at<double>(1, 1) * homography.at<double>(1, 1));
    if (N2 > 4 || N2 < 0.1)
        return HomographyInfo();
    
    const double N3 = sqrt(homography.at<double>(2, 0) * homography.at<double>(2, 0) + homography.at<double>(2, 1) * homography.at<double>(2, 1));
    if (N3 > 0.002)
        return HomographyInfo();
    
    std::vector<uchar> status;
    int linliers = 0;
    for(int i = 0; i < pts1.size(); i++) {
        if((int)inlier_mask.at<uchar>(i,0)==1) {
            status.push_back((uchar)1);
            inlier_matches.push_back(cv::DMatch(i, i, 0));
            linliers++;
        }
        else {
            status.push_back((uchar)0);
        }
    }
    //Return homography and corresponding inlier point sets
    return HomographyInfo(homography, status, inlier_matches);
}


} // namespace webarkit