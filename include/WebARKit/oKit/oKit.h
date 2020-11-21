/*
 *  oKit.h
 *  This file is part of WebARKitLib.
 *  Copyright 2020 WebARKit.
 *  Author(s): Walter Perdan, Edward Lu.
 *
 */
#ifndef OKIT_H
#define OKIT_H

#include <iostream>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;

#ifdef __cplusplus
extern "C" {
#endif

#define GOOD_MATCH_RATIO    0.7f
#define MAX_FEATURES        2000
#define N                   10

bool initialized = false;

Ptr<ORB> orb = NULL;
Ptr<BFMatcher> matcher = NULL;

Mat refGray, refDescr;
std::vector<KeyPoint> refKeyPts;

Mat H;
std::vector<Point2f> corners(4);

Mat framePrev;
int numMatches = 0;
std::vector<Point2f> framePts;

double *output = new double[17]; // 9 from homography matrix, 8 from warped corners

static Mat im_gray(uchar data[], size_t cols, size_t rows);

static inline bool homographyValid(Mat H);

static inline void fill_output(Mat H);

static inline void clear_output();

int initAR(uchar refData[], size_t refCols, size_t refRows);

double *resetTracking(uchar frameData[], size_t frameCols, size_t frameRows);

double *track(uchar frameData[], size_t frameCols, size_t frameRows);

#ifdef __cplusplus
}
#endif
#endif
