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

#ifdef __cplusplus
extern "C" {
#endif

int initAR(uchar refData[], size_t refCols, size_t refRows);

double *resetTracking(uchar frameData[], size_t frameCols, size_t frameRows);

double *track(uchar frameData[], size_t frameCols, size_t frameRows);

#ifdef __cplusplus
}
#endif
#endif
