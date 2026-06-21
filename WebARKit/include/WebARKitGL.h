#ifndef WEBARKIT_GL_H
#define WEBARKIT_GL_H

#include <opencv2/core.hpp>

namespace webarkit {

void arglCameraViewRHf(float para[3][4], float m_modelview[16], const float scale);

void arglCameraViewRHf(cv::Mat para, std::array<double, 16>& m_modelview, const double scale);

void cameraProjectionMatrix(const std::array<double, 9>& calibration, double nearPlane, double farPlane, int screenWidth, int screenHeight, std::array<double, 16>& projectionMatrix);
} // namespace webarkit

#endif // WEBARKITGL_H