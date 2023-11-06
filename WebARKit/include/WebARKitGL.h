#ifndef WEBARKIT_GL_H
#define WEBARKIT_GL_H

#include <opencv2/core.hpp>

namespace webarkit {

void arglCameraViewRHf(float para[3][4], float m_modelview[16], const float scale) {
    m_modelview[0 + 0 * 4] = para[0][0]; // R1C1
    m_modelview[0 + 1 * 4] = para[0][1]; // R1C2
    m_modelview[0 + 2 * 4] = para[0][2];
    m_modelview[0 + 3 * 4] = para[0][3];
    m_modelview[1 + 0 * 4] = -para[1][0]; // R2
    m_modelview[1 + 1 * 4] = -para[1][1];
    m_modelview[1 + 2 * 4] = -para[1][2];
    m_modelview[1 + 3 * 4] = -para[1][3];
    m_modelview[2 + 0 * 4] = -para[2][0]; // R3
    m_modelview[2 + 1 * 4] = -para[2][1];
    m_modelview[2 + 2 * 4] = -para[2][2];
    m_modelview[2 + 3 * 4] = -para[2][3];
    m_modelview[3 + 0 * 4] = 0.0f;
    m_modelview[3 + 1 * 4] = 0.0f;
    m_modelview[3 + 2 * 4] = 0.0f;
    m_modelview[3 + 3 * 4] = 1.0f;
    if (scale != 0.0f) {
        m_modelview[12] *= scale;
        m_modelview[13] *= scale;
        m_modelview[14] *= scale;
    }
}

void arglCameraViewRHf(cv::Mat para, std::array<double, 16>& m_modelview, const double scale) {
    m_modelview[0 + 0 * 4] = para.at<double>(0, 0); // R1C1
    m_modelview[0 + 1 * 4] = para.at<double>(0, 1); // R1C2
    m_modelview[0 + 2 * 4] = para.at<double>(0, 2);
    m_modelview[0 + 3 * 4] = para.at<double>(0, 3);
    m_modelview[1 + 0 * 4] = -para.at<double>(1, 0); // R2
    m_modelview[1 + 1 * 4] = -para.at<double>(1, 1);
    m_modelview[1 + 2 * 4] = -para.at<double>(1, 2);
    m_modelview[1 + 3 * 4] = -para.at<double>(1, 3);
    m_modelview[2 + 0 * 4] = -para.at<double>(2, 0); // R3
    m_modelview[2 + 1 * 4] = -para.at<double>(2, 1);
    m_modelview[2 + 2 * 4] = -para.at<double>(2, 2);
    m_modelview[2 + 3 * 4] = -para.at<double>(2, 3);
    m_modelview[3 + 0 * 4] = 0.0f;
    m_modelview[3 + 1 * 4] = 0.0f;
    m_modelview[3 + 2 * 4] = 0.0f;
    m_modelview[3 + 3 * 4] = 1.0f;
    if (scale != 0.0f) {
        m_modelview[12] *= scale;
        m_modelview[13] *= scale;
        m_modelview[14] *= scale;
    }
}
} // namespace webarkit

#endif // WEBARKITGL_H