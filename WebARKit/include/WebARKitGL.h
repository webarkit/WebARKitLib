#ifndef WEBARKITGL_H
#define WEBARKITGL_H

#include <opencv2/core.h>

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

void arglCameraViewRHf(cv::Mat para, std::array<float, 16>& m_modelview, const float scale) {
    m_modelview[0 + 0 * 4] = para.at(0, 0); // R1C1
    m_modelview[0 + 1 * 4] = para.at(0, 1); // R1C2
    m_modelview[0 + 2 * 4] = para.at(0, 2);
    m_modelview[0 + 3 * 4] = para.at(0, 3);
    m_modelview[1 + 0 * 4] = -para.at(1, 0); // R2
    m_modelview[1 + 1 * 4] = -para.at(1, 1);
    m_modelview[1 + 2 * 4] = -para.at(1, 2);
    m_modelview[1 + 3 * 4] = -para.at(1, 3);
    m_modelview[2 + 0 * 4] = -para.at(2, 0); // R3
    m_modelview[2 + 1 * 4] = -para.at(2, 1);
    m_modelview[2 + 2 * 4] = -para.at(2, 2);
    m_modelview[2 + 3 * 4] = -para.at(2, 3);
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

#endif // WEBARKITGL_H