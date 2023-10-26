#include <WebARKitCamera.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>

namespace webarkit {
WebARKitCamera::WebARKitCamera() : xsize(-1), ysize(-1), diagonal_fov_degrees(70.0) { cmat.fill(0.0); }

WebARKitCamera::~WebARKitCamera() {}

bool WebARKitCamera::setupCamera(int width, int height) {
    if (width <= 0 || height <= 0) {
        return false;
    }
    xsize = width;
    ysize = height;
    // simple routine to calculate focal length from diagonal field of view, and convert to camera matrix.
    diagonal_image_size = std::pow(std::pow(xsize, 2.0) + std::pow(ysize, 2.0), 0.5);
    diagonal_fov_radians = diagonal_fov_degrees * m_pi / 180.0;
    focal_length = 0.5 * diagonal_image_size / std::tan(0.5 * diagonal_fov_radians);

    cmat.at(0) = focal_length;
    cmat.at(2) = 0.5 * xsize;
    cmat.at(4) = focal_length;
    cmat.at(5) = 0.5 * ysize;
    cmat.at(8) = 1.0;
    kc.fill(0.0);
    return true;
};

void WebARKitCamera::printSettings() {
    printf("WebARKit: Camera Size %d , %d\n", xsize, ysize);
    printf("WebARKit: camera matrix = [%.2f  %.2f %.2f]\n", cmat[0], cmat[1], cmat[2]);
    printf("                          [%.2f  %.2f %.2f]\n", cmat[3], cmat[4], cmat[5]);
    printf("                          [%.2f  %.2f %.2f]\n", cmat[6], cmat[7], cmat[8]);
    printf("WebARKit: kc = [%.4f %.4f %.4f %.4f %.4f %.4f]\n", kc[0], kc[1], kc[2], kc[3], kc[4], kc[5]);
};
} // namespace webarkit