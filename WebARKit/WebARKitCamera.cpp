#include <WebARKitCamera.h>
#include <WebARKitLog.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitConfig.h>
// Use the umbrella <AR/ar.h> (not <AR/param.h> directly): ar.h includes param.h
// in the correct order so ARParam/ARParamLT are defined before ar.h uses them.
// It transitively declares arParamLoadFromBuffer / arParamChangeSize.
#include <AR/ar.h>
#include <cstddef>

namespace webarkit {
WebARKitCamera::WebARKitCamera() : xsize(-1), ysize(-1), diagonal_fov_degrees(70.0) { cmat.fill(0.0); }

WebARKitCamera::~WebARKitCamera() {}

bool WebARKitCamera::loadCameraParamFromBuffer(const void* buffer, int size, int width, int height) {
    if (!buffer || size <= 0 || width <= 0 || height <= 0) {
        return false;
    }
    ARParam param, scaled;
    if (arParamLoadFromBuffer(buffer, (size_t)size, &param) < 0) {
        WEBARKIT_LOGe("loadCameraParamFromBuffer: not a valid camera_para.dat buffer.\n");
        return false;
    }
    if (arParamChangeSize(&param, width, height, &scaled) < 0) {
        WEBARKIT_LOGe("loadCameraParamFromBuffer: arParamChangeSize failed.\n");
        return false;
    }

    xsize = width;
    ysize = height;

    // Intrinsic matrix K = first 3 columns of the calibrated 3x4 projection
    // matrix (the 4th column is the projection offset, ~0 for a pinhole).
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cmat.at(i * 3 + j) = (double)scaled.mat[i][j];
        }
    }
    focal_length = cmat.at(0);

    // Distortion -> OpenCV distCoeffs (k1,k2,p1,p2[,k3,...]).
    // This vendored lib is v4-era (AR_DIST_FACTOR_NUM_MAX = 9). For v4, the
    // OpenCV-compatible coefficients are dist_factor[0..3] with k3 = 0 (matching
    // ArtoolkitX OCVT). A future v5 (.dat with OpenCV rational model) would copy
    // dist_factor directly.
    // TODO: verify the v4 ArtoolkitX -> OpenCV distortion mapping against a real
    // camera_para.dat before relying on distortion (intrinsics are the main win).
    kc.fill(0.0);
    if (scaled.dist_function_version >= 5) {
        for (int i = 0; i < 6 && i < AR_DIST_FACTOR_NUM_MAX; ++i) {
            kc.at(i) = (double)scaled.dist_factor[i];
        }
    } else {
        kc.at(0) = (double)scaled.dist_factor[0]; // k1
        kc.at(1) = (double)scaled.dist_factor[1]; // k2
        kc.at(2) = (double)scaled.dist_factor[2]; // p1
        kc.at(3) = (double)scaled.dist_factor[3]; // p2
        kc.at(4) = 0.0;                           // k3
    }
    return true;
}

bool WebARKitCamera::setupCamera(int width, int height) {
    if (width <= 0 || height <= 0) {
        return false;
    }
    xsize = width;
    ysize = height;

    setFocalLength(xsize, ysize);

    cmat.at(0) = focal_length;
    cmat.at(2) = 0.5 * xsize;
    cmat.at(4) = focal_length;
    cmat.at(5) = 0.5 * ysize;
    cmat.at(8) = 1.0;
    kc.fill(0.0);
    return true;
};

void WebARKitCamera::printSettings() {
    WEBARKIT_LOGi("WebARKit: Camera Size %d , %d\n", xsize, ysize);
    WEBARKIT_LOGi("WebARKit: camera matrix = [%.2f  %.2f %.2f]\n", cmat[0], cmat[1], cmat[2]);
    WEBARKIT_LOGi("                          [%.2f  %.2f %.2f]\n", cmat[3], cmat[4], cmat[5]);
    WEBARKIT_LOGi("                          [%.2f  %.2f %.2f]\n", cmat[6], cmat[7], cmat[8]);
    WEBARKIT_LOGi("WebARKit: kc = [%.4f %.4f %.4f %.4f %.4f %.4f]\n", kc[0], kc[1], kc[2], kc[3], kc[4], kc[5]);
};

std::array<double, 9> WebARKitCamera::getCameraData() const {
    return cmat;
}

std::array<double, 6> WebARKitCamera::getDistortionCoefficients() const {
   return kc;
}

void WebARKitCamera::setFocalLength(int width, int height) {
    double diagonal_image_size;
    double diagonal_fov_radians;
     // simple routine to calculate focal length from diagonal field of view, and convert to camera matrix.
    diagonal_image_size = std::pow(std::pow(width, 2.0) + std::pow(height, 2.0), 0.5);
    diagonal_fov_radians = diagonal_fov_degrees * m_pi / 180.0;
    focal_length = 0.5 * diagonal_image_size / std::tan(0.5 * diagonal_fov_radians);
}
} // namespace webarkit