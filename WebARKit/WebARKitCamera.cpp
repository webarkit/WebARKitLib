#include <cmath>
#include <algorithm>
#include <WebARKitCamera.h>

namespace webarkit {
    WebARKitCamera::WebARKitCamera() : xsize(-1), ysize(-1), diagonal_fov_degrees(70.0) {
         cmat = cv::Mat.zeros(3, 3, CV_64FC1);
    }

    WebARKitCamera::~WebARKitCamera() {
    }

    bool WebARKitCamera::setup() {
        //diagonal_image_size = (w ** 2.0 + h ** 2.0) ** 0.5
        diagonal_image_size = std::pow(std::pow(xsize, 2.0) + std::pow(ysize, 2.0), 0.5);
        /*diagonal_fov_radians = \
            diagonal_fov_degrees * math.pi / 180.0*/
        diagonal_fov_radians = diagonal_fov_degrees * M_PI / 180.0;
        focal_length = 0.5 * diagonal_image_size / std::tan(0.5 * diagonal_fov_radians);
    };
}