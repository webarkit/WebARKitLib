#ifndef WEBARKITCAMERA_H
#define WEBARKITCAMERA_H

#include <opencv2/core.hpp>

namespace webarkit {
    class WebARKitCamera {
        public:
            WebARKitCamera();
            ~WebARKitCamera();

            bool setup();

            int xsize, ysize;
            cv::Mat cmat(3, 3, CV_64FC1);
            float kc[6];
        private:
            float focal_length;
            float diagonal_image_size;
            float diagonal_fov_degrees;
            float diagonal_fov_radians;

    };
}


#endif // WEBARKITCAMERA_H