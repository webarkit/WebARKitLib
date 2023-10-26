#ifndef WEBARKITCAMERA_H
#define WEBARKITCAMERA_H

#include <array>

namespace webarkit {
class WebARKitCamera {
  public:
    WebARKitCamera();
    ~WebARKitCamera();

    bool setupCamera(int width, int height);
    void printSettings();

    int xsize, ysize;
    std::array<double, 9> cmat;
    std::array<double, 6> kc;

  private:
    double focal_length;
    double diagonal_image_size;
    double diagonal_fov_degrees;
    double diagonal_fov_radians;
};
} // namespace webarkit

#endif // WEBARKITCAMERA_H