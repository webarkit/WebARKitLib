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

    std::array<double, 9> getCameraData() const;

    std::array<double, 6> getDistortionCoefficients() const;

  private:
    int xsize, ysize;
    std::array<double, 9> cmat;
    std::array<double, 6> kc;
    double focal_length;
    double diagonal_image_size;
    double diagonal_fov_degrees;
    double diagonal_fov_radians;
};
} // namespace webarkit

#endif // WEBARKITCAMERA_H