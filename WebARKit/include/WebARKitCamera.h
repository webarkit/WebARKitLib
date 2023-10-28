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

    double getFocalLength() const { return focal_length; }

  private:
    int xsize, ysize;
    std::array<double, 9> cmat;
    std::array<double, 6> kc;
    double focal_length;
    double diagonal_fov_degrees;

    void setFocalLength(int width, int height);
};
} // namespace webarkit

#endif // WEBARKITCAMERA_H