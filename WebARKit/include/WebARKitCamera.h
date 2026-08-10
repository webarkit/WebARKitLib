#ifndef WEBARKITCAMERA_H
#define WEBARKITCAMERA_H

#include <array>

namespace webarkit {
class WebARKitCamera {
  public:
    WebARKitCamera();
    ~WebARKitCamera();

    bool setupCamera(int width, int height);

    // Load real camera calibration from an ArtoolkitX camera_para.dat buffer
    // (ARParam), rescaled to width x height. Fills the intrinsic matrix and
    // distortion from the file instead of the synthetic FOV-based defaults.
    bool loadCameraParamFromBuffer(const void* buffer, int size, int width, int height);

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