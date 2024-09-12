#ifndef WEBARKIT_ENUMS_H
#define WEBARKIT_ENUMS_H

namespace webarkit {

enum TRACKER_TYPE {
    AKAZE_TRACKER = 0,
    ORB_TRACKER = 1,
    FREAK_TRACKER = 2,
    TEBLID_TRACKER = 3
};

enum ColorSpace {
    RGB = 0,
    RGBA = 1,
    GRAY = 2
};

enum BLUR_TYPE {
    MEDIAN_BLUR = 0,
    BOX_BLUR = 1,
    NONE_BLUR = 2
};

} // namespace webarkit

#endif