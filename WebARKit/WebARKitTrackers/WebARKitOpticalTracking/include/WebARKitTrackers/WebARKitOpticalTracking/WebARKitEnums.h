#ifndef WEBARKIT_ENUMS_H
#define WEBARKIT_ENUMS_H

namespace webarkit {

    enum TRACKER_TYPE {
        AKAZE_TRACKER = 0,
        ORB_TRACKER = 1 
    };

    enum ColorSpace {
        RGB = 0,
        RGBA = 1,
        GRAY = 2
    };

} // namespace webarkit

#endif