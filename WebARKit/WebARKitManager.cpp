#include <WebARKitManager.h>

namespace webarkit {

WebARKitManager::WebARKitManager() : state(NOTHING_INITIALISED), versionString("?"), m_trackerType(webarkit::AKAZE_TRACKER) {}

WebARKitManager::~WebARKitManager() {
    if (!versionString.empty()) {
        versionString.clear();
    }
}

std::string WebARKitManager::getWebARKitVersion() {
    if (versionString.empty()) {
        versionString = webarkitGetVersion();
        WEBARKIT_LOGi("Webarkit C++ lib version: %s.\n", versionString.c_str());
    }
    return versionString;
}

bool WebARKitManager::initialiseBase(webarkit::TRACKER_TYPE trackerType, int frameWidth, int frameHeight) {
    WEBARKIT_LOGd("WebARKItManager::initialiseBase(...)\n");
    if (state != NOTHING_INITIALISED) {
        WEBARKIT_LOGe("Initialise called while already initialised. Will finish first.\n");
        if (!shutdown()) {
            return false;
        }
    }

    versionString = webarkitGetVersion();
    WEBARKIT_LOGi("Webarkit C++ lib v%s initalised.\n", versionString.c_str());

    m_trackerType = trackerType;

    m_tracker = std::make_shared<webarkit::WebARKitTracker>();
    m_tracker->initialize(m_trackerType, frameWidth, frameHeight);

    state = BASE_INITIALISED;

    WEBARKIT_LOGd("WebARKitManager::initialiseBase() done.\n");
    return true;
}

bool WebARKitManager::initTracker(cv::Mat refData, size_t refCols, size_t refRows, ColorSpace colorSpace) {
    WEBARKIT_LOGd("WebARKitManager::initTracker(...)\n");
    if (refData.empty() || refCols <= 0 || refRows <= 0) {
        WEBARKIT_LOGe("Error initialising tracker.\n");
        return false;
    }
    m_tracker->initTracker(refData, refCols, refRows, colorSpace);
    state = WAITING_FOR_VIDEO;
    WEBARKIT_LOGd("WebARKitManager::initTracker() done.\n");
    return true;
}

bool WebARKitManager::initTracker(uchar* refData, size_t refCols, size_t refRows, ColorSpace colorSpace) {
    WEBARKIT_LOGd("WebARKitManager::initTracker(...)\n");
    if (!refData || refCols <= 0 || refRows <= 0) {
        WEBARKIT_LOGe("Error initialising tracker.\n");
        return false;
    }
    m_tracker->initTracker(refData, refCols, refRows, colorSpace);
    state = WAITING_FOR_VIDEO;
    WEBARKIT_LOGd("WebARKitManager::initTracker() done.\n");
    return true;
}

void WebARKitManager::setLogLevel(int logLevel) {
    WEBARKIT_LOGd("WebARKitManager::setLogLevel(...)\n");
    if (logLevel >= 0) {
        webarkitLogLevel = logLevel;
    }
    WEBARKIT_LOGd("WebARKitManager::setLogLevel() done.\n");
}

bool WebARKitManager::shutdown() {
    m_tracker.reset();
    WEBARKIT_LOGd("WebARKitManager::shutdown(): Shutting down WebARKitManager.");
    return true;
};

void WebARKitManager::processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace, BLUR_TYPE blurType, bool enableBlur) {
    WEBARKIT_LOGd("WebARKitManager::processFrameData(...)\n");
    if (state < WAITING_FOR_VIDEO) {
        WEBARKIT_LOGe("processFrameData called without init the tracker. Call first initTracker.\n");
    }
    if (!frameData || frameCols <= 0 || frameRows <= 0) {
        WEBARKIT_LOGe("Error initialising processFrameData.\n");
        //return false;
    }
  m_tracker->processFrameData(frameData, frameCols, frameRows, colorSpace, blurType, enableBlur);
  state = DETECTION_RUNNING;
  WEBARKIT_LOGd("WebARKitManager::processFrameData() done\n");
}

std::vector<double> WebARKitManager::getOutputData() {
    return m_tracker->getOutputData();
};

cv::Mat WebARKitManager::getPoseMatrix() {
    return m_tracker->getPoseMatrix();
}

float* WebARKitManager::getPoseMatrix2() {
    return m_tracker->getPoseMatrix2();
}

cv::Mat WebARKitManager::getGLViewMatrix() {
    return m_tracker->getGLViewMatrix();
}

std::array<double, 16> WebARKitManager::getTransformationMatrix() {
    std::array<double, 16> transformationMatrix;
    webarkit::arglCameraViewRHf(m_tracker->getPoseMatrix(), transformationMatrix, 1.0f);
    return transformationMatrix;
}

std::array<double, 16> WebARKitManager::getCameraProjectionMatrix() {
    return m_tracker->getCameraProjectionMatrix();
}

bool WebARKitManager::isValid() {
  return m_tracker->isValid();
}

} // namespace webarkit