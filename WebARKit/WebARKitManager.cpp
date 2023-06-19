#include <WebARKitManager.h>

namespace webarkit {

WebARKitManager::WebARKitManager() : state(NOTHING_INITIALISED), versionString(NULL) {}

WebARKitManager::~WebARKitManager() {
    if (versionString) {
        free(versionString);
        versionString = NULL;
    }
}

const char* WebARKitManager::getWebARKitVersion() {
    if (!versionString)
        webarkit::webarkitGetVersion(&versionString);
    return versionString;
}

bool WebARKitManager::initialiseBase(webarkit::TRACKER_TYPE trackerType) {
    WEBARKIT_LOGd("WebARKItManager::initialiseBase(...)\n");
    if (state != NOTHING_INITIALISED) {
        WEBARKIT_LOGe("Initialise called while already initialised. Will finish first.\n");
        if (!shutdown()) {
            return false;
        }
    }

    char* versionString = NULL;
    webarkitGetVersion(&versionString);
    WEBARKIT_LOGi("Webarkit C++ lib v%s initalised.\n", versionString);
    free(versionString);

    m_trackerType = trackerType;

    m_tracker = std::shared_ptr<webarkit::WebARKitTracker>(new webarkit::WebARKitTracker);
    m_tracker->initialize(m_trackerType);

    state = BASE_INITIALISED;

    WEBARKIT_LOGd("WebARKitManager::initialiseBase() done.\n");
    return true;
}

bool WebARKitManager::initTracker(uchar* refData, size_t refCols, size_t refRows) {
    WEBARKIT_LOGd("WebARKitManager::initTracker(...)\n");
    if (!refData || refCols <= 0 || refRows <= 0) {
        WEBARKIT_LOGe("Error initialising tracker.\n");
        return false;
    }
    m_tracker->initTracker(refData, refCols, refRows);
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

void WebARKitManager::processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) {
    WEBARKIT_LOGd("WebARKitManager::processFrameData(...)\n");
    if (state < WAITING_FOR_VIDEO) {
        WEBARKIT_LOGe("processFrameData called without init the tracker. Call first initTracker.\n");
    }
    if (!frameData || frameCols <= 0 || frameRows <= 0) {
        WEBARKIT_LOGe("Error initialising processFrameData.\n");
        //return false;
    }
  m_tracker->processFrameData(frameData, frameCols, frameRows, colorSpace);
  state = DETECTION_RUNNING;
  WEBARKIT_LOGd("WebARKitManager::processFrameData() done\n");
}

std::vector<double> WebARKitManager::getOutputData() {
    return m_tracker->getOutputData();
};

bool WebARKitManager::isValid() {
  return m_tracker->isValid();
}

} // namespace webarkit