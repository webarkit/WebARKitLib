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

    WEBARKIT_LOGd("WebARKItManager::initialiseBase() done.\n");
    return true;
}

bool WebARKitManager::initTracker(uchar* refData, size_t refCols, size_t refRows) {
    if (!refData || refCols <= 0 || refRows <= 0) {
        WEBARKIT_LOGe("Error initialising tracker.\n");
        return false;
    }
    m_tracker->initTracker(refData, refCols, refRows);
    return true;
}

void WebARKitManager::processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace) {
  m_tracker->processFrameData(frameData, frameCols, frameRows, colorSpace);
}

std::vector<double> WebARKitManager::getOutputData() {
    return m_tracker->getOutputData();
};

bool WebARKitManager::isValid() {
  return m_tracker->isValid();
}

} // namespace webarkit