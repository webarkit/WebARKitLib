#include <WebARKitManager.h>

namespace webarkit {

WebARKitManager::WebARKitManager(): state(NOTHING_INITIALISED), versionString(NULL) {}

WebARKitManager::~WebARKitManager() {
    if (versionString) {
        free(versionString);
        versionString = NULL;
    }
}

const char* WebARKitManager::getWebARKitVersion()
{
    if (!versionString) webarkit::webarkitGetVersion(&versionString);
	return versionString;
}

} // namespace webarkit