#ifndef WEBARKIT_TRACKER_H
#define WEBARKIT_TRACKER_H

#include "HomographyInfo.h"
#include "TrackableInfo.h"
#include "WebARKitConfig.h"
#include "WebARKitEnums.h"
#include "WebARKitFeatureDetector.h"


namespace webarkit {

class WebARKitTracker {
  public:
    WebARKitTracker();

    ~WebARKitTracker();

    WebARKitTracker(WebARKitTracker&&);

    WebARKitTracker& operator=(WebARKitTracker&&);

    void initialize(webarkit::TRACKER_TYPE trackerType);

    void initialize_w(webarkit::TRACKER_TYPE trackerType, size_t xsize, size_t ysize);

    void initTracker(uchar* refData, size_t refCols, size_t refRows);

    void loadARParam(std::string paramName, webarkit::TRACKER_TYPE trackerType);

    void AddMarker(uchar* buff, std::string fileName, int width, int height, int uid, float scale);

    bool GetTrackablePose(int trackableId, float transMat[3][4]);

    bool IsTrackableVisible(int trackableId);

    void processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace);

    void ProcessFrameData_w(uchar* frameData);

    std::vector<double> getOutputData();

    bool isValid();

  private:
    class WebARKitTrackerImpl;

    std::shared_ptr<WebARKitTrackerImpl> _trackerImpl;
};

} // namespace webarkit

#endif