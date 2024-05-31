/*
 *  WebARKitManager.h
 *  WebARKit
 *
 *  This file is part of WebARKit.
 *
 *  WebARKit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  WebARKit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with WebARKit.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2023 WebARKit.
 *
 *  Author(s): Walter Perdan
 *
 *
 */

#ifndef WEBARKIT_MANAGER_H
#define WEBARKIT_MANAGER_H

#include <WebARKitLog.h>
#include <WebARKitGL.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker.h>
#include <WebARKitTrackers/WebARKitOpticalTracking/WebARKitUtils.h>

namespace webarkit{

class WebARKitManager {
  private:
    typedef enum {
        NOTHING_INITIALISED, ///< No initialisation yet and no resources allocated.
        BASE_INITIALISED,    ///< Trackable management initialised, trackables can be added.
        WAITING_FOR_VIDEO,   ///< Waiting for video source to become ready.
        DETECTION_RUNNING    ///< Video running, additional initialisation occurred, tracking running.
    } WebARKitState;

    WebARKitState state; ///< Current state of operation, progress through initialisation
    std::string versionString;
    std::shared_ptr<WebARKitTracker> m_tracker;
    webarkit::TRACKER_TYPE m_trackerType;

  public:
    /**
     * Constructor.
     */
    WebARKitManager();

    /**
     * Destructor.
     */
    ~WebARKitManager();

    /**
     * Returns a string containing the WebARKit version, such as "1.0.0".
     * @return		The WebARKit version as a std::string
     */
    std::string getWebARKitVersion();

    /**
     * Start trackable management so trackables can be added and removed.
     * @return       true if initialisation was OK, false if an error occured.
     */
    bool initialiseBase(webarkit::TRACKER_TYPE trackerType, int frameWidth, int frameHeight);

    /**
     * Return the current tracker object.
     * @return  the WebARKitTracker object.
     */
    std::shared_ptr<webarkit::WebARKitTracker> getTracker() { return m_tracker; };

    bool initTracker(cv::Mat refData, size_t refCols, size_t refRows, ColorSpace colorSpace);

    bool initTracker(uchar* refData, size_t refCols, size_t refRows, ColorSpace colorSpace);

    bool update();

    void setLogLevel(int logLevel);

    bool shutdown();

    void processFrameData(uchar* frameData, size_t frameCols, size_t frameRows, ColorSpace colorSpace, bool enableBlur);

    std::vector<double> getOutputData();

    cv::Mat getPoseMatrix();

    float* getPoseMatrix2();

    cv::Mat getGLViewMatrix();

    std::array<double, 16> getTransformationMatrix();

    std::array<double, 16> getCameraProjectionMatrix();

    bool isValid();
};

} // namespace webarkit

#endif // WEBARKIT_MANAGER_H