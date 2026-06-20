#include <WebARKitPattern.h>
#include <opencv2/calib3d.hpp>

WebARKitPatternTrackingInfo::WebARKitPatternTrackingInfo() {
    pose3d = cv::Mat::zeros(4, 4, CV_64FC1);
    glViewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);
    m_scale = 1.0f;
}

void WebARKitPatternTrackingInfo::cameraPoseFromPoints(cv::Mat& pose, const std::vector<cv::Point3f>& objPts,
                                                       const std::vector<cv::Point2f>& imgPts, const cv::Matx33f& caMatrix,
                                              const cv::Mat& distCoeffs) {
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1); // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1); // output translation vector

    cv::solvePnPRansac(objPts, imgPts, caMatrix, distCoeffs, rvec, tvec);

    // Assemble pose matrix from rotation and translation vectors.
    cv::Mat rMat;
    Rodrigues(rvec, rMat);
    cv::hconcat(rMat, tvec, pose);

    // WebARKitLib#55: also populate pose3d -- the raw 4x4 OpenCV-convention camera
    // pose returned by getPoseMatrixCV(). The live path previously left pose3d at its
    // zero-initialised value, so getPoseMatrixCV() returned an all-zero matrix. Build
    // it here from the same rMat/tvec the GL path uses, so the CV and GL getters both
    // reflect the current frame. No handedness/scale correction is applied (that is
    // getPoseMatrixGL's trans); pose3d stays in raw OpenCV convention.
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            pose3d.at<double>(row, col) = rMat.at<double>(row, col);
        }
        pose3d.at<double>(row, 3) = tvec.at<double>(row, 0);
    }
    pose3d.at<double>(3, 3) = 1.0;
};

void WebARKitPatternTrackingInfo::getTrackablePose(cv::Mat& pose) {
    //float transMat [3][4];
    cv::Mat poseOut;
    pose.convertTo(poseOut, CV_32FC1);
    //std::cout << "poseOut: " << poseOut << std::endl;
    memcpy(transMat, poseOut.ptr<float>(0), 3*4*sizeof(float));
}

void WebARKitPatternTrackingInfo::updateTrackable() {
    if (transMat) {
        // CV->GL conversion, marker/object side: negate the Y and Z rotation
        // COLUMNS (matching ArtoolkitX ARTrackable2d::updateWithTwoDResults).
        // Combined with the Y,Z ROW negation applied downstream by arglCameraViewRHf
        // (when building matrixGL_RH / getGLViewMatrix), this yields the consistent
        // similarity D*R*D (D = diag(1,-1,-1)) -- a proper right-handed marker frame
        // with X=right, Y=up, Z=toward the viewer, so AR content placed at +Z pops
        // up out of the marker as expected. Without this column negation the frame
        // is the raw OpenCV handedness (Z into the marker / away from the camera).
        //
        // The translation (column 3) is NOT negated here -- it keeps its sign and
        // the marker-size scale. arglCameraViewRHf negates the Y,Z translation rows
        // downstream, so depth stays in front of the camera (no "behind camera"
        // regression); only the rotation handedness is corrected. See WebARKitLib#42.
        for (int j = 0; j < 3; j++) {
            trans[j][0] =  transMat[j][0];
            trans[j][1] = -transMat[j][1];
            trans[j][2] = -transMat[j][2];
            trans[j][3] = (transMat[j][3] * m_scale * 0.001f * 1.64f);
        }
    }
}

void WebARKitPatternTrackingInfo::computeGLviewMatrix(cv::Mat &pose) { cv::transpose(pose, glViewMatrix); }