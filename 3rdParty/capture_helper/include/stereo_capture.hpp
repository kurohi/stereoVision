/**
 * @file stereo_capture.hpp
 * @brief Program that captures images from dual cameras
 */
#ifndef STEREO_CAPTURE_HPP
#define STEREO_CAPTURE_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include "camera_control.hpp"

/**
 * @class StereoCapture
 * @brief Class for capturing images from dual cameras
 * @note requite that v4l recognizes the cameras on /dev/videoX
 */
class StereoCapture {
    public:
        StereoCapture(int left_id=0, int right_id=1);
        ~StereoCapture();
        void captureDoubleImages(cv::Mat &left_img, cv::Mat &right_img, bool with_rotation=true);
        cv::Mat captureLeftImage();
        cv::Mat captureRightImage();

        void matchExposureToLeftCamera();
        void backToAutoExposure();

        void identifyLeftRight();
        void identifyLeftRight(cv::Mat img1, cv::Mat img2, bool with_rotation=true);
        static bool identifyLeftRightIndependent(cv::Mat img1, cv::Mat img2, bool with_rotation=true);

        void saveImages(std::string folder_root);
        void saveImages(std::string folder_root, cv::Mat left_img, cv::Mat right_img);

        bool setLeftCameraId(int new_id);
        bool setRightCameraId(int new_id);
        int getLeftCameraId();
        int getRightCameraId();
    private:
        cv::VideoCapture camera_left, camera_right;
        CameraControl left_control, right_control;
        int left_camera_id, right_camera_id;

        static void rotate180(cv::Mat &src);
};

#endif
