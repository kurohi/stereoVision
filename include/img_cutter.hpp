/**
 * @brief This static class cuts a image into 4 trapeziuns and 1 square at the center. Used for finding walls in a corridor with RANSAC
 */
#ifndef IMG_CUTTER_HPP
#define IMG_CUTTER_HPP

#include "general.hpp"
#include <opencv2/imgproc.hpp>

class ImgCutter{
    public:
        /**
         * @brief Wrapper of all the cutters returning a list of all the cuts
         * @note The order is top->bottom->left->right->center
         * @param[in] cv::Mat source image to be cut
         * @param[in] cv::Rect the center square of the corridor
         * @returns std::vector with the list of images
         */
        static std::vector<cv::Mat> cut_all_corners(const cv::Mat &src, cv::Rect center_rect);
        /**
         * @brief cuts the top of the corridor 
         * @param[in] cv::Mat source image to be cut
         * @param[in] cv::Rect the center square of the corridor
         * @returns cv::Mat cut image
         */
        static cv::Mat cut_top_only(const cv::Mat &src, cv::Rect center_rect);
        /**
         * @brief cuts the bottom of the corridor 
         * @param[in] cv::Mat source image to be cut
         * @param[in] cv::Rect the center square of the corridor
         * @returns cv::Mat cut image
         */
        static cv::Mat cut_bottom_only(const cv::Mat &src, cv::Rect center_rect);
        /**
         * @brief cuts the left of the corridor 
         * @param[in] cv::Mat source image to be cut
         * @param[in] cv::Rect the center square of the corridor
         * @returns cv::Mat cut image
         */
        static cv::Mat cut_left_only(const cv::Mat &src, cv::Rect center_rect);
        /**
         * @brief cuts the right of the corridor 
         * @param[in] cv::Mat source image to be cut
         * @param[in] cv::Rect the center square of the corridor
         * @returns cv::Mat cut image
         */
        static cv::Mat cut_right_only(const cv::Mat &src, cv::Rect center_rect);
        /**
         * @brief cuts the center of the corridor 
         * @param[in] cv::Mat source image to be cut
         * @param[in] cv::Rect the center square of the corridor
         * @returns cv::Mat cut image
         */
        static cv::Mat cut_center_only(const cv::Mat &src, cv::Rect center_rect);
    private:
        ImgCutter(){}
        ~ImgCutter(){}
};

#endif

