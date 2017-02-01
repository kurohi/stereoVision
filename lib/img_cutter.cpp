#include "img_cutter.hpp"

std::vector<cv::Mat> ImgCutter::cut_all_corners(const cv::Mat &src, cv::Rect center_rect){
    std::vector<cv::Mat> img_stack;
    img_stack.push_back(cut_top_only(src, center_rect));
    img_stack.push_back(cut_bottom_only(src, center_rect));
    img_stack.push_back(cut_left_only(src, center_rect));
    img_stack.push_back(cut_right_only(src, center_rect));
    img_stack.push_back(cut_center_only(src, center_rect));
    return img_stack;
}

cv::Mat ImgCutter::cut_top_only(const cv::Mat &src, cv::Rect center_rect){
    cv::Mat mask = cv::Mat::zeros(src.size(), src.type());
    std::vector<cv::Point> contour;
    contour.push_back(cv::Point(0,0));
    contour.push_back(cv::Point(mask.cols, 0));
    contour.push_back(cv::Point(center_rect.x+center_rect.width, center_rect.y));
    contour.push_back(cv::Point(center_rect.x, center_rect.y));
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    if(mask.channels()>1){
        cv::drawContours(mask, contours, -1, cv::Scalar(255,255,255), CV_FILLED);
    }else{
        cv::drawContours(mask, contours, -1, 255, CV_FILLED);
    }
    cv::Mat dst;
    cv::bitwise_and(src, mask, dst);
    return dst;
}

cv::Mat ImgCutter::cut_bottom_only(const cv::Mat &src, cv::Rect center_rect){
    cv::Mat mask = cv::Mat::zeros(src.size(), src.type());
    std::vector<cv::Point> contour;
    contour.push_back(cv::Point(0,mask.rows));
    contour.push_back(cv::Point(mask.cols, mask.rows));
    contour.push_back(cv::Point(center_rect.x+center_rect.width, center_rect.y+center_rect.height));
    contour.push_back(cv::Point(center_rect.x, center_rect.y+center_rect.height));
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    if(mask.channels()>1){
        cv::drawContours(mask, contours, -1, cv::Scalar(255,255,255), CV_FILLED);
    }else{
        cv::drawContours(mask, contours, -1, 255, CV_FILLED);
    }
    cv::Mat dst;
    cv::bitwise_and(src, mask, dst);
    return dst;
}

cv::Mat ImgCutter::cut_left_only(const cv::Mat &src, cv::Rect center_rect){
    cv::Mat mask = cv::Mat::zeros(src.size(), src.type());
    std::vector<cv::Point> contour;
    contour.push_back(cv::Point(0,0));
    contour.push_back(cv::Point(0, mask.rows));
    contour.push_back(cv::Point(center_rect.x, center_rect.y+center_rect.height));
    contour.push_back(cv::Point(center_rect.x, center_rect.y));
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    if(mask.channels()>1){
        cv::drawContours(mask, contours, -1, cv::Scalar(255,255,255), CV_FILLED);
    }else{
        cv::drawContours(mask, contours, -1, 255, CV_FILLED);
    }
    cv::Mat dst;
    cv::bitwise_and(src, mask, dst);
    return dst;
}

cv::Mat ImgCutter::cut_right_only(const cv::Mat &src, cv::Rect center_rect){
    cv::Mat mask = cv::Mat::zeros(src.size(), src.type());
    std::vector<cv::Point> contour;
    contour.push_back(cv::Point(mask.cols, 0));
    contour.push_back(cv::Point(mask.cols, mask.rows));
    contour.push_back(cv::Point(center_rect.x+center_rect.width, center_rect.y+center_rect.height));
    contour.push_back(cv::Point(center_rect.x+center_rect.width, center_rect.y));
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    if(mask.channels()>1){
        cv::drawContours(mask, contours, -1, cv::Scalar(255,255,255), CV_FILLED);
    }else{
        cv::drawContours(mask, contours, -1, 255, CV_FILLED);
    }
    cv::Mat dst;
    cv::bitwise_and(src, mask, dst);
    return dst;
}

cv::Mat ImgCutter::cut_center_only(const cv::Mat &src, cv::Rect center_rect){
    cv::Mat mask = cv::Mat::zeros(src.size(), src.type());
    std::vector<cv::Point> contour;
    contour.push_back(cv::Point(center_rect.x, center_rect.y));
    contour.push_back(cv::Point(center_rect.x+center_rect.width, center_rect.y));
    contour.push_back(cv::Point(center_rect.x+center_rect.width, center_rect.y+center_rect.height));
    contour.push_back(cv::Point(center_rect.x, center_rect.y+center_rect.height));
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    if(mask.channels()>1){
        cv::drawContours(mask, contours, -1, cv::Scalar(255,255,255), CV_FILLED);
    }else{
        cv::drawContours(mask, contours, -1, 255, CV_FILLED);
    }
    cv::Mat dst;
    cv::bitwise_and(src, mask, dst);
    return dst;
}
