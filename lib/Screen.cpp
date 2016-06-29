/**
*   Implementation of Screen.hpp
*/

#include <Screen.hpp>

Screen::Screen(){
	screenName = std::string("Window");
	cv::namedWindow(screenName.c_str(), cv::WINDOW_NORMAL);
}

Screen::Screen(std::string name){
	screenName = name;
	cv::namedWindow(screenName.c_str(), cv::WINDOW_NORMAL);
}

Screen::~Screen(){
}

void Screen::putImage(cv::Mat img){
	currentFrame = img;
	if(img.empty()){
		throw NULL_IMAGE;
	}
	cv::imshow(screenName, currentFrame);
}

std::string Screen::getScreenName(){
	return screenName;
}

void Screen::drawRectangle(cv::Mat& img, cv::Rect2d rect){
	cv::rectangle(img, rect, cv::Scalar(255, 0, 0), 2, 1);
}
