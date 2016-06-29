/**
*  Screen module
*  Responsable for showing the images and drawing on it
*/

#ifndef SCREEN_HPP
#define SCREEN_HPP

#include <General.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>


class Screen {
private:
	std::string screenName;
	cv::Mat currentFrame;

public:
	Screen();
	Screen(std::string name);
	~Screen();
	void putImage(cv::Mat img);
	std::string getScreenName();
	void drawRectangle(cv::Mat& img, cv::Rect2d rect);
};


#endif
