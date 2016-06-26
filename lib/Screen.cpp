/**
*   Implementation of Screen.hpp
*/

#include <Screen.hpp>

Screen::Screen(){
	screenName = string("Window");
	namedWindow(screenName.c_str(), WINDOW_NORMAL);
}

Screen::Screen(string name){
	screenName = name;
	namedWindow(screenName.c_str(), WINDOW_NORMAL);
}

Screen::~Screen(){
}

void Screen::putImage(Mat img){
	currentFrame = img;
	if(img.empty()){
		throw NULL_IMAGE;
	}
	imshow(screenName, currentFrame);
}

string Screen::getScreenName(){
	return screenName;
}

void Screen::drawRectangle(Mat& img, Rect2d rect){
	rectangle(img, rect, Scalar(255, 0, 0), 2, 1);
}
