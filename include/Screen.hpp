/**
*  Screen module
*  Responsable for showing the images and drawing on it
*/

#ifndef SCREEN_HPP
#define SCREEN_HPP

#include <General.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

class Screen {
private:
	string screenName;
	Mat currentFrame;

public:
	Screen();
	Screen(string name);
	~Screen();
	void putImage(Mat img);
	string getScreenName();
	void drawRectangle(Mat& img, Rect2d rect);
};


#endif
