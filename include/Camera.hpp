/**
*  Camera capture module
*  Responsable for creating a stream of frames
*/

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <General.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;

class Camera{
public:
	Camera();
	Camera(int camera_number);
	Camera(bool fake);
	~Camera();
	Mat getImage();
	bool isOpened();
	void reopen(int camera_number);
	void release();
	bool grabing();
	bool retrieving(Mat& img, int channels);

protected:
	VideoCapture cameraHandler;
	static int CAMERA_COUNTER;
};

#endif
