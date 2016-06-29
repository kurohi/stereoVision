/**
*   Implementation of Camera.hpp
*/

#include<Camera.hpp>

int Camera::CAMERA_COUNTER = 0;

Camera::Camera(){
	cameraHandler.open(CAMERA_COUNTER++);
	if(!cameraHandler.isOpened()){
		return;
	}
	//cameraHandler.set(CV_CAP_PROP_FRAME_WIDTH, DEFAULT_WIDTH);
	//cameraHandler.set(CV_CAP_PROP_FRAME_HEIGHT, DEFAULT_HEIGHT);
	//cameraHandler.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G'));
	//cameraHandler.set(CV_CAP_PROP_CONVERT_RGB , false);
	//cameraHandler.set(CV_CAP_PROP_FPS , 5);
}

Camera::Camera(bool fake){
}

Camera::Camera(int camera_number){
	cameraHandler.open(camera_number);
	if(!cameraHandler.isOpened()){
		return;
	}
	cameraHandler.set(CV_CAP_PROP_FRAME_WIDTH, DEFAULT_WIDTH);
	cameraHandler.set(CV_CAP_PROP_FRAME_HEIGHT, DEFAULT_HEIGHT);
	cameraHandler.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G'));
	cameraHandler.set(CV_CAP_PROP_CONVERT_RGB , false);
	cameraHandler.set(CV_CAP_PROP_FPS , 5);
}
Camera::~Camera(){
	cameraHandler.release();
}

cv::Mat Camera::getImage(){
	cv::Mat img;
	if(cameraHandler.isOpened()){
		cameraHandler >> img;
	}
	return img;
}

bool Camera::isOpened(){
	return cameraHandler.isOpened();
}


void Camera::reopen(int camera_number=0){
	cameraHandler.open(camera_number);
}

void Camera::release(){
	cameraHandler.release();
}


bool Camera::grabing(){
	return cameraHandler.grab();
}

bool Camera::retrieving(cv::Mat& img, int channels=0){
	return cameraHandler.retrieve(img,channels);
}


