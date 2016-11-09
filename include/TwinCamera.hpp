#ifndef TWINCAMERA_HPP
#define TWINCAMERA_HPP

#include<Camera.hpp>
#include<camera_control.hpp>


class TwinCamera{
public:
	TwinCamera(int cam_1, int cam_2);
	~TwinCamera();
	
	void getDoubleImages(cv::Mat &img1, cv::Mat &img2);
	void rectifyForStereo(cv::Mat &img1, cv::Mat &img2);
	
	bool loadCameraParameters(char *filename, cv::Mat img1, cv::Mat img2);
	cv::Mat getQMatrix();
protected:
	int camInd1, camInd2;
	Camera Cam1, Cam2;
    CameraControl cam1_control, cam2_control;
	//matrixes for calibration
	bool calibMatrixLoaded;
	cv::Mat map1x, map1y, map2x, map2y, Q;
};


#endif
