#ifndef TWINCAMERA_HPP
#define TWINCAMERA_HPP

#include<Camera.hpp>

using namespace cv;

class TwinCamera{
public:
	TwinCamera(int cam_1, int cam_2);
	~TwinCamera();
	
	void getDoubleImages(Mat &img1, Mat &img2);
	void rectifyForStereo(Mat &img1, Mat &img2);
	
	bool loadCameraParameters(char *filename, Mat img1, Mat img2);
protected:
	int camInd1, camInd2;
	Camera Cam1, Cam2;
	//matrixes for calibration
	bool calibMatrixLoaded;
	Mat map1x, map1y, map2x, map2y;
};


#endif
