#include<TwinCamera.hpp>

TwinCamera::TwinCamera(int cam1, int cam2){
	camInd1 = cam1;
	camInd2 = cam2;
	calibMatrixLoaded = false;
}

TwinCamera::~TwinCamera(){
}

void TwinCamera::getDoubleImages(Mat &img1, Mat& img2){
	Mat Mean;
	Cam1.reopen(camInd1);
	if(!Cam1.isOpened()){
		return;
	}
	img1 = Cam1.getImage();
	/*
	Mean.create(img1.rows, img1.cols,CV_32FC3);
	int counter;
	for(counter = 0 ; counter< 100; counter++){
		img1 = Cam1.getImage();
		img1.convertTo(img1, CV_32FC3);
		Mean = Mean + img1;
		Mean = Mean / 2;
	}
	Mean.convertTo(Mean,CV_8U);
	img1 = Mean;
	*/
	Cam1.release();
	Cam2.reopen(camInd2);
	if(!Cam2.isOpened()){
		return;
	}
	img2 = Cam2.getImage();
	/*
	Mean = Mat::zeros(img2.rows, img2.cols,CV_32FC3);
	for(counter = 0 ; counter< 100; counter++){
		img2 = Cam2.getImage();
		img2.convertTo(img2, CV_32FC3);
		
		Mean = Mean + img2;
		Mean = Mean / 2;
	}
	Mean.convertTo(Mean,CV_8U);
	img2 = Mean;
	*/
	Cam2.release();
}

bool TwinCamera::loadCameraParameters(char *filename, Mat img1, Mat img2){
	Mat CM1, CM2;
	Mat D1, D2;
	Mat R, T, E, F;
	Mat R1, R2, P1, P2, Q;
	
	try{
		FileStorage fs(filename, FileStorage::READ);
		if(!fs.isOpened()){
			return false;
		}
		CM1 = Mat(3, 3, CV_64FC1);
		CM2 = Mat(3, 3, CV_64FC1);
		
		fs["CM1"] >> CM1;
		fs["CM2"] >> CM2;
		fs["D1"] >> D1;
		fs["D2"] >> D2;
		fs["R"] >> R;
		fs["T"] >> T;
		fs["E"] >> E;
		fs["F"] >> F;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs.release();
	
	}catch(char *err){
		std::cout<<"Fail to load the file: "<<filename<<std::endl;
		return false;
	}

	initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CV_32FC1, map2x, map2y);
	calibMatrixLoaded = true;
	return true;
}

void TwinCamera::rectifyForStereo(Mat &img1, Mat &img2){
	Mat imgU1, imgU2;

	remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	
	img1 = imgU1;
	img2 = imgU2;
}