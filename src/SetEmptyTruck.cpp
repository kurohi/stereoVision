#include <TwinCamera.hpp>
#include <StereoDepth.hpp>

int main(int argc, char **argv){
	if(argc<2){
		std::cout<<"Need to specify the calibration file"<<std::endl;
		return 1;
	}
	Mat img1,img2, disparity;
	TwinCamera twin(0,1);
	twin.getDoubleImages(img1,img2);
	twin.loadCameraParameters(argv[1], img1, img2);
	StereoDepth stereoDepth;
	twin.rectifyForStereo(img1, img2);
	stereoDepth.setImage1(img1);
	stereoDepth.setImage2(img2);
	if(stereoDepth.doDepth()){
		disparity = stereoDepth.getDisparity();
		cv::equalizeHist(disparity,disparity);
		imwrite("img1.jpg",img1);
		imwrite("img2.jpg",img2);
		
		imwrite("emptyTruck.jpg", disparity);
	}else{
		std::cout<<"Error when computing the stereo image."<<std::endl;
	}
}
