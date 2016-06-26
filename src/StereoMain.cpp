#include <Screen.hpp>
#include <TwinCamera.hpp>
#include <StereoDepth.hpp>

int main(int argc, char **argv){
	if(argc<2){
		std::cout<<"Need to specify the calibration file"<<std::endl;
		return 1;
	}
	Mat img1,img2;
	TwinCamera twin(0,1);
	twin.getDoubleImages(img1,img2);
	twin.loadCameraParameters(argv[1], img1, img2);
	Screen image1("Image1");
	Screen image2("Image2");
	Screen disp("Disparity");
	StereoDepth stereoDepth;
	char c = 0;
	while(c!=27){
		switch(c){
			case '1':
				image1.putImage(img1);
				image2.putImage(img2);
				break;
			case '3':
				twin.getDoubleImages(img1,img2);
				twin.rectifyForStereo(img1, img2);
				stereoDepth.setImage1(img1);
				stereoDepth.setImage2(img2);
				if(stereoDepth.doDepth()){
					disp.putImage(stereoDepth.getDisparity());
				}break;
		}
		
		
		c = waitKey(10);
	}
}
