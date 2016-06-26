#include <Camera.hpp>
#include <Screen.hpp>
#include <StereoDepth.hpp>

int main(int argc, char **argv){
	Camera cam(0);
	//Screen screen("video");
	Screen image1("Image1");
	Screen image2("Image2");
	Screen disp("Disparity");
	StereoDepth stereoDepth;
	char c = 0;
	Mat img1,img2;
	/*
	while(c!=27){
		cam.grabing();
		cam.retrieving(img, CV_8UC3);
		screen.putImage(img);
		switch(c){
			case '1':
				stereoDepth.setImage1(img);
				image1.putImage(img);
				break;
			case '2':
				stereoDepth.setImage2(img);
				image2.putImage(img);
				break;
			case '3':
				if(stereoDepth.doDepth()){
					disp.putImage(stereoDepth.getDisparity());
				}break;
		}
		
		
		c = waitKey(10);
	}
	*/
	img1 = imread( argv[1] );
	img2 = imread( argv[2] );
	
	image1.putImage(img1);
	image2.putImage(img2);
	waitKey(0);

	stereoDepth.setImage1(img1);
	stereoDepth.setImage2(img2);
	stereoDepth.doDepth();
	disp.putImage(stereoDepth.getDisparity());
	
	waitKey(0);
}
