#include "camera.hpp"
#include "screen.hpp"

int main(int argc, char **argv){
	Camera cam1(0);
//	Camera cam2(1);
	cam1.reopen(1);
	Screen screen1("video1");
	Screen screen2("video2");
	char c = 0;
    cv::Mat img1,img2;
	bool left = true;
	while(c!=27){
		if(left){
			cam1.grabing();
			cam1.retrieving(img1,CV_16UC1);
		//	left = false;
			screen1.putImage(img1);
		}else{
		//	cam2.grabing();
		//	cam2.retrieving(img2,CV_16UC1);
		//	left = true;
		//	screen2.putImage(img2);
		}
		c = cv::waitKey(0);
	}
}
