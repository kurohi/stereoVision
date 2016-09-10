#include <TwinCamera.hpp>
#include <Screen.hpp>
#include <time.h>
#include <stdlib.h>
#include <string>

int main(int argc, char **argv){
	Screen screen_left("left");
	Screen screen_right("right");
	TwinCamera twin(0,1);
	cv::Mat left_img,right_img;
	char c='p';
	while(c!='q'){
		twin.getDoubleImages( left_img, right_img);
		//twin.loadCameraParameters(argv[1], img1, img2);
		//twin.rectifyForStereo(img1,img2);
		screen_left.putImage(left_img);
		screen_right.putImage(right_img);
		c = cv::waitKey(10);
		if(c==' '){
			std::string filenameseed = std::to_string(time(NULL));
			cv::imwrite(filenameseed + std::string("_left.png"), left_img);
			cv::imwrite(filenameseed + std::string("_right.png"), right_img);
			std::cout<<"Image saved!"<<std::endl;
		}
	}
	return 0;
}
