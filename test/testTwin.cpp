#include <TwinCamera.hpp>
#include <time.h>
#include <stdlib.h>
#include <string>

int main(int argc, char **argv){
	TwinCamera twin(0,1);
	cv::Mat img1, img2;
	twin.getDoubleImages( img1, img2);
	//twin.loadCameraParameters(argv[1], img1, img2);
	twin.rectifyForStereo(img1,img2);
	std::string filenameseed = std::to_string(time(NULL));
	cv::imwrite(filenameseed + std::string("_left.jpg"), img1);
	cv::imwrite(filenameseed + std::string("right.jpg"), img2);
	return 0;
}
